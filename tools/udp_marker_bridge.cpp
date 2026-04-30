#include "tools/udp_marker_bridge.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>

namespace tools
{

UdpMarkerBridgeNode::UdpMarkerBridgeNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("udp_marker_bridge", options)
{
  bind_address_ = this->declare_parameter<std::string>("bind_address", "0.0.0.0");
  port_ = static_cast<uint16_t>(this->declare_parameter<int>("port", 9870));

  marker_topic_ = this->declare_parameter<std::string>("marker_topic", "/udp_plot/marker");
  frame_id_ = this->declare_parameter<std::string>("frame_id", "odom");

  position_scale_ = this->declare_parameter<double>("position_scale", 0.12);
  velocity_scale_ = this->declare_parameter<double>("velocity_scale", 0.5);
  yawrate_scale_z_ = this->declare_parameter<double>("yawrate_scale_z", 0.2);

  show_armors_ = this->declare_parameter<bool>("show_armors", true);
  default_target_armor_num_ = this->declare_parameter<int>("default_target_armor_num", 4);
  armor_pitch_rad_ = this->declare_parameter<double>("armor_pitch_rad", 0.2618);
  armor_scale_x_ = this->declare_parameter<double>("armor_scale_x", 0.03);
  armor_scale_y_small_ = this->declare_parameter<double>("armor_scale_y_small", 0.135);
  armor_scale_y_big_ = this->declare_parameter<double>("armor_scale_y_big", 0.23);
  armor_scale_z_ = this->declare_parameter<double>("armor_scale_z", 0.125);

  show_armor_measurement_ = this->declare_parameter<bool>("show_armor_measurement", true);
  show_text_ = this->declare_parameter<bool>("show_text", true);

  marker_lifetime_ms_ = this->declare_parameter<int>("marker_lifetime_ms", 200);

  // RViz Marker/MarkerArray commonly expects reliable (and often works best with transient_local)
  // Using SensorDataQoS (best_effort) can lead to QoS incompatibility where RViz subscribes
  // but receives no messages.
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    marker_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());

  socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    throw std::runtime_error("Failed to create UDP socket");
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = ::htons(port_);

  if (bind_address_ == "0.0.0.0" || bind_address_ == "" || bind_address_ == "*") {
    addr.sin_addr.s_addr = INADDR_ANY;
  } else {
    const auto ip = ::inet_addr(bind_address_.c_str());
    if (ip == INADDR_NONE) {
      ::close(socket_fd_);
      socket_fd_ = -1;
      throw std::runtime_error("Invalid bind_address: " + bind_address_);
    }
    addr.sin_addr.s_addr = ip;
  }

  int yes = 1;
  ::setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  if (::bind(socket_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    const int err = errno;
    ::close(socket_fd_);
    socket_fd_ = -1;
    throw std::runtime_error("Failed to bind UDP socket: " + std::string(std::strerror(err)));
  }

  RCLCPP_INFO(
    this->get_logger(), "Listening UDP on %s:%u, publishing MarkerArray: %s (frame_id=%s)",
    bind_address_.c_str(), static_cast<unsigned>(port_), marker_topic_.c_str(), frame_id_.c_str());

  rx_thread_ = std::thread([this]() { this->rxLoop(); });
}

UdpMarkerBridgeNode::~UdpMarkerBridgeNode()
{
  stop_.store(true);
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
}

std::optional<double> UdpMarkerBridgeNode::get_number(const nlohmann::json & obj, const char * key)
{
  if (!obj.is_object() || !obj.contains(key)) {
    return std::nullopt;
  }
  const auto & v = obj.at(key);
  if (v.is_number_float()) return v.get<double>();
  if (v.is_number_integer()) return static_cast<double>(v.get<int64_t>());
  if (v.is_number_unsigned()) return static_cast<double>(v.get<uint64_t>());
  if (v.is_boolean()) return v.get<bool>() ? 1.0 : 0.0;
  return std::nullopt;
}

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

inline double limit_rad(double a)
{
  // Map to [-pi, pi]
  return std::remainder(a, kTwoPi);
}

inline geometry_msgs::msg::Quaternion quat_from_rpy(double roll, double pitch, double yaw)
{
  // Standard aerospace sequence: roll (X), pitch (Y), yaw (Z)
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);

  geometry_msgs::msg::Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

inline int pick_target_armor_num(const nlohmann::json & j, int default_value)
{
  const char * keys[] = {"target_armor_num", "target_armors_num", "armors_num", "armor_num"};
  for (const char * k : keys) {
    if (!j.is_object() || !j.contains(k)) {
      continue;
    }
    const auto & v = j.at(k);
    if (v.is_number_integer() || v.is_number_unsigned()) {
      const int n = v.get<int>();
      if (n == 3 || n == 4) {
        return n;
      }
    }
  }
  return (default_value == 3 || default_value == 4) ? default_value : 4;
}

inline bool is_big_armor(const nlohmann::json & j)
{
  // Optional sender hint: armor_type: "small"|"big"
  if (j.is_object() && j.contains("armor_type") && j.at("armor_type").is_string()) {
    const auto s = j.at("armor_type").get<std::string>();
    if (s == "big" || s == "large") {
      return true;
    }
    if (s == "small") {
      return false;
    }
  }
  return false;
}
}  // namespace

visualization_msgs::msg::Marker UdpMarkerBridgeNode::make_base_marker(
  int32_t id, const std::string & ns, int32_t type) const
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id_;
  m.header.stamp = this->now();
  m.ns = ns;
  m.id = id;
  m.type = type;
  m.action = visualization_msgs::msg::Marker::ADD;

  if (marker_lifetime_ms_ > 0) {
    m.lifetime = rclcpp::Duration(std::chrono::milliseconds(marker_lifetime_ms_));
  }

  // default orientation
  m.pose.orientation.w = 1.0;
  return m;
}

void UdpMarkerBridgeNode::set_color_rgba(
  visualization_msgs::msg::Marker & m, float r, float g, float b, float a)
{
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;
}

void UdpMarkerBridgeNode::publish_from_json(const nlohmann::json & j, const std::string & raw)
{
  // Expect the sender to provide these keys (from mt_auto_aim_debug.cpp):
  // x,vx,y,vy,z,vz,a,w,r,l,h,last_id
  const auto x = get_number(j, "x");
  const auto y = get_number(j, "y");
  const auto z = get_number(j, "z");
  const auto vx = get_number(j, "vx");
  const auto vy = get_number(j, "vy");
  const auto vz = get_number(j, "vz");
  const auto w = get_number(j, "w");
  const auto r = get_number(j, "r");
  const auto l = get_number(j, "l");
  const auto h = get_number(j, "h");

  // yaw angle in rad
  double yaw_rad = 0.0;
  if (const auto yaw = get_number(j, "yaw")) {
    yaw_rad = yaw.value();
  } else if (const auto a_rad = get_number(j, "a_rad")) {
    yaw_rad = a_rad.value();
  } else if (const auto a_deg = get_number(j, "a")) {
    yaw_rad = a_deg.value() * kPi / 180.0;
  }

  visualization_msgs::msg::MarkerArray array;

    // 0) position sphere
    if (x && y && z) {
      auto pos = make_base_marker(0, "target", visualization_msgs::msg::Marker::SPHERE);
      pos.pose.position.x = *x;
      pos.pose.position.y = *y;
      pos.pose.position.z = *z;
      pos.scale.x = position_scale_;
      pos.scale.y = position_scale_;
      pos.scale.z = position_scale_;
      set_color_rgba(pos, 0.1f, 0.9f, 0.1f, 1.0f);
      array.markers.emplace_back(std::move(pos));
    }

    // 1) linear velocity arrow
    if (x && y && z && vx && vy && vz) {
      auto vmark = make_base_marker(1, "target", visualization_msgs::msg::Marker::ARROW);
      vmark.scale.x = 0.03;  // shaft diameter
      vmark.scale.y = 0.06;  // head diameter
      vmark.scale.z = 0.08;  // head length
      set_color_rgba(vmark, 0.1f, 0.4f, 1.0f, 1.0f);

      geometry_msgs::msg::Point p0;
      p0.x = *x;
      p0.y = *y;
      p0.z = *z;

      geometry_msgs::msg::Point p1 = p0;
      p1.x += (*vx) * velocity_scale_;
      p1.y += (*vy) * velocity_scale_;
      p1.z += (*vz) * velocity_scale_;

      vmark.points.emplace_back(p0);
      vmark.points.emplace_back(p1);
      array.markers.emplace_back(std::move(vmark));
    }

    // 2) yaw rate arrow (draw along +Z)
    if (x && y && z && w) {
      auto ymark = make_base_marker(2, "target", visualization_msgs::msg::Marker::ARROW);
      ymark.scale.x = 0.03;
      ymark.scale.y = 0.06;
      ymark.scale.z = 0.08;
      set_color_rgba(ymark, 1.0f, 0.2f, 0.2f, 1.0f);

      geometry_msgs::msg::Point p0;
      p0.x = *x;
      p0.y = *y;
      p0.z = *z;

      geometry_msgs::msg::Point p1 = p0;
      p1.z += (*w) * yawrate_scale_z_;

      ymark.points.emplace_back(p0);
      ymark.points.emplace_back(p1);
      array.markers.emplace_back(std::move(ymark));
    }

    // 3) armor measurement point (optional)
    if (show_armor_measurement_) {
      const auto ax = get_number(j, "armor_x");
      const auto ay = get_number(j, "armor_y");
      if (ax && ay) {
        auto am = make_base_marker(3, "measurement", visualization_msgs::msg::Marker::SPHERE);
        am.pose.position.x = *ax;
        am.pose.position.y = *ay;
        am.pose.position.z = 0.0;
        am.scale.x = 0.08;
        am.scale.y = 0.08;
        am.scale.z = 0.08;
        set_color_rgba(am, 1.0f, 1.0f, 0.2f, 1.0f);
        array.markers.emplace_back(std::move(am));
      }
    }

    // 10) text panel
    if (show_text_) {
      auto text = make_base_marker(10, "text", visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      text.pose.position.x = x.value_or(0.0);
      text.pose.position.y = y.value_or(0.0);
      text.pose.position.z = z.value_or(0.0) + 0.5;
      text.scale.z = 0.15;
      set_color_rgba(text, 1.0f, 1.0f, 1.0f, 1.0f);

      const auto a = get_number(j, "a");
      const auto r = get_number(j, "r");
      const auto l = get_number(j, "l");
      const auto h = get_number(j, "h");
      const auto nis = get_number(j, "nis");
      const auto nees = get_number(j, "nees");
      const auto gy = get_number(j, "gimbal_yaw");
      const auto gp = get_number(j, "gimbal_pitch");

      char buf[512];
      std::snprintf(
        buf, sizeof(buf),
        "x=%.3f y=%.3f z=%.3f\n"
        "vx=%.3f vy=%.3f vz=%.3f\n"
        "a=%.2fdeg w=%.3f\n"
        "r=%.3f l=%.3f h=%.3f\n"
        "gimbal_yaw=%.2f gimbal_pitch=%.2f\n"
        "nis=%.3f nees=%.3f\n",
        x.value_or(0.0), y.value_or(0.0), z.value_or(0.0),
        vx.value_or(0.0), vy.value_or(0.0), vz.value_or(0.0),
        a.value_or(0.0), w.value_or(0.0),
        r.value_or(0.0), l.value_or(0.0), h.value_or(0.0),
        gy.value_or(0.0), gp.value_or(0.0),
        nis.value_or(0.0), nees.value_or(0.0));

      text.text = buf;
      array.markers.emplace_back(std::move(text));
    }

    // 20) armors cubes generated from target state (same logic as auto_aim::Target::h_armor_xyz)
    if (show_armors_ && x && y && z && r) {
      const int armor_num = pick_target_armor_num(j, default_target_armor_num_);
      const double l_val = l.value_or(0.0);
      const double h_val = h.value_or(0.0);
      const bool big = is_big_armor(j);

      for (int i = 0; i < armor_num; ++i) {
        const double angle = limit_rad(yaw_rad + static_cast<double>(i) * kTwoPi / armor_num);

        double r_use = r.value();
        double armor_z = z.value();

        // 4 armor: long/short axis and height difference
        const bool use_l_h = (armor_num == 4) && (i == 1 || i == 3);
        if (armor_num == 4) {
          if (use_l_h) {
            r_use = r.value() + l_val;
            armor_z = z.value() + h_val;
          }
        }

        // 3 armor (outpost): l/h reused as z deltas to armor1/2
        if (armor_num == 3) {
          armor_z = (i == 0) ? z.value() : (i == 1) ? (z.value() + l_val) : (z.value() + h_val);
        }

        const double armor_x = x.value() - r_use * std::cos(angle);
        const double armor_y = y.value() - r_use * std::sin(angle);

        auto cube = make_base_marker(20 + i, "armors", visualization_msgs::msg::Marker::CUBE);
        cube.pose.position.x = armor_x;
        cube.pose.position.y = armor_y;
        cube.pose.position.z = armor_z;
        cube.pose.orientation = quat_from_rpy(0.0, armor_pitch_rad_, angle);

        cube.scale.x = armor_scale_x_;
        cube.scale.y = big ? armor_scale_y_big_ : armor_scale_y_small_;
        cube.scale.z = armor_scale_z_;

        set_color_rgba(cube, 0.2f, 0.6f, 1.0f, 1.0f);
        array.markers.emplace_back(std::move(cube));
      }
    }

  // If we didn't create anything, don't spam
  if (!array.markers.empty()) {
    marker_pub_->publish(array);
  } else {
    (void)raw;
  }
}

void UdpMarkerBridgeNode::rxLoop()
{
  std::string buffer;
  buffer.resize(65536);

  while (rclcpp::ok() && !stop_.load()) {
    sockaddr_in from{};
    socklen_t from_len = sizeof(from);
    const ssize_t n = ::recvfrom(
      socket_fd_, buffer.data(), buffer.size(), 0, reinterpret_cast<sockaddr *>(&from),
      &from_len);

    if (n <= 0) {
      if (stop_.load()) {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    const std::string payload(buffer.data(), static_cast<size_t>(n));
    auto j = nlohmann::json::parse(payload, nullptr, false);
    if (j.is_discarded() || !j.is_object()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "UDP payload is not valid JSON object (len=%zu)", payload.size());
      continue;
    }

    publish_from_json(j, payload);
  }
}

}  // namespace tools

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<tools::UdpMarkerBridgeNode>(rclcpp::NodeOptions{});
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::fprintf(stderr, "udp_marker_bridge fatal: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
