#ifndef TOOLS__UDP_MARKER_BRIDGE_HPP
#define TOOLS__UDP_MARKER_BRIDGE_HPP

#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <nlohmann/json.hpp>

#include <atomic>
#include <cstdint>
#include <optional>
#include <string>
#include <thread>

namespace tools
{

class UdpMarkerBridgeNode final : public rclcpp::Node
{
public:
  explicit UdpMarkerBridgeNode(const rclcpp::NodeOptions & options);
  ~UdpMarkerBridgeNode() override;

private:
  static std::optional<double> get_number(const nlohmann::json & obj, const char * key);

  visualization_msgs::msg::Marker make_base_marker(int32_t id, const std::string & ns, int32_t type) const;
  static void set_color_rgba(visualization_msgs::msg::Marker & m, float r, float g, float b, float a);

  void publish_from_json(const nlohmann::json & j, const std::string & raw);
  void rxLoop();

private:
  std::string bind_address_;
  uint16_t port_{};

  std::string marker_topic_;
  std::string frame_id_;

  double position_scale_{0.12};
  double velocity_scale_{0.5};
  double yawrate_scale_z_{0.2};

  // Armor cubes (generated from target state)
  bool show_armors_{true};
  int default_target_armor_num_{4};
  double armor_pitch_rad_{0.2618};
  double armor_scale_x_{0.03};
  double armor_scale_y_small_{0.135};
  double armor_scale_y_big_{0.23};
  double armor_scale_z_{0.125};

  bool show_armor_measurement_{true};
  bool show_text_{true};
  int marker_lifetime_ms_{200};

  int socket_fd_{-1};
  std::atomic<bool> stop_{false};
  std::thread rx_thread_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace tools

#endif  // TOOLS__UDP_MARKER_BRIDGE_HPP
