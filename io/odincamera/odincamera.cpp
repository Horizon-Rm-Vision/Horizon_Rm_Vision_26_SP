#include "odincamera.hpp"

#include <cv_bridge/cv_bridge.h>

#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io
{

OdinCamera::OdinCamera(const std::string & topic_name, const std::string & config_path)
: queue_(2), exit_future_(exit_signal_.get_future())
{
  // ---------- 读取 YAML 配置 ----------
  auto yaml = tools::load(config_path);

  if (yaml["odin_camera"]) {
    auto oc = yaml["odin_camera"];

    device_name = tools::read<std::string>(oc, "camera_position");

    enable_resize_ = tools::read<bool>(oc, "enable_resize");
    if (enable_resize_) {
      resize_width_ = tools::read<int>(oc, "resize_width");
      resize_height_ = tools::read<int>(oc, "resize_height");
    }
  } else {
    // 兼容没有 odin_camera 段的老配置
    device_name = "left";
    tools::logger()->warn("[OdinCamera] no 'odin_camera' section in config, using defaults");
  }

  // ---------- 初始化 ROS2 ----------
  // 注意: 假定 rclcpp::init() 已在 main() 中通过 io::ROS2 完成
  // 节点名加入 device_name 防止多实例冲突
  const std::string node_name = "odin_camera_" + device_name;
  node_ = std::make_shared<rclcpp::Node>(node_name);

  // 使用 best_effort + 队列深度 1，只保留最新帧
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.best_effort().durability_volatile();

  sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    topic_name, qos,
    std::bind(&OdinCamera::image_callback, this, std::placeholders::_1));

  // ---------- 启动 spin 线程 ----------
  spin_thread_ = std::thread([this]() {
    try {
      rclcpp::spin_until_future_complete(node_, exit_future_);
    } catch (const std::exception & e) {
      tools::logger()->error("[OdinCamera] spin thread exception: {}", e.what());
    }
  });

  tools::logger()->info(
    "[OdinCamera] subscribed to '{}' as '{}', resize={} ({}x{})",
    topic_name, device_name, enable_resize_, resize_width_, resize_height_);
}

OdinCamera::~OdinCamera()
{
  // 通知 spin 线程退出（通过 future）
  exit_signal_.set_value();
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }

  tools::logger()->info("[OdinCamera] '{}' destroyed", device_name);
}

void OdinCamera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  FrameData data;
  queue_.pop(data);  // 阻塞直到有新帧

  img = data.img;
  timestamp = data.timestamp;
}

void OdinCamera::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    // ROS Image -> cv::Mat
    auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

    FrameData data;
    data.img = cv_ptr->image;

    // 可选缩放
    if (enable_resize_ && (data.img.cols != resize_width_ || data.img.rows != resize_height_)) {
      cv::resize(data.img, data.img, cv::Size(resize_width_, resize_height_));
    }

    data.timestamp = std::chrono::steady_clock::now();

    queue_.push(data);
  } catch (const std::exception & e) {
    tools::logger()->error("[OdinCamera] callback error: {}", e.what());
  }
}

}  // namespace io
