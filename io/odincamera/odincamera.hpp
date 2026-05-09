#ifndef IO__ODINCAMERA_HPP
#define IO__ODINCAMERA_HPP

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "../camera.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{

/**
 * @brief Odin1 ROS2 图像订阅相机
 *
 * 订阅 odin_ros_driver 发布的 /odin1_{i}/image/undistorted 话题，
 * 提供与 USBCamera 相同的 read() 接口，使 Perceptron 可直接使用。
 *
 * 配置项（在 sentry_odin.yaml 的 odin_camera: 段）：
 *   topic            - 订阅的话题名（默认 /odin1_0/image/undistorted）
 *   camera_position  - 方位名（left/right/front/back），见 device_name
 *   enable_resize    - 是否缩放图像
 *   resize_width     - 目标宽度
 *   resize_height    - 目标高度
 *
 * 如需改变 Odin 模组安装方位，只需修改 camera_position 字段。
 */
class OdinCamera : public CameraBase
{
public:
  /**
   * @param topic_name  ROS2 图像话题（如 /odin1_0/image/undistorted）
   * @param config_path YAML 配置文件路径（读取 resize / position 等参数）
   */
  OdinCamera(const std::string & topic_name, const std::string & config_path);
  ~OdinCamera();

  /// 阻塞读取最新一帧（线程安全）
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
  struct FrameData
  {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
  };

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

  tools::ThreadSafeQueue<FrameData> queue_;
  std::thread spin_thread_;
  std::promise<void> exit_signal_;
  std::future<void> exit_future_;

  /// resize 配置
  bool enable_resize_ = false;
  int resize_width_ = 1280;
  int resize_height_ = 720;

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
};

}  // namespace io

#endif  // IO__ODINCAMERA_HPP
