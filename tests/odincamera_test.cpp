#include "io/odincamera/odincamera.hpp"

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ?        |                           | 输出命令行参数说明}"
  "{topic                 | /odin1_0/image/undistorted | Odin 相机话题名 }"
  "{@config-path          | ../configs/sentry_odin.yaml | 位置参数，yaml配置文件路径 }"
  "{d display             |                           | 显示视频流       }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  tools::Exiter exiter;

  auto config_path = cli.get<std::string>(0);
  auto topic_name = cli.get<std::string>("topic");
  auto display = cli.has("display");

  // 初始化 ROS2（OdinCamera 内部使用 rclcpp 订阅话题）
  rclcpp::init(0, nullptr);

  {
    io::OdinCamera odin_cam(topic_name, config_path);

    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
    auto last_stamp = std::chrono::steady_clock::now();

    tools::logger()->info(
      "[odincamera_test] 开始接收 {} 图像（按 Ctrl+C 退出）", topic_name);

    while (!exiter.exit()) {
      odin_cam.read(img, timestamp);

      auto dt = tools::delta_time(timestamp, last_stamp);
      last_stamp = timestamp;

      tools::logger()->info("[odincamera_test] {:.2f} fps", 1 / dt);
      
      //./odincamera_test -d 即可启动图像
      if (!display) continue;
      cv::imshow("odin_cam", img);
      if (cv::waitKey(1) == 'q') break;
    }
  }  // odin_cam 析构，spin 线程安全退出

  rclcpp::shutdown();
  return 0;
}
