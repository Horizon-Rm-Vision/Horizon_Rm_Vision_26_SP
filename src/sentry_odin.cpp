/**
 * @file sentry_odin.cpp
 * @brief 支持 Odin1 模组的哨兵全向感知主程序（ROS2 Humble）
 *
 * 与 sentry_multithread.cpp 功能相同，但全向感知的全向相机支持使用
 * odin_ros_driver 输出的去畸变图像 (ROS2 话题) 代替 USB 相机。
 *
 * 使用方式：
 *   1. 启动 Odin 驱动: ros2 launch odin_ros_driver odin1_ros2.launch.py
 *   2. 运行本程序:     ./sentry_odin ../configs/sentry_odin.yaml
 *
 * 全向相机配置：
 *   - 默认 1 个 Odin 模块（安装在 left 方位），话题 /odin1_0/image/undistorted
 *   - 可在 sentry_odin.yaml 的 odin_camera 段中修改 topic / camera_position
 *   - 如需更多全向方向，可添加 USBCamera 或更多 OdinCamera 实例
 */

#include <fmt/core.h>

#include <chrono>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "io/odincamera/odincamera.hpp"
#include "io/ros2/ros2.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tasks/omniperception/perceptron.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | ../configs/sentry_odin.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);

  // ========== 初始化 ==========
  io::ROS2 ros2;
  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);  // 主云台相机（工业相机）
  io::Camera back_camera("../configs/camera.yaml");  // 后方相机（如需要）

  // ========== Odin 全向相机 ==========
  // 读取配置确定 Odin 话题名，默认 /odin1_0/image/undistorted
  // 参见 configs/sentry_odin.yaml 中 odin_camera 段
  auto yaml_config = tools::load(config_path);
  std::string odin_topic = "/odin1_0/image/undistorted";
  if (yaml_config["odin_camera"] && yaml_config["odin_camera"]["topic"]) {
    odin_topic = yaml_config["odin_camera"]["topic"].as<std::string>();
  }

  io::OdinCamera odin_cam(odin_topic, config_path);

  // ========== 可选的 USB 相机（补充其他全向方位） ==========
  // 当 Odin 模组只有一个时, 其他方向仍可使用 USB 相机
  // 如果某个方向没有相机, 传 nullptr 即可
  io::USBCamera usbcam1("video0", config_path);   // e.g. right 方向
  io::USBCamera usbcam2("video2", config_path);   // e.g. front 方向
  io::USBCamera usbcam3("video4", config_path);   // e.g. back 方向
  // ──────────────────────────────────────────────────────────
  // 提示: 如果有设备路径冲突，可在上方修改 "videoX" 的编号
  // 或用 nullptr 跳过某一路:
  //   io::USBCamera* usbcam2_ptr = nullptr; // 或者直接传 nullptr
  // ──────────────────────────────────────────────────────────

  // ========== 全向感知 ==========
  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  omniperception::Decider decider(config_path);

  // 全向感知执行器 —— 4 路相机并行 YOLO 推理
  omniperception::Perceptron perceptron(&odin_cam, &usbcam1, &usbcam2, &usbcam3, config_path);

  // ========== 主循环 ==========
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  io::Command last_command;
  auto last_t = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    camera.read(img, timestamp);
    Eigen::Quaterniond q = gimbal.q(timestamp);
    recorder.record(img, q, timestamp);

    /// 自瞄核心逻辑
    solver.set_R_gimbal2world(q);

    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = yolo.detect(img);

    decider.get_invincible_armor(ros2.subscribe_enemy_status());

    decider.armor_filter(armors);

    decider.set_priority(armors);

    auto detection_queue = perceptron.get_detection_queue();

    decider.sort(detection_queue);

    auto [switch_target, targets] = tracker.track(detection_queue, armors, timestamp);

    io::Command command{false, false, 0, 0};

    /// 全向感知逻辑
    if (tracker.state() == "switching") {
      command.control = switch_target.armors.empty() ? false : true;
      command.shoot = false;
      command.pitch = tools::limit_rad(switch_target.delta_pitch);
      command.yaw = tools::limit_rad(switch_target.delta_yaw + gimbal_pos[0]);
    }

    else if (tracker.state() == "lost") {
      command = decider.decide(detection_queue);
      command.yaw = tools::limit_rad(command.yaw + gimbal_pos[0]);
    }

    else {
      command = aimer.aim(targets, timestamp, gimbal.state().bullet_speed);
    }

    /// 发射逻辑
    command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);

    gimbal.send(command.control, command.shoot, command.yaw, 0.0f, 0.0f, command.pitch, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0);

    /// ROS2 通信
    Eigen::Vector4d target_info = decider.get_target_info(armors, targets);

    ros2.publish(target_info);

    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_t).count();
    last_t = now;
    tools::logger()->info("[FPS] {:.1f}", 1.0 / dt);
  }

  return 0;
}
