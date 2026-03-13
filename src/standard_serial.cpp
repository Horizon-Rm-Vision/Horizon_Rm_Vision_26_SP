#include <fmt/core.h>

#include <chrono>
#include <cmath>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | ../configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  // UI显示相关变量
  int frame_count = 0;
  float fps = 0.0f;
  auto last_fps_time = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    camera.read(img, t);
    q = gimbal.q(t - 1ms);
    auto gimbal_mode = gimbal.mode();
    // Map GimbalMode to Mode
    if (gimbal_mode == io::GimbalMode::IDLE) mode = io::Mode::idle;
    else if (gimbal_mode == io::GimbalMode::AUTO_AIM) mode = io::Mode::auto_aim;
    else if (gimbal_mode == io::GimbalMode::SMALL_BUFF) mode = io::Mode::small_buff;
    else if (gimbal_mode == io::GimbalMode::BIG_BUFF) mode = io::Mode::big_buff;
    else mode = io::Mode::idle;  // default

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }

    // recorder.record(img, q, t);

    solver.set_R_gimbal2world(q);

    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto bullet_speed = gimbal.state().bullet_speed;

    auto armors = detector.detect(img);

    auto targets = tracker.track(armors, t);

    auto command = aimer.aim(targets, t, bullet_speed);

    // 在图像上绘制检测结果（合并原 detection 窗口信息）
    for (const auto & armor : armors) {
      // 画装甲四点与标签
      tools::draw_points(img, armor.points, {0, 255, 0});
      auto info = fmt::format("{:.2f} {} {} {}", armor.confidence, auto_aim::COLORS[armor.color], auto_aim::ARMOR_NAMES[armor.name], auto_aim::ARMOR_TYPES[armor.type]);
      tools::draw_text(img, info, armor.center, {0, 255, 0});
    }

    if (!targets.empty()) {
      auto target = targets.front();
      // 当前帧target更新后
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points = solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }
      // aimer瞄准位置
      auto aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid) tools::draw_points(img, image_points, {0, 0, 255});
    }

    // 计算FPS
    frame_count++;
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_fps_time).count();
    if (elapsed_time >= 1000) {
      fps = static_cast<float>(frame_count) * 1000.0f / static_cast<float>(elapsed_time);
      frame_count = 0;
      last_fps_time = current_time;
    }

    // 绘制UI信息
    int y_offset = 30;
    int line_height = 25;
    cv::Scalar text_color(0, 255, 0);
    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.6;
    int thickness = 2;

    // FPS
    std::string fps_text = fmt::format("FPS: {:.1f}", fps);
    cv::putText(img, fps_text, cv::Point(10, y_offset), font_face, font_scale, text_color, thickness);
    y_offset += line_height;

    // 运行模式
    std::string mode_text = fmt::format("Mode: {}", io::MODES[mode]);
    cv::putText(img, mode_text, cv::Point(10, y_offset), font_face, font_scale, text_color, thickness);
    y_offset += line_height;

    // 目标检测状态
    bool has_target = !targets.empty();
    std::string detect_text = fmt::format("Detect: {}", has_target ? "YES" : "NO");
    cv::putText(img, detect_text, cv::Point(10, y_offset), font_face, font_scale, 
                has_target ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), thickness);
    y_offset += line_height;

    // // 开火状态
    // std::string fire_text = fmt::format("Fire: {}", plan.fire ? "YES" : "NO");
    // cv::putText(img, fire_text, cv::Point(10, y_offset), font_face, font_scale, 
    //             plan.fire ? cv::Scalar(0, 0, 255) : text_color, thickness);
    // y_offset += line_height;

    // 云台状态
    auto gs = gimbal.state();
    std::string gimbal_status = fmt::format("Gimbal Yaw: {:.2f}  Pitch: {:.2f}", gs.yaw * 180.0 / std::acos(-1.0), gs.pitch * 180.0 / std::acos(-1.0));
    cv::putText(img, gimbal_status, cv::Point(10, y_offset), font_face, font_scale, text_color, thickness);
    y_offset += line_height;

    // 子弹速度
    std::string bullet_speed_text = fmt::format("Bullet Speed: {:.1f}", bullet_speed);
    cv::putText(img, bullet_speed_text, cv::Point(10, y_offset), font_face, font_scale, text_color, thickness);
    y_offset += line_height;

    // 目标数量
    std::string target_count_text = fmt::format("Targets: {}", targets.size());
    cv::putText(img, target_count_text, cv::Point(10, y_offset), font_face, font_scale, 
                targets.empty() ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), thickness);
    y_offset += line_height;

    // 装甲板数量
    std::string armor_count_text = fmt::format("Armors: {}", armors.size());
    cv::putText(img, armor_count_text, cv::Point(10, y_offset), font_face, font_scale, 
                armors.empty() ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), thickness);

    // 显示图像
    cv::namedWindow("Vision System", cv::WINDOW_NORMAL);
    cv::imshow("Vision System", img);
    int key = cv::waitKey(1);
    if (key == 'q' || key == 27) {  // q 或 ESC 退出
      break;
    }

    plotter.drawData({gs.yaw * 180/M_PI, command.yaw * 180/M_PI}, {"gimbal_yaw", "target_yaw"});
    gimbal.send(has_target, command.shoot, command.yaw, 0, 0, command.pitch, 0, 0);
  }

  return 0;
}