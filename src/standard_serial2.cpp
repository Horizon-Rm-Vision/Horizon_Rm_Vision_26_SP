#include <fmt/core.h>

#include <chrono>
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

  while (!exiter.exit()) {
    camera.read(img, t);
    q = gimbal.imu_at(t - 1ms);
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

    // 在图像上绘制检测结果（合并原 detection 窗口信息）
    for (const auto & armor : armors) {
      // 画装甲四点与标签
      tools::draw_points(img, armor.points, {0, 255, 0});
      auto info = fmt::format("{:.2f} {} {} {}", armor.confidence, auto_aim::COLORS[armor.color], auto_aim::ARMOR_NAMES[armor.name],auto_aim::ARMOR_TYPES[armor.type]);
      tools::draw_text(img, info, armor.center, {0, 255, 0});
    }
    
    
    if (!targets.empty()) {
      auto target = targets.front();
      // 当前帧target更新后
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      // Eigen::Vector4d aim_xyza = planner.debug_xyza;
      // auto image_points =
      //   solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      // tools::draw_points(img, image_points, {0, 0, 255});

      // aimer瞄准位置
      auto aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid) tools::draw_points(img, image_points, {0, 0, 255});
    }

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;

    auto command = aimer.aim(targets, t, bullet_speed);

    gimbal.send(command.control, command.shoot, command.yaw, 0, 0, command.pitch, 0, 0);

    //plotter.drawData({gs.yaw * 180/M_PI, plan.target_yaw * 180/M_PI, plan.yaw * 180/M_PI}, {"gimbal_yaw", "target_yaw", "plann_yaw"});
  }

  return 0;
}