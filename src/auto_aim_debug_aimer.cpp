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
#include "tools/ui_manager.hpp"
#include "tools/ui_web_stream.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/ui_manager.hpp"
#include "tools/yaml.hpp"

#ifdef SENTRY_SR
#include "io/ros2/publish2nav.hpp"
#include "io/ros2/ros2.hpp"
#endif

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

  auto yaml_config = tools::load(config_path);
  bool enable_recorder = yaml_config["recorder"] ? yaml_config["recorder"].as<bool>() : false;

  auto last_t = std::chrono::steady_clock::now();

  #ifdef SENTRY_SR
  auto yaml = YAML::LoadFile(config_path);
  auto velocity_n = yaml["velocity_n"].as<int>();
  io::ROS2 ros2;
  #endif

  #ifdef FIRE_CONSTRAINT
  // 读取开火约束配置
  #ifndef SENTRY_SR
  auto yaml = YAML::LoadFile(config_path);
  #endif
  double gimbal_yaw_threshold = yaml["gimbal_yaw_threshold"].as<double>() / 180.0 * M_PI; // degree to rad
  double target_distance_threshold = yaml["target_distance_threshold"].as<double>();
  #endif

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

  tools::UIManager ui_manager(config_path);
  tools::UIWebStream ui_web_stream(config_path);
  ui_manager.setProgramMode("AutoAim AIMER");

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  while (!exiter.exit()) {
    // UI FPS更新
    ui_manager.updateFPS();
    
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_t).count();
    last_t = now;
    tools::logger()->info("[FPS] {:.1f}", 1.0 / dt);
    
    camera.read(img, t);
    ui_web_stream.sendImage(img);
    ui_web_stream.beginFrame(img.cols, img.rows);
    q = gimbal.q(t);
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

    if (enable_recorder) recorder.record(img, q, t);

    solver.set_R_gimbal2world(q);

    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto bullet_speed = gimbal.state().bullet_speed;

    auto armors = detector.detect(img);

    auto targets = tracker.track(armors, t);

    auto command = aimer.aim(targets, t, bullet_speed);

    // 使用Shooter决定开火（锁中心约束已集成在Shooter内部）
    Eigen::Vector3d gimbal_pos = ypr;
    command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);

    //#ifdef SHOW_UI
    for (const auto & armor : armors) {
      // 画装甲四点与标签
      tools::draw_points(img, armor.points, {0, 255, 0});
      auto info = fmt::format("{:.2f} {} {} {}", armor.confidence, auto_aim::COLORS[armor.color], auto_aim::ARMOR_NAMES[armor.name], auto_aim::ARMOR_TYPES[armor.type]);
      tools::draw_text(img, info, armor.center, {0, 255, 0});
      // 绘制世界坐标系数值
      tools::draw_text(img, fmt::format("armor_x: {:.2f}", armor.xyz_in_world[0]), {10, 600}, {0, 255, 0});
      tools::draw_text(img, fmt::format("armor_y: {:.2f}", armor.xyz_in_world[1]), {10, 630}, {0, 255, 0});
      tools::draw_text(img, fmt::format("armor_z: {:.2f}", armor.xyz_in_world[2]), {10, 660}, {0, 255, 0});
      #ifdef NOVA_Q
      tools::draw_text(img, fmt::format("queue_size: {}", gimbal.q_size()), {10, 720}, {0, 255, 0});
      #endif
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
      #ifndef NOVA_AIM_CENTER
      if (aim_point.valid) tools::draw_points(img, image_points, {0, 0, 255});
      #endif

      #ifdef NOVA_AIM_CENTER
      if (aim_point.valid) {
        // 锁中心模式用不同颜色绘制瞄准点
        auto aim_color = aimer.center_tracked() ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 0, 255);
        tools::draw_points(img, image_points, aim_color);
      }
      #endif
    }

    // UI初始化
    ui_manager.initialize(img);
    
    // 添加左侧UI元素
    bool has_target = !targets.empty();
    ui_manager.addLeftText("detect", fmt::format("Detect: {}", has_target ? "YES" : "NO"), 
                          has_target ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255));
    ui_manager.addLeftText("fire", fmt::format("Fire: {}", command.shoot ? "YES" : "NO"),
                          command.shoot ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0));

    #ifdef NOVA_AIM_CENTER
    // 锁中心状态
    if (!targets.empty()) {
      auto target = targets.front();
      bool is_center_locked = aimer.center_tracked();
      ui_manager.addLeftText("center_lock", fmt::format("Center Lock: {}", is_center_locked ? "ON" : "OFF"),
                            is_center_locked ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 255, 0));
      if (is_center_locked) {
        auto ekf_x = target.ekf_x();
        auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);
        auto r = ekf_x[8];
        auto lock_x = ekf_x[0] - r * std::cos(center_yaw);
        auto lock_y = ekf_x[2] - r * std::sin(center_yaw);
        ui_manager.addLeftText("lock_point", fmt::format("Lock Pt: ({:.2f}, {:.2f})", lock_x, lock_y),
                              cv::Scalar(0, 255, 255));
      }
    }
    #endif

    // 云台状态 - 接收到的数据
    auto gs = gimbal.state();
    #ifdef SENTRY_SR
    auto velocity = ros2.get_nav_velocity();
    auto gimbal_form = ros2.get_gimbal_form();
    auto form = ros2.subscribe_form();
    int8_t gimbal_form_value = gimbal_form ? gimbal_form->data : 0;
    //发布导航的信息
    ros2.publish_status(gs.game_progress, gs.stage_remain_time, gs.current_hp, gs.ally_outpost_hp, gs.state, gs.energy_state,gs.bullets,gs.judge);
    #endif
    ui_manager.addLeftText("gimbal_status", fmt::format("Gimbal Yaw: {:.2f}  Pitch: {:.2f}", -gs.yaw * 180.0 / M_PI, -gs.pitch * 180.0 / M_PI));
    
    //发送的数据
    ui_manager.addLeftText("command_status", fmt::format("Command Yaw: {:.2f}  Pitch: {:.2f}", -command.yaw * 180.0 / M_PI, -command.pitch * 180.0 / M_PI), cv::Scalar(0, 165, 255));
    
    #ifdef SENTRY_SR
    // Sentry SR特有的导航相关数据
    ui_manager.addLeftText("game_progress", fmt::format("Game Status: {} ", (int)gs.game_progress));
    ui_manager.addLeftText("stage_remain_time", fmt::format("Blood: {} ", (int)gs.stage_remain_time));
    ui_manager.addLeftText("current_hp", fmt::format("Bullet: {} ", (int)gs.current_hp));
    ui_manager.addLeftText("ally_outpost_hp", fmt::format("Ally Outpost HP: {} ", (int)gs.ally_outpost_hp));
    ui_manager.addLeftText("state", fmt::format("State: {} ", (int)gs.state));
    ui_manager.addLeftText("energy_state", fmt::format("  Energy State: {} ", (int)gs.energy_state));
    ui_manager.addLeftText("bullets", fmt::format("  Bullets: {} ", (int)gs.bullets));
    #endif
    
    // 添加右侧UI元素
    ui_manager.addRightText("bullet_speed", fmt::format("Bullet Speed: {:.1f}", gs.bullet_speed));
    ui_manager.addRightText("target_count", fmt::format("Targets: {}", targets.size()), 
                           targets.empty() ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0));
    ui_manager.addRightText("armor_count", fmt::format("Armors: {}", armors.size()), 
                           armors.empty() ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0));
    
    #ifdef FIRE_CONSTRAINT
    // 添加is gyro状态到右侧UI
    if (!targets.empty()) {
      auto target = targets.front();
      Eigen::VectorXd x = target.ekf_x();
      bool is_gyro = std::abs(x[8]) > aimer.get_gyro_speed_threshold();
      ui_manager.addRightText("is_gyro", fmt::format("Is Gyro: {}", is_gyro ? "TRUE" : "FALSE"), 
                             is_gyro ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0));
    }
    #endif
    
    // 应用UI绘制
    ui_manager.render(img);
    ui_web_stream.capturePanels(ui_manager);
    ui_web_stream.sendFrame();

    // 显示图像
    if (ui_manager.isImshowEnabled()) {
      cv::namedWindow("Vision System", cv::WINDOW_NORMAL);
      cv::imshow("Vision System", img);
      int key = cv::waitKey(1);
      if (key == 'q' || key == 27) {  // q 或 ESC 退出
        break;
      }
    }
    //#endif

    plotter.subplot("Yaw", {gs.yaw * 180/M_PI, command.yaw * 180/M_PI},
                    {"gimbal_yaw", "target_yaw"});
    plotter.draw();

    #ifdef FIRE_CONSTRAINT
    // 开火约束检查
    bool allow_fire = command.shoot;
    // 云台角度约束
    if (std::abs(command.yaw - gs.yaw) > gimbal_yaw_threshold) {
      allow_fire = false;
    }
    // 目标距离约束
    if (!targets.empty()) {
      auto target = targets.front();
      Eigen::VectorXd x = target.ekf_x();
      double distance = std::sqrt(x[0]*x[0] + x[2]*x[2]); // x[0] is x, x[2] is z
      if (distance > target_distance_threshold) {
        allow_fire = false;
      }
    }
    command.shoot = allow_fire;
    #endif
    
    #ifndef SENTRY_SR
    gimbal.send(has_target, command.shoot, command.yaw, 0, 0, command.pitch, 0, 0);
    #endif
    #ifdef SENTRY_SR
    gimbal.send(has_target, command.shoot, command.yaw, 0, 0, command.pitch, 0, 0, velocity->linear.x*velocity_n, velocity->linear.y*velocity_n, velocity->angular.z, form.data,gimbal_form_value,0);
    #endif
  }

  return 0;
}