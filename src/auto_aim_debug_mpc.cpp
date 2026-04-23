#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/ui_manager.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"
#include "tools/ui_manager.hpp"
#include "tools/yaml.hpp"

#ifdef SENTRY_SR
#include "io/ros2/publish2nav.hpp"
#include "io/ros2/ros2.hpp"
#endif


using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | ../configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  // 终端 FPS 显示变量
  auto last_fps_time = std::chrono::steady_clock::now();
  int frame_count = 0;
  float terminal_fps = 0.0f;

  #ifdef SENTRY_SR
  auto yaml = YAML::LoadFile(config_path);
  auto velocity_n = yaml["velocity_n"].as<int>();
  io::ROS2 ros2;
  #endif

  #ifdef FIRE_CONSTRAINT
  // 读取开火约束配置
  auto yaml_config = YAML::LoadFile(config_path);
  double gimbal_yaw_threshold = yaml_config["gimbal_yaw_threshold"].as<double>() / 180.0 * M_PI; // degree to rad
  double target_distance_threshold = yaml_config["target_distance_threshold"].as<double>();
  #endif

  tools::UIManager ui_manager(config_path);
  ui_manager.setProgramMode("AutoAim MPC");

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  #ifdef FIRE_CONSTRAINT
  // 记录初始云台yaw
  auto initial_gimbal_state = gimbal.state();
  double initial_yaw = initial_gimbal_state.yaw;
  #endif

  std::atomic<bool> quit = false;
  
  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;

    while (!quit) {
      auto target = target_queue.front();
      auto gs = gimbal.state();
      #ifdef SENTRY_SR
      auto velocity = ros2.get_nav_velocity();
      auto form = ros2.subscribe_form();
      auto gimbal_form = ros2.get_gimbal_form();
      int8_t gimbal_form_value = gimbal_form ? gimbal_form->data : 0;
      #endif
      auto plan = planner.plan(target, gs.bullet_speed);

      #ifdef FIRE_CONSTRAINT
      // 开火约束检查
      bool allow_fire = plan.fire;
      // 云台角度约束
      if (std::abs(plan.yaw - initial_yaw) > gimbal_yaw_threshold) {
        allow_fire = false;
      }
      // 目标距离约束
      if (target.has_value()) {
        Eigen::VectorXd x = target->ekf_x();
        double distance = std::sqrt(x[0]*x[0] + x[2]*x[2]); // x[0] is x, x[2] is z
        if (distance > target_distance_threshold) {
          allow_fire = false;
        }
      }
      plan.fire = allow_fire;
      #endif

      #ifndef SENTRY_SR
      gimbal.send(
        plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
        plan.pitch_acc);
      #endif
      #ifdef SENTRY_SR
      gimbal.send(
        plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
        plan.pitch_acc,velocity->linear.x*velocity_n,velocity->linear.y*velocity_n,velocity->angular.z,form.data,gimbal_form_value);
      #endif
        
      auto fired = gs.bullet_count > last_bullet_count;
      last_bullet_count = gs.bullet_count;

      nlohmann::json data;
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

      data["gimbal_yaw"] = gs.yaw;
      data["gimbal_yaw_vel"] = gs.yaw_vel;
      data["gimbal_pitch"] = gs.pitch;
      data["gimbal_pitch_vel"] = gs.pitch_vel;

      data["target_yaw"] = plan.target_yaw;
      data["target_pitch"] = plan.target_pitch;

      data["plan_yaw"] = plan.yaw;
      data["plan_yaw_vel"] = plan.yaw_vel;
      data["plan_yaw_acc"] = plan.yaw_acc;

      data["plan_pitch"] = plan.pitch;
      data["plan_pitch_vel"] = plan.pitch_vel;
      data["plan_pitch_acc"] = plan.pitch_acc;

      data["fire"] = plan.fire ? 1 : 0;
      data["fired"] = fired ? 1 : 0;

      if (target.has_value()) {
        data["target_z"] = target->ekf_x()[4];   //z
        data["target_vz"] = target->ekf_x()[5];  //vz
      }

      if (target.has_value()) {
        data["w"] = target->ekf_x()[7];
      } else {
        data["w"] = 0.0;
      }

      //plotter.plot(data);
      plotter.drawData({gs.yaw * 180/M_PI, plan.target_yaw * 180/M_PI, plan.yaw * 180/M_PI}, {"gimbal_yaw", "target_yaw", "plann_yaw"});

      std::this_thread::sleep_for(10ms);
    }
  });

  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    auto loop_start_time = std::chrono::steady_clock::now();
    
    // UI FPS更新
    ui_manager.updateFPS();
    
    // 终端 FPS 计算和显示
    frame_count++;
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_fps_time).count();
    
    if (elapsed_time >= 1000) {
      terminal_fps = static_cast<float>(frame_count) * 1000.0f / static_cast<float>(elapsed_time);
      frame_count = 0;
      last_fps_time = current_time;
      
      // 输出 FPS 到终端
      fmt::print("[FPS] {:.1f}\n", terminal_fps);
    }
    
    camera.read(img, t);
    auto q = gimbal.q(t);

    solver.set_R_gimbal2world(q);
    auto armors = yolo.detect(img);
    auto targets = tracker.track(armors, t);

    #ifdef AIM_CENTER
    if(tracker.aim_strategy_ == "follow") {
      planner.aim_center_ = false;
    }
    else {
      planner.aim_center_ = true;
    }
    #endif

    if (!targets.empty())
      target_queue.push(targets.front());
    else
      target_queue.push(std::nullopt);

    // 在图像上绘制检测结果（合并原 detection 窗口信息）
    for (const auto & armor : armors) {
      // 画装甲四点与标签
      tools::draw_points(img, armor.points, {0, 255, 0});
      auto info = fmt::format("{:.2f} {} {} {}", armor.confidence, auto_aim::COLORS[armor.color], auto_aim::ARMOR_NAMES[armor.name],auto_aim::ARMOR_TYPES[armor.type]);
      tools::draw_text(img, info, armor.center, {0, 255, 0});
      // 绘制世界坐标系数值
      tools::draw_text(img, fmt::format("armor_x: {:.2f}", armor.xyz_in_world[0]), {10, 600}, {0, 255, 0});
      tools::draw_text(img, fmt::format("armor_y: {:.2f}", armor.xyz_in_world[1]), {10, 630}, {0, 255, 0});
      tools::draw_text(img, fmt::format("armor_z: {:.2f}", armor.xyz_in_world[2]), {10, 660}, {0, 255, 0});

    }
    #ifdef AIM_CENTER
    // 绘制锁定中心
    if (planner.aim_center_) {
      auto center_image_points = solver.reproject_point(planner.center_points);
      tools::draw_points(img, center_image_points, {0, 0, 255}, 10);
    }
    #endif

    if (!targets.empty()) {
      auto target = targets.front();
      // 当前帧target更新后
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      Eigen::Vector4d aim_xyza = planner.debug_xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      #ifndef AIM_CENTER
      tools::draw_points(img, image_points, {0, 0, 255});
      #endif
      #ifdef AIM_CENTER
      if(planner.aim_center_ == false){
        tools::draw_points(img, image_points, {0, 0, 255});
      }
      #endif
    }
    
    // 获取云台状态和规划信息用于UI显示
    auto gs = gimbal.state();
    std::optional<auto_aim::Target> target_opt = target_queue.front();
    bool has_target = target_opt.has_value();
    auto plan = planner.plan(target_opt, gs.bullet_speed);
    
    #ifdef SENTRY_SR
    //发布导航的信息
    ros2.publish_status(gs.game_progress,gs.stage_remain_time,gs.current_hp,gs.ally_outpost_hp,gs.x,gs.y,gs.angle,gs.state,gs.energy_state);
    #endif

    // UI初始化和配置
    ui_manager.initialize(img);
    
    // 添加左侧UI元素
    ui_manager.addLeftText("detect", fmt::format("Detect: {}", has_target ? "YES" : "NO"), 
                          has_target ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255));
    ui_manager.addLeftText("fire", fmt::format("Fire: {}", plan.fire ? "YES" : "NO"), 
                          plan.fire ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0));
    ui_manager.addLeftText("gimbal_status", fmt::format("Gimbal Yaw: {:.2f}  Pitch: {:.2f}", -gs.yaw * 180.0 / M_PI, -gs.pitch * 180.0 / M_PI));
    ui_manager.addLeftText("gimbal_vel", fmt::format("Gimbal Vel Y: {:.2f}  P: {:.2f}", -gs.yaw_vel * 180.0 / M_PI, -gs.pitch_vel * 180.0 / M_PI));
    ui_manager.addLeftText("plan_status", fmt::format("Plan Yaw: {:.2f}  Pitch: {:.2f}", -plan.yaw * 180.0 / M_PI, -plan.pitch * 180.0 / M_PI), cv::Scalar(0, 165, 255));
    ui_manager.addLeftText("plan_vel", fmt::format("Plan Vel Y: {:.2f}  P: {:.2f}", -plan.yaw_vel * 180.0 / M_PI, -plan.pitch_vel * 180.0 / M_PI), cv::Scalar(0, 165, 255));
    ui_manager.addLeftText("plan_acc", fmt::format("Plan Acc Y: {:.2f}  P: {:.2f}", plan.yaw_acc, plan.pitch_acc), cv::Scalar(0, 165, 255));
    
    #ifdef SENTRY_SR
    // Sentry SR特有的导航相关数据
    ui_manager.addLeftText("game_progress", fmt::format("Game Status: {} ", (int)gs.game_progress));
    ui_manager.addLeftText("current_hp", fmt::format("Blood: {} ", (int)gs.current_hp));
    ui_manager.addLeftText("ally_outpost_hp", fmt::format("Bullet: {} ", (int)gs.ally_outpost_hp));
    ui_manager.addLeftText("position", fmt::format("Position X: {:.2f}  Y: {:.2f}", gs.x, gs.y));
    ui_manager.addLeftText("angle", fmt::format("Angle: {:.2f}", gs.angle));
    ui_manager.addLeftText("state", fmt::format("State: {} ", (int)gs.state));
    ui_manager.addLeftText("energy_state", fmt::format("  Energy State: {} ", (int)gs.energy_state));
    #endif
    
    // 目标信息
    if (has_target) {
      auto& target = target_opt.value();
      ui_manager.addLeftText("target_info", fmt::format("Target Z: {:.2f}  Vz: {:.2f}", target.ekf_x()[4], target.ekf_x()[5]), cv::Scalar(255, 255, 0));
      
      if (target.ekf_x().size() > 7) {
        ui_manager.addLeftText("target_w", fmt::format("Target W: {:.2f}", target.ekf_x()[7]), cv::Scalar(255, 255, 0));
      }
      
      // Eigen::Vector4d aim_xyza = planner.debug_xyza;
      // ui_manager.addLeftText("armor_xyz", fmt::format("Armor X: {:.2f}  Y: {:.2f}  Z: {:.2f}", aim_xyza[0], aim_xyza[1], aim_xyza[2]), cv::Scalar(255, 255, 0));
    }
    
    #ifdef NOVA_Q
    ui_manager.addLeftText("queue_size", fmt::format("Queue Size: {}", gimbal.q_size()));
    #endif
    
    // 添加右侧UI元素
    ui_manager.addRightText("bullet_speed", fmt::format("Bullet Speed: {:.1f}", gs.bullet_speed));
    ui_manager.addRightText("target_count", fmt::format("Targets: {}", targets.size()), 
                           targets.empty() ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0));
    ui_manager.addRightText("armor_count", fmt::format("Armors: {}", armors.size()), 
                           armors.empty() ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0));
    
    #ifdef AIM_CENTER
    ui_manager.addRightText("aim_strategy", fmt::format("Aim Strategy: {}", tracker.aim_strategy_));
    #endif
    
    // 应用UI绘制
    ui_manager.render(img);

    // cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    if (ui_manager.isImshowEnabled()) {
      cv::namedWindow("reprojection", 0);
      cv::imshow("reprojection", img);
      auto key = cv::waitKey(1);
      if (key == 'q') break;
    }
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();

  #ifndef SENTRY_SR
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
  #endif
  #ifdef SENTRY_SR
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  #endif

  return 0;
}
