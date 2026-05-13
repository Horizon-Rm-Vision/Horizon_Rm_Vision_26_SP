#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <list>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_tracker.hpp"
#include "tasks/auto_buff/buff_type.hpp"
#include "tools/exiter.hpp"
#include "tools/ui_manager.hpp"
#include "tools/ui_web_stream.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"
#include "tools/ui_manager.hpp"
#include "tools/yaml.hpp"
#include "tools/recorder.hpp"
#include "tools/pose_buffer.hpp"
#include <yaml-cpp/yaml.h>

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
  // 初始化绘图器、录制器、退出器
  tools::Plotter plotter;
  tools::Recorder recorder;
  tools::Exiter exiter;

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  auto recorder_config = tools::load(config_path);
  bool enable_recorder = recorder_config["recorder"] ? recorder_config["recorder"].as<bool>() : false;

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
  tools::UIWebStream ui_web_stream(config_path);
  plotter.configureWebStreamFromConfig(config_path);
  ui_manager.setProgramMode("AutoAim MPC");

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);
  ui_web_stream.setExposureHandler([&camera](double exposure_ms) {
    camera.setExposureMs(exposure_ms);
  });

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  auto_buff::Buff_Detector buff_detector(config_path);
  auto_buff::Solver buff_solver(config_path);
  auto_buff::BuffTracker buff_tracker(config_path, auto_buff::SMALL);
  auto_buff::Aimer buff_aimer(config_path);

  // 运动延时补偿 (pose buffer, 移植自 auto_buff_debug_mpc)
  double motion_delay_ms = 0.0;
  size_t pose_buffer_size = 200;
  {
    auto yaml = YAML::LoadFile(config_path);
    if (yaml["buff_tracker"].IsDefined()) {
      auto node = yaml["buff_tracker"];
      motion_delay_ms = node["motion_delay_ms"].as<double>(motion_delay_ms);
      pose_buffer_size = node["pose_buffer_size"].as<int>(static_cast<int>(pose_buffer_size));
    }
  }
  tools::PoseBuffer pose_buffer(pose_buffer_size);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  #ifdef FIRE_CONSTRAINT
  // 记录初始云台yaw
  auto initial_gimbal_state = gimbal.state();
  double initial_yaw = initial_gimbal_state.yaw;
  #endif

  std::atomic<bool> quit = false;
  std::atomic<int64_t> last_time_ns{0};
  
  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;

    while (!quit) {
      auto target = target_queue.front();
      auto gs = gimbal.state();
      auto mode = gimbal.mode();
      #ifdef SENTRY_SR
      auto velocity = ros2.get_nav_velocity();
      auto form = ros2.subscribe_form();
      auto gimbal_form = ros2.get_gimbal_form();
      int8_t gimbal_form_value = gimbal_form ? gimbal_form->data : 0;
      #endif
      if (mode == io::GimbalMode::SMALL_BUFF || mode == io::GimbalMode::BIG_BUFF) {
        std::this_thread::sleep_for(10ms);
        continue;
      }

      auto plan = planner.plan(target, gs.bullet_speed);

      auto time_ns = last_time_ns.load();
      auto plan_time = (time_ns > 0)
        ? std::chrono::steady_clock::time_point(std::chrono::nanoseconds(time_ns))
        : std::chrono::steady_clock::now();

      #ifdef FIRE_CONSTRAINT
      if (mode == io::GimbalMode::AUTO_AIM || mode == io::GimbalMode::IDLE) {
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
      }
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
      plotter.setWindowName("MPC Debug");
      plotter.subplot("Yaw", {gs.yaw * 180/M_PI, plan.target_yaw * 180/M_PI, plan.yaw * 180/M_PI},
                      {"gimbal_yaw", "target_yaw", "plan_yaw"});
      plotter.subplot("Pitch", {gs.pitch * 180/M_PI, plan.target_pitch * 180/M_PI, plan.pitch * 180/M_PI},
                      {"gimbal_pitch", "target_pitch", "plan_pitch"});
      //plotter.subplot("Yaw Vel", {gs.yaw_vel, plan.yaw_vel},
                      //{"gimbal_yaw_vel", "plan_yaw_vel"});
      //plotter.subplot("Acc", {plan.yaw_acc, plan.pitch_acc},
                      //{"plan_yaw_acc", "plan_pitch_acc"});
      // plotter.draw();

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
    last_time_ns.store(std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count());
    ui_web_stream.sendImage(img);
    ui_web_stream.beginFrame(img.cols, img.rows);
    auto mode = gimbal.mode();
    auto q = gimbal.q(t);
    pose_buffer.push(q, t);
    if (mode == io::GimbalMode::SMALL_BUFF || mode == io::GimbalMode::BIG_BUFF) {
      auto q_sample = pose_buffer.sample(t - std::chrono::milliseconds(static_cast<int>(motion_delay_ms)));
      if (q_sample.has_value()) {
        q = q_sample.value();
      }
    }
    auto gs = gimbal.state();
    if (enable_recorder) recorder.record(img, q, t);

    if (mode == io::GimbalMode::BIG_BUFF) {
      buff_detector.setBig2026Mode(true);
      buff_tracker.set_type(auto_buff::BIG);
    } else if (mode == io::GimbalMode::SMALL_BUFF) {
      buff_detector.setBig2026Mode(false);
      buff_tracker.set_type(auto_buff::SMALL);
    }

    solver.set_R_gimbal2world(q);
    buff_solver.set_R_gimbal2world(q);

    auto armors = std::list<auto_aim::Armor>();
    auto targets = std::list<auto_aim::Target>();
    std::optional<auto_buff::PowerRune> power_runes = std::nullopt;
    bool buff_found = false;
    std::unique_ptr<auto_buff::Target> buff_target_copy = nullptr;
    auto_aim::Plan buff_plan = {false, false, 0, 0, 0, 0, 0, 0, 0, 0};

    if (mode == io::GimbalMode::AUTO_AIM || mode == io::GimbalMode::IDLE) {
      armors = yolo.detect(img);
      targets = tracker.track(armors, t);
    } else if (mode == io::GimbalMode::SMALL_BUFF || mode == io::GimbalMode::BIG_BUFF) {
      power_runes = buff_detector.detect(img);
      buff_found = buff_tracker.update(power_runes, t, buff_solver);
      auto target_copy = buff_tracker.clone_target();
      if (buff_found && target_copy) {
        buff_target_copy = std::move(target_copy);
        buff_plan = buff_aimer.mpc_aim(*buff_target_copy, t, gs, true);
      }
    }

    #ifdef AIM_CENTER
    if(tracker.aim_strategy_ == "follow") {
      planner.aim_center_ = false;
    }
    else {
      planner.aim_center_ = true;
    }
    #endif

    if (mode == io::GimbalMode::AUTO_AIM || mode == io::GimbalMode::IDLE) {
      if (!targets.empty())
        target_queue.push(targets.front());
      else
        target_queue.push(std::nullopt);
    } else {
      target_queue.push(std::nullopt);
    }

    // 在图像上绘制检测结果（合并原 detection 窗口信息）
    if (mode == io::GimbalMode::AUTO_AIM || mode == io::GimbalMode::IDLE) {
      for (const auto & armor : armors) {
        // 画装甲四点与标签
        tools::draw_points(img, armor.points, {0, 255, 0});
        auto info = fmt::format("{:.2f} {} {} {}", armor.confidence, auto_aim::COLORS[armor.color], auto_aim::ARMOR_NAMES[armor.name], auto_aim::ARMOR_TYPES[armor.type]);
        tools::draw_text(img, info, armor.center, {0, 255, 0});
        // 绘制世界坐标系数值
        tools::draw_text(img, fmt::format("armor_x: {:.2f}", armor.xyz_in_world[0]), {10, 600}, {0, 255, 0});
        tools::draw_text(img, fmt::format("armor_y: {:.2f}", armor.xyz_in_world[1]), {10, 630}, {0, 255, 0});
        tools::draw_text(img, fmt::format("armor_z: {:.2f}", armor.xyz_in_world[2]), {10, 660}, {0, 255, 0});
      }
    }
    #ifdef AIM_CENTER
    // 绘制锁定中心
    if (planner.aim_center_) {
      auto center_image_points = solver.reproject_point(planner.center_points);
      tools::draw_points(img, center_image_points, {0, 0, 255}, 10);
    }
    #endif

    if (mode == io::GimbalMode::AUTO_AIM || mode == io::GimbalMode::IDLE) {
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
        if (planner.aim_center_ == false) {
          tools::draw_points(img, image_points, {0, 0, 255});
        }
        #endif
      }
    } else if (mode == io::GimbalMode::SMALL_BUFF || mode == io::GimbalMode::BIG_BUFF) {
      #ifndef SENTRY_SR
      gimbal.send(
        buff_plan.control, buff_plan.fire, buff_plan.yaw, buff_plan.yaw_vel, buff_plan.yaw_acc,
        buff_plan.pitch, buff_plan.pitch_vel, buff_plan.pitch_acc);
      #endif
      #ifdef SENTRY_SR
      auto velocity = ros2.get_nav_velocity();
      auto form = ros2.subscribe_form();
      auto gimbal_form = ros2.get_gimbal_form();
      int8_t gimbal_form_value = gimbal_form ? gimbal_form->data : 0;
      gimbal.send(
        buff_plan.control, buff_plan.fire, buff_plan.yaw, buff_plan.yaw_vel, buff_plan.yaw_acc,
        buff_plan.pitch, buff_plan.pitch_vel, buff_plan.pitch_acc,
        velocity->linear.x * velocity_n, velocity->linear.y * velocity_n, velocity->angular.z,
        form.data, gimbal_form_value);
      #endif
      if (buff_found) {
        auto selected = buff_tracker.last_observation();
        if (selected.has_value()) {
          auto & p = selected.value();
          for (int i = 0; i < 4; i++) tools::draw_point(img, p.target().points[i]);
          tools::draw_point(img, p.target().center, {0, 0, 255}, 3);
          tools::draw_point(img, p.r_center, {0, 0, 255}, 3);

          auto & target_ref = buff_tracker.target();
          auto Rxyz_in_world_now = target_ref.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
          auto image_points = buff_solver.reproject_buff(
            Rxyz_in_world_now, target_ref.ekf_x()[4], target_ref.ekf_x()[5]);
          tools::draw_points(
            img, std::vector<cv::Point2f>(image_points.begin(), image_points.begin() + 4), {0, 255, 0});
          tools::draw_points(
            img, std::vector<cv::Point2f>(image_points.begin() + 4, image_points.end()), {0, 255, 0});

          if (buff_target_copy) {
            auto Rxyz_in_world_pre = target_ref.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
            auto image_points_pre = buff_solver.reproject_buff(
              Rxyz_in_world_pre, buff_target_copy->ekf_x()[4], buff_target_copy->ekf_x()[5]);
            tools::draw_points(
              img, std::vector<cv::Point2f>(image_points_pre.begin(), image_points_pre.begin() + 4), {255, 0, 0});
            tools::draw_points(
              img, std::vector<cv::Point2f>(image_points_pre.begin() + 4, image_points_pre.end()), {255, 0, 0});
          }
        }
      }
    }
    
    // 获取云台状态和规划信息用于UI显示
    auto mode_for_ui = gimbal.mode();
    std::optional<auto_aim::Target> target_opt = target_queue.front();
    bool has_target = target_opt.has_value();
    auto plan = planner.plan(target_opt, gs.bullet_speed);
    if (mode_for_ui == io::GimbalMode::SMALL_BUFF || mode_for_ui == io::GimbalMode::BIG_BUFF) {
      plan = buff_plan;
    }
    
    #ifdef SENTRY_SR
    //发布导航的信息
    ros2.publish_status(gs.game_progress,gs.stage_remain_time,gs.current_hp,gs.ally_outpost_hp,gs.state,gs.energy_state,gs.bullets);
    #endif

    // UI初始化和配置
    ui_manager.setProgramMode(fmt::format("{}", gimbal.str(mode_for_ui)));
    ui_manager.initialize(img);
    
    // 添加左侧UI元素
    bool buff_detected = buff_found;
    auto detect_text = (mode_for_ui == io::GimbalMode::AUTO_AIM || mode_for_ui == io::GimbalMode::IDLE)
      ? (has_target ? "YES" : "NO")
      : (buff_detected ? "YES" : "NO");
    auto detect_color = (mode_for_ui == io::GimbalMode::AUTO_AIM || mode_for_ui == io::GimbalMode::IDLE)
      ? (has_target ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255))
      : (buff_detected ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255));
    ui_manager.addLeftText("detect", fmt::format("Detect: {}", detect_text), detect_color);
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
    ui_manager.addLeftText("state", fmt::format("State: {} ", (int)gs.state));
    ui_manager.addLeftText("energy_state", fmt::format("  Energy State: {} ", (int)gs.energy_state));
    ui_manager.addLeftText("bullets", fmt::format("  Bullets: {} ", (int)gs.bullets));
    #endif
    
    // 目标信息
    if (mode_for_ui == io::GimbalMode::AUTO_AIM || mode_for_ui == io::GimbalMode::IDLE) {
      if (has_target) {
        auto & target = target_opt.value();
        ui_manager.addLeftText("target_info", fmt::format("Target Z: {:.2f}  Vz: {:.2f}", target.ekf_x()[4], target.ekf_x()[5]), cv::Scalar(255, 255, 0));
        if (target.ekf_x().size() > 7) {
          ui_manager.addLeftText("target_w", fmt::format("Target W: {:.2f}", target.ekf_x()[7]), cv::Scalar(255, 255, 0));
        }
      }
    } else if (mode_for_ui == io::GimbalMode::SMALL_BUFF || mode_for_ui == io::GimbalMode::BIG_BUFF) {
      if (buff_found) {
        auto & target_ref = buff_tracker.target();
        Eigen::VectorXd x = target_ref.ekf_x();
        if (x.size() >= 6) {
          ui_manager.addLeftText("buff_angle", fmt::format("Angle: {:.1f}  Spd: {:.2f}", x[5] * 57.3, x[6]), cv::Scalar(255, 255, 0));
        }
      }
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
    ui_web_stream.capturePanels(ui_manager);
    ui_web_stream.sendFrame();

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
