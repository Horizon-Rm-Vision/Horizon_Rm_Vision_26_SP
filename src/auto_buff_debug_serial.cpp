#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
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
#include "tools/recorder.hpp"
#include "tools/yaml.hpp"
#include "tools/trajectory.hpp"
#include "tools/pose_buffer.hpp"
#include <yaml-cpp/yaml.h>

// 定义命令行参数
const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | ../configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }
  auto yaml_config = tools::load(config_path);
  bool enable_recorder = yaml_config["recorder"] ? yaml_config["recorder"].as<bool>() : false;

  // 初始化绘图器、录制器、退出器
  tools::Plotter plotter;
  tools::Recorder recorder;
  tools::Exiter exiter;

  // 初始化UIManager和WebStream
  tools::UIManager ui_manager(config_path);
  tools::UIWebStream ui_web_stream(config_path);
  plotter.configureWebStreamFromConfig(config_path);
  ui_manager.setProgramMode("AutoBuff Serial");

  // 终端 FPS 显示变量
  auto last_fps_time = std::chrono::steady_clock::now();
  int frame_count = 0;
  float terminal_fps = 0.0f;

  // 初始化云台、相机
  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);
  ui_web_stream.setExposureHandler([&camera](double exposure_ms) {
    camera.setExposureMs(exposure_ms);
  });

  // 运动延时补偿 (pose buffer, 移植自 MK2)
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

  // 初始化识别器、解算器、追踪器、瞄准器
  auto_buff::Buff_Detector detector(config_path);
  auto_buff::Solver solver(config_path);
  auto_buff::BuffTracker tracker(config_path, auto_buff::SMALL);
  auto_buff::Aimer aimer(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    auto loop_start_time = std::chrono::steady_clock::now();

    camera.read(img, t);
    ui_web_stream.sendImage(img);
    q = gimbal.q(t);
    pose_buffer.push(q, t);
    {
      auto q_sample = pose_buffer.sample(t - std::chrono::milliseconds(static_cast<int>(motion_delay_ms)));
      if (q_sample.has_value()) {
        q = q_sample.value();
      }
    }
    auto gs = gimbal.state();
    if (enable_recorder) recorder.record(img, q, t);

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
      fmt::print("[FPS] {:.1f}\n", terminal_fps);
    }

    ui_web_stream.beginFrame(img.cols, img.rows);

    // -------------- 打符核心逻辑 (BuffTracker + Aimer) --------------

    solver.set_R_gimbal2world(q);

    auto power_runes = detector.detect(img);

    auto found = tracker.update(power_runes, t, solver);

    auto target_copy = tracker.clone_target();

    io::Command cmd = {false, false, 0, 0};
    if (found && target_copy) {
      cmd = aimer.aim(*target_copy, t, gs.bullet_speed, true);
    }

    // 将命令通过串口发送给云台，速度/加速度字段置为 0
    gimbal.send(
      cmd.control, cmd.shoot, static_cast<float>(cmd.yaw), 0.f, 0.f,
      static_cast<float>(cmd.pitch), 0.f, 0.f);

    // -------------- 调试绘制 --------------

    auto selected = tracker.last_observation();
    if (found && selected.has_value() && target_copy) {
      auto & p = selected.value();

      // 显示
      for (int i = 0; i < 4; i++) tools::draw_point(img, p.target().points[i]);
      tools::draw_point(img, p.target().center, {0, 0, 255}, 3);
      tools::draw_point(img, p.r_center, {0, 0, 255}, 3);

      // 当前帧target更新后buff
      auto & target_ref = tracker.target();
      auto Rxyz_in_world_now = target_ref.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
      auto image_points =
        solver.reproject_buff(Rxyz_in_world_now, target_ref.ekf_x()[4], target_ref.ekf_x()[5]);
      tools::draw_points(
        img, std::vector<cv::Point2f>(image_points.begin(), image_points.begin() + 4), {0, 255, 0});
      tools::draw_points(
        img, std::vector<cv::Point2f>(image_points.begin() + 4, image_points.end()), {0, 255, 0});

      // buff瞄准位置(预测)
      double dangle = target_ref.ekf_x()[5] - target_copy->ekf_x()[5];
      auto Rxyz_in_world_pre = target_ref.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
      image_points =
        solver.reproject_buff(Rxyz_in_world_pre, target_copy->ekf_x()[4], target_copy->ekf_x()[5]);
      tools::draw_points(
        img, std::vector<cv::Point2f>(image_points.begin(), image_points.begin() + 4), {255, 0, 0});
      tools::draw_points(
        img, std::vector<cv::Point2f>(image_points.begin() + 4, image_points.end()), {255, 0, 0});
    }

    // -------------- 绘图器数据收集 --------------

    nlohmann::json data;

    if (selected.has_value()) {
      const auto & p = selected.value();
      data["buff_R_yaw"] = p.ypd_in_world[0];
      data["buff_R_pitch"] = p.ypd_in_world[1];
      data["buff_R_dis"] = p.ypd_in_world[2];
      data["buff_yaw"] = p.ypr_in_world[0] * 57.3;
      data["buff_pitch"] = p.ypr_in_world[1] * 57.3;
      data["buff_roll"] = p.ypr_in_world[2] * 57.3;
    }

    if (tracker.is_tracking() && target_copy) {
      auto & target_ref = tracker.target();
      Eigen::VectorXd x = target_ref.ekf_x();
      data["R_yaw"] = x[0];
      data["R_V_yaw"] = x[1];
      data["R_pitch"] = x[2];
      data["R_dis"] = x[3];
      data["yaw"] = x[4] * 57.3;

      data["angle"] = x[5] * 57.3;
      data["spd"] = x[6] * 57.3;
      if (x.size() >= 10) {
        data["spd"] = x[6];
        data["a"] = x[7];
        data["w"] = x[8];
        data["fi"] = x[9];
        data["spd0"] = target_ref.spd;
      }
    }

    // 云台响应情况
    data["gimbal_yaw"] = gs.yaw * 57.3;
    data["gimbal_pitch"] = gs.pitch * 57.3;

    if (cmd.control) {
      data["plan_yaw"] = cmd.yaw * 57.3;
      data["plan_pitch"] = cmd.pitch * 57.3;
      data["shoot"] = cmd.shoot ? 1 : 0;
    }

    plotter.plot(data);

    // -------------- UI渲染 --------------

    ui_manager.initialize(img);

    // 左侧面板：云台状态与cmd数据、旋转信息
    ui_manager.addLeftText("gimbal", fmt::format("Gimbal Y: {:.1f}  P: {:.1f}",
                            -gs.yaw * 57.3, -gs.pitch * 57.3));

    if (cmd.control) {
      ui_manager.addLeftText("cmd", fmt::format("Cmd Y: {:.1f}  P: {:.1f}",
                              -cmd.yaw * 57.3, -cmd.pitch * 57.3), cv::Scalar(0, 165, 255));
    }
    // 大符文姿态与距离
    if (selected.has_value()) {
      const auto & p = selected.value();
      ui_manager.addLeftText("rune_attitude", fmt::format("Rune Y:{:.1f} P:{:.1f} R:{:.1f}",
                            p.ypr_in_world[0] * 57.3, p.ypr_in_world[1] * 57.3, p.ypr_in_world[2] * 57.3));
      ui_manager.addLeftText("rune_distance", fmt::format("Rune Dist: {:.2f}m", p.ypd_in_world[2]));
    }

    bool is_tracking = tracker.is_tracking();
    // EKF目标状态
    if (is_tracking && target_copy) {
      auto & target_ref = tracker.target();
      Eigen::VectorXd x = target_ref.ekf_x();

      // 核心旋转信息：当前角度和角速度
      ui_manager.addLeftText("rotation", fmt::format("Angle: {:.1f}  Spd: {:.2f}",
                            x[5] * 57.3, x[6]), cv::Scalar(255, 255, 0));

      // 扇叶观测器内参
      ui_manager.addLeftText("ekf_r", fmt::format("R_yaw: {:.2f}  R_Vyaw: {:.2f}", x[0], x[1]));
      ui_manager.addLeftText("ekf_rp", fmt::format("R_pitch: {:.2f}  R_dis: {:.2f}", x[2], x[3]));

      // 谐波模型参数 (仅BigTarget)
      if (x.size() >= 10) {
        ui_manager.addLeftText("ekf_harmonic", fmt::format("a:{:.2f}  w:{:.2f}  fi:{:.2f}  spd0:{:.2f}",
                              x[7], x[8], x[9], target_ref.spd), cv::Scalar(0, 165, 255));
      }
    }

    // 右侧面板：云台状态与瞄准指令
    ui_manager.addRightText("bullet_speed", fmt::format("Bullet Speed: {:.1f}", gs.bullet_speed));


    bool has_rune = power_runes.has_value();
    ui_manager.addRightText("detect", fmt::format("Detect: {}", has_rune ? "YES" : "NO"),
                          has_rune ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255));


    ui_manager.addRightText("track", fmt::format("Track: {}", is_tracking ? "YES" : "NO"),
                          is_tracking ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255));

    ui_manager.addRightText("shoot", fmt::format("Shoot: {}", cmd.shoot ? "YES" : "NO"),
                          cmd.shoot ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0));

    ui_manager.render(img);
    ui_web_stream.capturePanels(ui_manager);
    ui_web_stream.sendFrame();

    if (ui_manager.isImshowEnabled()) {
      cv::namedWindow("result", 0);
      cv::imshow("result", img);
      auto key = cv::waitKey(1);
      if (key == 'q') break;
    }
  }

  return 0;
}
