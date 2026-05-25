#include <fmt/core.h>

#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

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

const std::string keys =
  "{help h usage ? |                   | 输出命令行参数说明 }"
  "{config-path c  | ../configs/auto_buff_test.yaml | yaml配置文件的路径}"
  "{start-index s  | 0                 | 视频起始帧下标    }"
  "{end-index e    | 0                 | 视频结束帧下标    }"
  "{@input-path    | ../assets/demo/demo  | avi和txt文件的路径}";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");

  tools::Plotter plotter;
  tools::Exiter exiter;

  // 初始化UIManager和WebStream (参考 auto_buff_debug_aimer)
  tools::UIManager ui_manager(config_path);
  tools::UIWebStream ui_web_stream(config_path);
  plotter.configureWebStreamFromConfig(config_path);
  ui_manager.setProgramMode("AutoBuff Replay");

  auto video_path = fmt::format("{}.avi", input_path);
  auto text_path = fmt::format("{}.txt", input_path);
  cv::VideoCapture video(video_path);
  std::ifstream text(text_path);

  auto_buff::Buff_Detector detector(config_path);
  auto_buff::Solver solver(config_path);

  auto yaml = YAML::LoadFile(config_path);
  std::string buff_mode_str = yaml["buff_mode"].as<std::string>("big");
  bool is_big = (buff_mode_str == "big");
  auto_buff::PowerRune_type buff_type = is_big ? auto_buff::BIG : auto_buff::SMALL;
  tools::logger()->info("[auto_buff_test] buff_mode = {}", buff_mode_str);

  auto_buff::BuffTracker tracker(config_path, buff_type);
  auto_buff::Aimer aimer(config_path);

  cv::Mat img, drawing;
  auto t0 = std::chrono::steady_clock::now();

  double last_t = -1;
  auto last_frame_time = std::chrono::steady_clock::now();

  video.set(cv::CAP_PROP_POS_FRAMES, start_index);
  for (int i = 0; i < start_index; i++) {
    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
  }

  // 使用 UIManager 的 FPS 计时
  auto last_ui_time = std::chrono::steady_clock::now();

  for (int frame_count = start_index; !exiter.exit(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;

    auto loop_start = std::chrono::steady_clock::now();

    video.read(img);
    if (img.empty()) break;

    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
    auto timestamp = t0 + std::chrono::microseconds(int(t * 1e6));

    ui_web_stream.sendImage(img);

    /// 自瞄核心逻辑 (BuffTracker + Aimer)

    solver.set_R_gimbal2world({w, x, y, z});

    auto detector_start = std::chrono::steady_clock::now();
    auto power_runes = detector.detect_24(img);

    auto tracker_start = std::chrono::steady_clock::now();
    auto found = tracker.update(power_runes, timestamp, solver);

    auto target_copy = tracker.clone_target();

    auto aim_start = std::chrono::steady_clock::now();
    io::Command command = {false, false, 0, 0};
    if (found && target_copy) {
      command = aimer.aim(*target_copy, timestamp, 22, false);
    }

    auto finish = std::chrono::steady_clock::now();

    // -------------- 调试输出 --------------

    tools::logger()->info(
      "[{}] detector: {:.1f}ms, tracker: {:.1f}ms, aimer: {:.1f}ms", frame_count,
      tools::delta_time(tracker_start, detector_start) * 1e3,
      tools::delta_time(aim_start, tracker_start) * 1e3,
      tools::delta_time(finish, aim_start) * 1e3);

    // UI FPS更新
    ui_manager.updateFPS();

    auto now_ui = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now_ui - last_ui_time).count();
    last_ui_time = now_ui;
    if (dt > 0) tools::logger()->info("[FPS] {:.1f}", 1.0 / dt);

    ui_web_stream.beginFrame(img.cols, img.rows);

    // -------------- 绘图数据收集 --------------

    nlohmann::json data;

    auto selected = tracker.last_observation();

    // buff原始观测数据
    if (selected.has_value()) {
      const auto & p = selected.value();
      data["buff_R_yaw"] = p.ypd_in_world[0];
      data["buff_R_pitch"] = p.ypd_in_world[1];
      data["buff_R_dis"] = p.ypd_in_world[2];
      data["buff_yaw"] = p.ypr_in_world[0] * 57.3;
      data["buff_pitch"] = p.ypr_in_world[1] * 57.3;
      data["buff_roll"] = p.ypr_in_world[2] * 57.3;
    }

    // -------------- 绘制 --------------

    if (tracker.is_tracking() && selected.has_value() && target_copy) {
      auto & p = selected.value();

      // 显示
      for (int i = 0; i < 4; i++) tools::draw_point(img, p.target().points[i]);
      tools::draw_point(img, p.target().center, {0, 0, 255}, 3);
      tools::draw_point(img, p.r_center, {0, 0, 255}, 3);

      // 大符模式下显示所有检测到的扇叶的first/last标识
      if (is_big && power_runes.has_value()) {
        for (const auto & blade : power_runes.value().fanblades) {
          if (blade.type == auto_buff::_unlight) continue;
          auto role = tracker.get_blade_role(blade.center, power_runes.value().r_center);
          if (role == auto_buff::BladeRole::FIRST) {
            tools::draw_text(img, "first", blade.center + cv::Point2f(15, -10), cv::Scalar(0, 255, 0));
          } else if (role == auto_buff::BladeRole::LAST) {
            tools::draw_text(img, "last", blade.center + cv::Point2f(15, -10), cv::Scalar(0, 165, 255));
          }
        }
      }

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

      // 观测器内部数据
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
    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    data["gimbal_yaw"] = ypr[0] * 57.3;
    data["gimbal_pitch"] = -ypr[1] * 57.3;

    if (command.control) {
      data["cmd_yaw"] = command.yaw * 57.3;
      data["cmd_pitch"] = command.pitch * 57.3;
    }

    plotter.plot(data);

    // -------------- UI渲染 (参考 auto_buff_debug_aimer) --------------

    ui_manager.initialize(img);

    // 左侧面板：模式标识
    {
      auto mode_color = is_big ? cv::Scalar(0, 165, 255) : cv::Scalar(0, 255, 0);
      ui_manager.addLeftText("buff_mode", fmt::format("Mode: {}", is_big ? "BIG" : "SMALL"),
                             mode_color);
    }

    // 左侧面板：云台状态与cmd数据
    ui_manager.addLeftText("gimbal", fmt::format("Gimbal Y: {:.1f}  P: {:.1f}",
                            -ypr[0] * 57.3, ypr[1] * 57.3));

    if (command.control) {
      ui_manager.addLeftText("cmd", fmt::format("Cmd Y: {:.1f}  P: {:.1f}",
                              -command.yaw * 57.3, -command.pitch * 57.3), cv::Scalar(0, 165, 255));
    }

    // 大符文姿态与距离
    if (selected.has_value()) {
      const auto & p = selected.value();
      ui_manager.addLeftText("rune_attitude", fmt::format("Rune Y:{:.1f} P:{:.1f} R:{:.1f}",
                            p.ypr_in_world[0] * 57.3, p.ypr_in_world[1] * 57.3, p.ypr_in_world[2] * 57.3));
      ui_manager.addLeftText("rune_distance", fmt::format("Rune Dist: {:.2f}m", p.ypd_in_world[2]));
    }

    // 大符模式下显示选择器状态
    if (is_big) {
      const auto & sel = tracker.selector();
      if (sel.is_initialized()) {
        ui_manager.addLeftText("big_buff_sel",
          fmt::format("BigBuff: {}", sel.is_tracking_first() ? "FIRST" : "LAST"),
          sel.is_tracking_first() ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255));
      }
    }

    // EKF目标状态
    bool is_tracking = tracker.is_tracking();
    if (is_tracking && target_copy) {
      auto & target_ref = tracker.target();
      Eigen::VectorXd x = target_ref.ekf_x();

      // 核心旋转信息：当前角度和角速度
      ui_manager.addLeftText("rotation", fmt::format("Angle: {:.1f}  Spd: {:.2f}",
                        x[5] * 57.3, x[6] * 57.3), cv::Scalar(255, 255, 0));

      // 扇叶观测器内参
      ui_manager.addLeftText("ekf_r", fmt::format("R_yaw: {:.2f}  R_Vyaw: {:.2f}", x[0], x[1]));
      ui_manager.addLeftText("ekf_rp", fmt::format("R_pitch: {:.2f}  R_dis: {:.2f}", x[2], x[3]));

      // 谐波模型参数 (大符 BigTarget)
      if (x.size() >= 10) {
        ui_manager.addLeftText("ekf_harmonic", fmt::format("a:{:.2f}  w:{:.2f}  fi:{:.2f}  spd0:{:.2f}",
                              x[7], x[8], x[9], target_ref.spd), cv::Scalar(0, 165, 255));
      }
    }

    // 右侧面板：检测与追踪状态
    bool has_rune = power_runes.has_value();
    ui_manager.addRightText("detect", fmt::format("Detect: {}", has_rune ? "YES" : "NO"),
                          has_rune ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255));

    ui_manager.addRightText("track", fmt::format("Track: {}", is_tracking ? "YES" : "NO"),
                          is_tracking ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255));

    ui_manager.addRightText("shoot", fmt::format("Shoot: {}", command.shoot ? "YES" : "NO"),
                          command.shoot ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0));

    // FPS叠加 (避免与UIManager内部FPS重复，放在右侧)
    auto now = std::chrono::steady_clock::now();
    double fps = 1.0 / tools::delta_time(now, last_frame_time);
    last_frame_time = now;
    ui_manager.addRightText("fps", fmt::format("FPS: {:.1f}", fps));

    ui_manager.render(img);
    ui_web_stream.capturePanels(ui_manager);
    ui_web_stream.sendFrame();

    if (ui_manager.isImshowEnabled()) {
      cv::namedWindow("result", 0);
      cv::imshow("result", img);
      int key = cv::waitKey(1);
      if (key == 'q') break;
      while (key == ' ') {
        int y = cv::waitKey(30);
        if (y == 'q') break;
        if (y == ' ') break;  // 再按空格继续
      }
    }
  }
  cv::destroyAllWindows();
  text.close();

  return 0;
}
