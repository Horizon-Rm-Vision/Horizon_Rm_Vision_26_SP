#include <fmt/core.h>

#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_director.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_tracker.hpp"
#include "tasks/auto_buff/buff_type.hpp"
#include "tools/exiter.hpp"
#include "tools/ui_manager.hpp"
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

  detector.setBig2026Mode(is_big);

  auto_buff::BuffTracker tracker(config_path, buff_type);
  auto_buff::Aimer aimer(config_path);
  auto_buff::Buff2026Director director;

  cv::Mat img, drawing;
  auto t0 = std::chrono::steady_clock::now();

  io::Command last_command;
  double last_t = -1;
  auto last_frame_time = std::chrono::steady_clock::now();

  video.set(cv::CAP_PROP_POS_FRAMES, start_index);
  for (int i = 0; i < start_index; i++) {
    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
  }

  for (int frame_count = start_index; !exiter.exit(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;

    video.read(img);
    if (img.empty()) break;

    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
    auto timestamp = t0 + std::chrono::microseconds(int(t * 1e6));

    /// 自瞄核心逻辑 (BuffTracker + Aimer)

    solver.set_R_gimbal2world({w, x, y, z});

    auto detector_start = std::chrono::steady_clock::now();
    auto power_runes = detector.detect_24(img);

    auto tracker_start = std::chrono::steady_clock::now();
    auto found = tracker.update(power_runes, timestamp, solver);

    auto target_copy = tracker.clone_target();

    auto director_start = std::chrono::steady_clock::now();
    io::Command command = {false, false, 0, 0};
    int blade_id = 0;
    if (found && target_copy) {
      if (is_big) {
        cv::Point2f image_center(img.cols / 2.0f, img.rows / 2.0f);
        director.update(power_runes, timestamp, tracker.target(), image_center);
        blade_id = director.getAimBladeId();
        if (blade_id < 0) blade_id = 0;
      }
      command = aimer.aim(*target_copy, timestamp, 22, false, blade_id);
    }

    auto finish = std::chrono::steady_clock::now();

    // -------------- 调试输出 --------------

    tools::logger()->info(
      "[{}] detector: {:.1f}ms, tracker: {:.1f}ms, director+aimer: {:.1f}ms", frame_count,
      tools::delta_time(tracker_start, detector_start) * 1e3,
      tools::delta_time(director_start, tracker_start) * 1e3,
      tools::delta_time(finish, director_start) * 1e3);

    nlohmann::json data;

    // data["bullet_speed"] = cboard.bullet_speed;

    // buff原始观测数据
    auto selected = tracker.last_observation();
    if (selected.has_value()) {
      const auto & p = selected.value();
      data["buff_R_yaw"] = p.ypd_in_world[0];
      data["buff_R_pitch"] = p.ypd_in_world[1];
      data["buff_R_dis"] = p.ypd_in_world[2];
      data["buff_yaw"] = p.ypr_in_world[0] * 57.3;
      data["buff_pitch"] = p.ypr_in_world[1] * 57.3;
      data["buff_roll"] = p.ypr_in_world[2] * 57.3;
    }

    if (tracker.is_tracking() && selected.has_value() && target_copy) {
      auto & p = selected.value();

      // 显示
      if (is_big) {
        for (auto * blade : p.get_targets()) {
          for (int i = 0; i < 4; i++) tools::draw_point(img, blade->points[i]);
          tools::draw_point(img, blade->center, {0, 0, 255}, 3);
        }
      } else {
        for (int i = 0; i < 4; i++) tools::draw_point(img, p.target().points[i]);
        tools::draw_point(img, p.target().center, {0, 0, 255}, 3);
      }
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

      if (is_big) {
        data["director_state"] = static_cast<int>(director.getState());
        data["director_completed"] = director.getCompletedGroups();
        data["director_blade_id"] = blade_id;
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

    auto now = std::chrono::steady_clock::now();
    double fps = 1.0 / tools::delta_time(now, last_frame_time);
    last_frame_time = now;
    tools::draw_text(
      img, fmt::format("FPS: {:.1f}", fps), {10, 30}, {255, 0, 0});

    cv::imshow("result", img);

    int key = cv::waitKey(1);
    if (key == 'q') break;
    while (key == ' ') {
      int y = cv::waitKey(30);
      if (y == 'q') break;
    }
  }
  cv::destroyAllWindows();
  text.close();  // 关闭文件

  return 0;
}
