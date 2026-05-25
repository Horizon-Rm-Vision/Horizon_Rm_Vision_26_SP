#include <fmt/core.h>

#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/ui_manager.hpp"
#include "tools/ui_web_stream.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"


const std::string keys =
  "{help h usage ? |                   | 输出命令行参数说明 }"
  "{config-path c  | ../configs/demo.yaml | yaml配置文件的路径}"
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

  tools::UIManager ui_manager(config_path);
  tools::UIWebStream ui_web_stream(config_path);
  plotter.configureWebStreamFromConfig(config_path);
  ui_manager.setProgramMode("AutoAim MPC Replay");

  auto video_path = fmt::format("{}.avi", input_path);
  auto text_path = fmt::format("{}.txt", input_path);
  cv::VideoCapture video(video_path);
  std::ifstream text(text_path);

  auto_aim::YOLO yolo(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  cv::Mat img;
  auto t0 = std::chrono::steady_clock::now();

  auto last_frame_time = std::chrono::steady_clock::now();

  video.set(cv::CAP_PROP_POS_FRAMES, start_index);
  for (int i = 0; i < start_index; i++) {
    double t, w, x, y, z;
    int frame_idx;
    text >> t >> w >> x >> y >> z >> frame_idx;
  }

  auto last_ui_time = std::chrono::steady_clock::now();

  for (int frame_count = start_index; !exiter.exit(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;

    video.read(img);
    if (img.empty()) break;

    double t, w, x, y, z;
    int frame_idx;
    text >> t >> w >> x >> y >> z >> frame_idx;
    auto timestamp = t0 + std::chrono::microseconds(int(t * 1e6));

    ui_web_stream.sendImage(img);

    /// 自瞄核心逻辑（MPC）

    solver.set_R_gimbal2world({w, x, y, z});

    auto yolo_start = std::chrono::steady_clock::now();
    auto armors = yolo.detect(img, frame_count);

    auto tracker_start = std::chrono::steady_clock::now();
    auto targets = tracker.track(armors, timestamp);

    auto plan_start = std::chrono::steady_clock::now();
    auto_aim::Plan plan = {false};
    if (!targets.empty()) {
      // 直接调用 plan(Target, bullet_speed)，避免 plan(optional<Target>)
      // 内部使用 steady_clock::now() 导致回放时间错乱
      plan = planner.plan(targets.front(), 27);
    }

    auto finish = std::chrono::steady_clock::now();
    tools::logger()->info(
      "[{}] yolo: {:.1f}ms, tracker: {:.1f}ms, planner: {:.1f}ms", frame_count,
      tools::delta_time(tracker_start, yolo_start) * 1e3,
      tools::delta_time(plan_start, tracker_start) * 1e3,
      tools::delta_time(finish, plan_start) * 1e3);

    // UI FPS更新
    ui_manager.updateFPS();

    auto now_ui = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now_ui - last_ui_time).count();
    last_ui_time = now_ui;
    if (dt > 0) tools::logger()->info("[FPS] {:.1f}", 1.0 / dt);

    ui_web_stream.beginFrame(img.cols, img.rows);

    /// 调试绘制

    // 装甲板原始观测数据
    for (const auto & armor : armors) {
      tools::draw_points(img, armor.points, {0, 255, 0});
      auto info = fmt::format("{:.2f} {} {} {}", armor.confidence,
        auto_aim::COLORS[armor.color], auto_aim::ARMOR_NAMES[armor.name],
        auto_aim::ARMOR_TYPES[armor.type]);
      tools::draw_text(img, info, armor.center, {0, 255, 0});
    }

    Eigen::Quaternion gimbal_q = {w, x, y, z};
    auto yaw = tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0)[0];
    auto pitch = tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0)[1];

    // 绘图器数据收集
    nlohmann::json data;

    data["armor_num"] = armors.size();
    if (!armors.empty()) {
      const auto & armor = armors.front();
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
      data["armor_center_x"] = armor.center_norm.x;
      data["armor_center_y"] = armor.center_norm.y;
    }

    data["gimbal_yaw"] = yaw * 57.3;
    data["plan_yaw"] = plan.yaw * 57.3;
    data["plan_pitch"] = plan.pitch * 57.3;
    data["fire"] = plan.fire;

    if (!targets.empty()) {
      auto target = targets.front();

      // 当前帧target更新后
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      // planner预测位置（红框）
      Eigen::Vector4d aim_xyza = planner.debug_xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      tools::draw_points(img, image_points, {0, 0, 255});

      // EKF内部数据
      Eigen::VectorXd x = target.ekf_x();
      data["x"] = x[0];
      data["vx"] = x[1];
      data["y"] = x[2];
      data["vy"] = x[3];
      data["z"] = x[4];
      data["vz"] = x[5];
      data["a"] = x[6] * 57.3;
      data["w"] = x[7];
      data["r"] = x[8];
      data["l"] = x[9];
      data["h"] = x[10];
      data["last_id"] = target.last_id;

      // 卡方检验数据
      data["residual_yaw"] = target.ekf().data.at("residual_yaw");
      data["residual_pitch"] = target.ekf().data.at("residual_pitch");
      data["residual_distance"] = target.ekf().data.at("residual_distance");
      data["residual_angle"] = target.ekf().data.at("residual_angle");
      data["nis"] = target.ekf().data.at("nis");
      data["nees"] = target.ekf().data.at("nees");
      data["nis_fail"] = target.ekf().data.at("nis_fail");
      data["nees_fail"] = target.ekf().data.at("nees_fail");
      data["recent_nis_failures"] = target.ekf().data.at("recent_nis_failures");
    }

    plotter.subplot("Yaw", {yaw * 180/M_PI, plan.yaw * 180/M_PI},
                    {"gimbal_yaw", "plan_yaw"});
    plotter.subplot("Pitch", {pitch * 180/M_PI, plan.pitch * 180/M_PI},
                    {"gimbal_pitch", "plan_pitch"});
    plotter.draw();

    // -------------- UI渲染 --------------

    ui_manager.initialize(img);

    // 左侧面板：检测与开火状态
    bool has_target = !targets.empty();
    ui_manager.addLeftText("detect", fmt::format("Detect: {}", has_target ? "YES" : "NO"),
                          has_target ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255));
    ui_manager.addLeftText("fire", fmt::format("Fire: {}", plan.fire ? "YES" : "NO"),
                          plan.fire ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0));

    // 左侧面板：云台姿态与规划指令
    Eigen::Vector3d ypr = tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0);
    ui_manager.addLeftText("gimbal", fmt::format("Gimbal Y: {:.1f}  P: {:.1f}",
                            -ypr[0] * 57.3, ypr[1] * 57.3));

    if (plan.control) {
      ui_manager.addLeftText("plan_cmd", fmt::format("Plan Y: {:.1f}  P: {:.1f}",
                                -plan.yaw * 57.3, -plan.pitch * 57.3), cv::Scalar(0, 165, 255));
      ui_manager.addLeftText("plan_vel", fmt::format("Plan Vel Y: {:.1f}  P: {:.1f}",
                                -plan.yaw_vel * 57.3, -plan.pitch_vel * 57.3), cv::Scalar(0, 165, 255));
    }

    // EKF目标状态
    if (!targets.empty()) {
      auto target = targets.front();
      Eigen::VectorXd x = target.ekf_x();
      ui_manager.addLeftText("ekf_pos", fmt::format("x:{:.2f} y:{:.2f} z:{:.2f}", x[0], x[2], x[4]));
      ui_manager.addLeftText("ekf_vel", fmt::format("vx:{:.2f} vy:{:.2f} vz:{:.2f}", x[1], x[3], x[5]));
      ui_manager.addLeftText("ekf_rot", fmt::format("a:{:.1f}deg w:{:.2f} r:{:.2f}", x[6] * 57.3, x[7], x[8]));
    }

    // 右侧面板：目标统计与FPS
    ui_manager.addRightText("target_count", fmt::format("Targets: {}", targets.size()),
                           targets.empty() ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0));
    ui_manager.addRightText("armor_count", fmt::format("Armors: {}", armors.size()),
                           armors.empty() ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0));

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
        if (y == ' ') break;
      }
    }
  }

  cv::destroyAllWindows();
  text.close();

  return 0;
}
