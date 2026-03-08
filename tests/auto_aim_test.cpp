#include <fmt/core.h>

#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

#ifdef EZ_FILTER
#include "tools/yaml.hpp"
#endif

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

  #ifdef EZ_FILTER
  // load filter configuration (same as in auto_aim_debug_mpc)
  auto yaml_cfg = tools::load(config_path);
  YAML::Node filter_node = yaml_cfg["filter"] ? yaml_cfg["filter"] : YAML::Node();

  // replicate SendCmd and SpikeFilter here
  struct SendCmd {
    bool control;
    bool fire;
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
  };

  class SpikeFilter {
  public:
    SpikeFilter(const YAML::Node &cfg) {
      enabled_ = cfg["enabled"] ? cfg["enabled"].as<bool>() : true;
      m_ = cfg["m"] ? cfg["m"].as<int>() : 30;
      n_ = cfg["n"] ? cfg["n"].as<int>() : 3;
      d_degree_ = cfg["d_degree"] ? cfg["d_degree"].as<double>() : 5.0;
      x_ = cfg["x"] ? cfg["x"].as<int>() : 5;
      if (m_ <= 0) m_ = 30;
      if (n_ <= 0) n_ = 3;
      if (x_ <= 0) x_ = 5;
    }
    std::vector<SendCmd> process(const SendCmd &cmd) {
      std::vector<SendCmd> out;
      if (!enabled_) {
        out.push_back(cmd);
        last_normal_ = cmd;
        push_history(cmd);
        return out;
      }
      double avg_yaw_deg = 0.0, avg_pitch_deg = 0.0;
      int count = 0;
      for (int i = (int)history_.size() - 1; i >= 0 && count < n_; --i, ++count) {
        avg_yaw_deg += history_[i].yaw * 180.0 / M_PI;
        avg_pitch_deg += history_[i].pitch * 180.0 / M_PI;
      }
      if (count == 0) {
        avg_yaw_deg = last_normal_.yaw * 180.0 / M_PI;
        avg_pitch_deg = last_normal_.pitch * 180.0 / M_PI;
        count = 1;
      } else {
        avg_yaw_deg /= count;
        avg_pitch_deg /= count;
      }
      double cmd_yaw_deg = cmd.yaw * 180.0 / M_PI;
      double cmd_pitch_deg = cmd.pitch * 180.0 / M_PI;
      double diff = std::max(std::abs(cmd_yaw_deg - avg_yaw_deg), std::abs(cmd_pitch_deg - avg_pitch_deg));
      if (diff > d_degree_) {
        abnormal_buffer_.push_back(cmd);
        consecutive_like_abnormal_ = 0;
        consecutive_like_normal_ = 0;
        return out;
      }
      if (!abnormal_buffer_.empty()) {
        const SendCmd &last_ab = abnormal_buffer_.back();
        double last_ab_yaw_deg = last_ab.yaw * 180.0 / M_PI;
        double last_ab_pitch_deg = last_ab.pitch * 180.0 / M_PI;
        double diff_to_ab = std::max(std::abs(cmd_yaw_deg - last_ab_yaw_deg), std::abs(cmd_pitch_deg - last_ab_pitch_deg));
        double last_norm_yaw_deg = last_normal_.yaw * 180.0 / M_PI;
        double last_norm_pitch_deg = last_normal_.pitch * 180.0 / M_PI;
        double diff_to_norm = std::max(std::abs(cmd_yaw_deg - last_norm_yaw_deg), std::abs(cmd_pitch_deg - last_norm_pitch_deg));
        if (diff_to_ab < d_degree_) {
          consecutive_like_abnormal_++;
        } else {
          consecutive_like_abnormal_ = 0;
        }
        if (diff_to_norm < d_degree_) {
          consecutive_like_normal_++;
        } else {
          consecutive_like_normal_ = 0;
        }
        if (consecutive_like_abnormal_ >= x_) {
          for (const auto &c : abnormal_buffer_) out.push_back(c);
          abnormal_buffer_.clear();
          consecutive_like_abnormal_ = 0;
        } else if (consecutive_like_normal_ >= x_) {
          abnormal_buffer_.clear();
          consecutive_like_normal_ = 0;
        }
      }
      out.push_back(cmd);
      last_normal_ = cmd;
      push_history(cmd);
      return out;
    }
  private:
    void push_history(const SendCmd &c) {
      history_.push_back(c);
      if ((int)history_.size() > m_) history_.pop_front();
    }
    bool enabled_ = true;
    int m_ = 30;
    int n_ = 3;
    double d_degree_ = 5.0;
    int x_ = 5;
    std::deque<SendCmd> history_;
    std::vector<SendCmd> abnormal_buffer_;
    SendCmd last_normal_{};
    int consecutive_like_abnormal_ = 0;
    int consecutive_like_normal_ = 0;
  };
  SpikeFilter spike_filter(filter_node);
  #endif

  auto video_path = fmt::format("{}.avi", input_path);
  auto text_path = fmt::format("{}.txt", input_path);
  cv::VideoCapture video(video_path);
  std::ifstream text(text_path);

  auto_aim::YOLO yolo(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);

  cv::Mat img, drawing;
  auto t0 = std::chrono::steady_clock::now();

  auto_aim::Target last_target;
  io::Command last_command;
  double last_t = -1;

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

    /// 自瞄核心逻辑

    solver.set_R_gimbal2world({w, x, y, z});

    auto yolo_start = std::chrono::steady_clock::now();
    auto armors = yolo.detect(img, frame_count);

    auto tracker_start = std::chrono::steady_clock::now();
    auto targets = tracker.track(armors, timestamp);

    auto aimer_start = std::chrono::steady_clock::now();
    auto command = aimer.aim(targets, timestamp, 27, false);

    #ifdef EZ_FILTER
    // apply spike filter to command (mimic sending step)
    SendCmd scmd;
    scmd.control = command.control;
    scmd.fire = command.shoot; // treat shoot as fire
    scmd.yaw = command.yaw;
    scmd.yaw_vel = 0;
    scmd.yaw_acc = 0;
    scmd.pitch = command.pitch;
    scmd.pitch_vel = 0;
    scmd.pitch_acc = 0;
    auto filtered = spike_filter.process(scmd);
    if (!filtered.empty()) {
      // use last filtered command for logic
      const auto &f = filtered.back();
      command.control = f.control;
      command.yaw = f.yaw;
      command.pitch = f.pitch;
      command.shoot = f.fire;
    } else {
      // no command sent this frame
      command.control = false;
      command.shoot = false;
    }
    #endif

    if (
      !targets.empty() && aimer.debug_aim_point.valid &&
      std::abs(command.yaw - last_command.yaw) * 57.3 < 2)
      command.shoot = true;

    if (command.control) last_command = command;
    /// 调试输出

    auto finish = std::chrono::steady_clock::now();
    tools::logger()->info(
      "[{}] yolo: {:.1f}ms, tracker: {:.1f}ms, aimer: {:.1f}ms", frame_count,
      tools::delta_time(tracker_start, yolo_start) * 1e3,
      tools::delta_time(aimer_start, tracker_start) * 1e3,
      tools::delta_time(finish, aimer_start) * 1e3);

    tools::draw_text(
      img,
      fmt::format(
        "command is {},{:.2f},{:.2f},shoot:{}", command.control, command.yaw * 57.3,
        command.pitch * 57.3, command.shoot),
      {10, 60}, {154, 50, 205});

    Eigen::Quaternion gimbal_q = {w, x, y, z};
    tools::draw_text(
      img,
      fmt::format(
        "gimbal yaw{:.2f}", (tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0) * 57.3)[0]),
      {10, 90}, {255, 255, 255});

    nlohmann::json data;

    // 装甲板原始观测数据
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

    Eigen::Quaternion q{w, x, y, z};
    auto yaw = tools::eulers(q, 2, 1, 0)[0];
    data["gimbal_yaw"] = yaw * 57.3;
    data["cmd_yaw"] = command.yaw * 57.3;
    data["shoot"] = command.shoot;

    if (!targets.empty()) {
      auto target = targets.front();

      if (last_t == -1) {
        last_target = target;
        last_t = t;
        continue;
      }

      std::vector<Eigen::Vector4d> armor_xyza_list;

      // 当前帧target更新后
      armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      // aimer瞄准位置
      auto aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid) tools::draw_points(img, image_points, {0, 0, 255});

      // 观测器内部数据
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

          

          //plotter.drawData({ command.pitch * 180/M_PI}, {"target_pitch"});
    }
plotter.drawData({ command.yaw * 180/M_PI,yaw * 180/M_PI}, {"target_yaw","gimbal_yaw"});
    //plotter.plot(data);
  

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(30);
    if (key == 'q') break;
  }

  return 0;
}