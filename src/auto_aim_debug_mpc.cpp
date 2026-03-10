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
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"
#include "tools/yaml.hpp"

#ifdef EZ_FILTER
#include <deque>
#include <vector>
#endif

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | ../configs/standard3.yaml | 位置参数，yaml配置文件路径 }";
bool has_target = 0;

#ifdef EZ_FILTER
//简易极端值过滤器，用于抑制目前出现的异常的pitch/yaw命令发送，待测试
struct SendCmd
{
  bool control;
  bool fire;
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
};

class SpikeFilter
{
public:
  SpikeFilter(const YAML::Node &cfg)
  {
    enabled_ = cfg["enabled"] ? cfg["enabled"].as<bool>() : true;
    m_ = cfg["m"] ? cfg["m"].as<int>() : 30;
    n_ = cfg["n"] ? cfg["n"].as<int>() : 3;
    d_degree_ = cfg["d_degree"] ? cfg["d_degree"].as<double>() : 5.0; // degrees
    x_ = cfg["x"] ? cfg["x"].as<int>() : 5;
    // ensure sane values
    if (m_ <= 0) m_ = 30;
    if (n_ <= 0) n_ = 3;
    if (x_ <= 0) x_ = 5;
  }

  // process a new candidate; returns a list of commands that should be sent now (may be empty)
  std::vector<SendCmd> process(const SendCmd &cmd)
  {
    std::vector<SendCmd> out;
    if (!enabled_) {
      out.push_back(cmd);
      last_normal_ = cmd;
      push_history(cmd);
      return out;
    }

    // compute average of last n history (or use last_normal_ if not enough samples)
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
      // mark as abnormal (do not send)
      abnormal_buffer_.push_back(cmd);
      // reset counters
      consecutive_like_abnormal_ = 0;
      consecutive_like_normal_ = 0;
      return out;
    }

    // This cmd is similar to recent history (candidate normal)
    // If there are pending abnormal frames, check reconciliation conditions
    if (!abnormal_buffer_.empty()) {
      // Compare this cmd to last abnormal and to last normal
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
        // treat previous abnormal buffer as actually normal -> send them now
        for (const auto &c : abnormal_buffer_) out.push_back(c);
        abnormal_buffer_.clear();
        consecutive_like_abnormal_ = 0;
      } else if (consecutive_like_normal_ >= x_) {
        // previous abnormal frames are true anomalies -> drop them
        abnormal_buffer_.clear();
        consecutive_like_normal_ = 0;
      }
    }

    // send current cmd as normal
    out.push_back(cmd);
    last_normal_ = cmd;
    push_history(cmd);
    return out;
  }

private:
  void push_history(const SendCmd &c)
  {
    history_.push_back(c);
    if ((int)history_.size() > m_) history_.pop_front();
  }

  bool enabled_ = true;
  int m_ = 30; // history capacity
  int n_ = 3;  // compare with last n frames
  double d_degree_ = 5.0; // threshold in degrees
  int x_ = 5; // consecutive frames threshold

  std::deque<SendCmd> history_;
  std::vector<SendCmd> abnormal_buffer_;
  SendCmd last_normal_{};
  int consecutive_like_abnormal_ = 0;
  int consecutive_like_normal_ = 0;
};
#endif

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

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  #ifdef EZ_FILTER
  // Load filter config from yaml (optional)
  auto yaml_cfg = tools::load(config_path);
  YAML::Node filter_node = yaml_cfg["filter"] ? yaml_cfg["filter"] : YAML::Node();
  SpikeFilter spike_filter(filter_node);
  #endif

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  std::atomic<bool> quit = false;
  
  // FPS计算相关变量
  auto last_fps_time = std::chrono::steady_clock::now();
  int frame_count = 0;
  float fps = 0.0f;
  
  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;

    while (!quit) {
      auto target = target_queue.front();
      auto gs = gimbal.state();
      // auto plan = planner.plan(target, gs.bullet_speed);
      auto plan = planner.go(tracker.armor_);

      #ifdef EZ_FILTER
      //准备命令并通过过滤器处理
      SendCmd cmd;
      cmd.control = has_target;
      cmd.fire = plan.fire;
      cmd.yaw = plan.yaw;
      cmd.yaw_vel = plan.yaw_vel;
      cmd.yaw_acc = plan.yaw_acc;
      cmd.pitch = plan.pitch;
      cmd.pitch_vel = plan.pitch_vel;
      cmd.pitch_acc = plan.pitch_acc;

      auto to_send = spike_filter.process(cmd);
      for (const auto &c : to_send) {
        gimbal.send(c.control, c.fire, c.yaw, c.yaw_vel, c.yaw_acc, c.pitch, c.pitch_vel, c.pitch_acc);
      }
      #else
      std::cout<<plan.control<<std::endl;
      gimbal.send(
        has_target, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
        plan.pitch_acc);
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
    
    camera.read(img, t);

    auto armors = yolo.detect(img);
    auto q = gimbal.q(t);

    solver.set_R_gimbal2world(q);
    auto targets = tracker.track(armors, t);
    
    if(tracker.aim_strategy_ == "follow") {
      planner.aim_center_ = false;
    }
    else {
      planner.aim_center_ = true;
    }
    tools::draw_text(img, fmt::format("Aim Strategy: {}", tracker.aim_strategy_), {10, 690}, {0, 255, 0});
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
    // 绘制锁定中心
    if (planner.aim_center_) {
      auto center_image_points = solver.reproject_point(planner.center_points);
      tools::draw_points(img, center_image_points, {0, 0, 255}, 10);
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

      Eigen::Vector4d aim_xyza = planner.debug_xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if(planner.aim_center_ == false){
        tools::draw_points(img, image_points, {0, 0, 255});
      }
    }
    
    // 获取云台状态和规划信息用于UI显示
    auto gs = gimbal.state();
    std::optional<auto_aim::Target> target_opt = target_queue.front();
    //bool has_target = target_opt.has_value();
    has_target = target_opt.has_value();
    auto plan = planner.plan(target_opt, gs.bullet_speed);
    
    // 计算FPS
    frame_count++;
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_fps_time).count();
    
    if (elapsed_time >= 1000) { // 每秒更新一次FPS
      fps = static_cast<float>(frame_count) * 1000.0f / static_cast<float>(elapsed_time);
      frame_count = 0;
      last_fps_time = current_time;
    }
    
    // 在图像上绘制UI信息
    int y_offset = 30;
    int line_height = 25;
    cv::Scalar text_color(0, 255, 0); // 绿色文本
    cv::Scalar highlight_color(0, 165, 255); // 橙色高亮文本
    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.6;
    int thickness = 2;
    
    // FPS信息
    std::string fps_text = fmt::format("FPS: {:.1f}", fps);
    cv::putText(img, fps_text, cv::Point(10, y_offset), font_face, font_scale, text_color, thickness);
    y_offset += line_height;
    
    // 运行模式
    std::string mode_text = "Mode: AutoAim";
    cv::putText(img, mode_text, cv::Point(10, y_offset), font_face, font_scale, text_color, thickness);
    y_offset += line_height;
    
    // 目标检测状态
    std::string detect_text = fmt::format("Detect: {}", has_target ? "YES" : "NO");
    cv::putText(img, detect_text, cv::Point(10, y_offset), font_face, font_scale, 
                has_target ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), thickness);
    y_offset += line_height;
    
    // 开火状态
    std::string fire_text = fmt::format("Fire: {}", plan.fire ? "YES" : "NO");
    cv::putText(img, fire_text, cv::Point(10, y_offset), font_face, font_scale, 
                plan.fire ? cv::Scalar(0, 0, 255) : text_color, thickness);
    y_offset += line_height;
    
    // 云台状态 - 接收到的数据
    std::string gimbal_status = fmt::format("Gimbal Yaw: {:.2f}  Pitch: {:.2f}", -gs.yaw * 180.0 / M_PI, -gs.pitch * 180.0 / M_PI);
    cv::putText(img, gimbal_status, cv::Point(10, y_offset), font_face, font_scale, text_color, thickness);
    y_offset += line_height;

    std::string gimbal_vel = fmt::format("Gimbal Vel Y: {:.2f}  P: {:.2f}", -gs.yaw_vel * 180.0 / M_PI, -gs.pitch_vel * 180.0 / M_PI);
    cv::putText(img, gimbal_vel, cv::Point(10, y_offset), font_face, font_scale, text_color, thickness);
    y_offset += line_height;
    
    // 规划状态 - 发送的数据
    std::string plan_status = fmt::format("Plan Yaw: {:.2f}  Pitch: {:.2f}", -plan.yaw * 180.0 / M_PI, -plan.pitch * 180.0 / M_PI);
    cv::putText(img, plan_status, cv::Point(10, y_offset), font_face, font_scale, highlight_color, thickness);
    y_offset += line_height;
    
    std::string plan_vel = fmt::format("Plan Vel Y: {:.2f}  P: {:.2f}", -plan.yaw_vel * 180.0 / M_PI, -plan.pitch_vel * 180.0 / M_PI);
    cv::putText(img, plan_vel, cv::Point(10, y_offset), font_face, font_scale, highlight_color, thickness);
    y_offset += line_height;
    
    std::string plan_acc = fmt::format("Plan Acc Y: {:.2f}  P: {:.2f}", plan.yaw_acc, plan.pitch_acc);
    cv::putText(img, plan_acc, cv::Point(10, y_offset), font_face, font_scale, highlight_color, thickness);
    y_offset += line_height;
    
    // 目标信息（如果存在）
    if (has_target) {
      auto& target = target_opt.value();
      std::string target_info = fmt::format("Target Z: {:.2f}  Vz: {:.2f}", target.ekf_x()[4], target.ekf_x()[5]);
      cv::putText(img, target_info, cv::Point(10, y_offset), font_face, font_scale, cv::Scalar(255, 255, 0), thickness);
      y_offset += line_height;
      
      if (target.ekf_x().size() > 7) {
        std::string target_w = fmt::format("Target W: {:.2f}", target.ekf_x()[7]);
        cv::putText(img, target_w, cv::Point(10, y_offset), font_face, font_scale, cv::Scalar(255, 255, 0), thickness);
        y_offset += line_height;
      }
    }
    
    // 在图像右侧添加分隔线和状态指示
    int img_width = img.cols;
    cv::line(img, cv::Point(img_width - 200, 20), cv::Point(img_width - 200, y_offset + 20), cv::Scalar(0, 255, 255), 2);
    
    // 右侧状态指示
    int right_x = img_width - 180;
    y_offset = 30;
    
    std::string status_title = "STATUS";
    cv::putText(img, status_title, cv::Point(right_x, y_offset), font_face, font_scale + 0.1, cv::Scalar(255, 255, 0), thickness);
    y_offset += line_height + 10;
    
    // 系统时间
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::string time_str = std::ctime(&now_c);
    time_str = time_str.substr(11, 8); // 只取HH:MM:SS部分
    std::string time_text = fmt::format("Time: {}", time_str);
    cv::putText(img, time_text, cv::Point(right_x, y_offset), font_face, font_scale, text_color, thickness);
    y_offset += line_height;
    
    // 子弹速度
    std::string bullet_speed_text = fmt::format("Bullet Speed: {:.1f}", gs.bullet_speed);
    cv::putText(img, bullet_speed_text, cv::Point(right_x, y_offset), font_face, font_scale, text_color, thickness);
    y_offset += line_height;
    
    // 目标数量
    std::string target_count_text = fmt::format("Targets: {}", targets.size());
    cv::putText(img, target_count_text, cv::Point(right_x, y_offset), font_face, font_scale, 
                targets.empty() ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), thickness);
    y_offset += line_height;
    
    // 检测到的装甲板数量
    std::string armor_count_text = fmt::format("Armors: {}", armors.size());
    cv::putText(img, armor_count_text, cv::Point(right_x, y_offset), font_face, font_scale, 
                armors.empty() ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), thickness);
    y_offset += line_height;

    //cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::namedWindow("reprojection", 0);
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}
