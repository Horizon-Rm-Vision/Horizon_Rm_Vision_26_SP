#include "tracker.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <tuple>

#include "io/gimbal/gimbal.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Tracker::Tracker(const std::string & config_path, Solver & solver)
: solver_{solver},
  enemy_color_{Color::blue},
  enemy_color_auto_{false},
  detect_count_(0),
  #ifdef NOVA_OUTPOST_V2
  detect_fail_count_(0),
  #endif
  temp_lost_count_(0),
  state_{"lost"},
  pre_state_{"lost"},
  last_timestamp_(std::chrono::steady_clock::now()),
  omni_target_priority_{ArmorPriority::fifth},
  outpost_correction_enable_{false},
  outpost_correction_active_{false},
  outpost_correction_min_detect_count_{3},
  outpost_correction_cancel_count_{5},
  outpost_seen_streak_{0},
  non_outpost_seen_streak_{0},
  outpost_top_filter_enable_{false},
  outpost_top_pitch_{27.5},
  outpost_front_pitch_{-15.0}
{
  auto yaml = YAML::LoadFile(config_path);
  const auto enemy_color_cfg = yaml["enemy_color"].as<std::string>();
  if (enemy_color_cfg == "auto") {
    enemy_color_auto_ = true;
    refresh_enemy_color_from_serial();
  } else {
    enemy_color_ = (enemy_color_cfg == "red") ? Color::red : Color::blue;
  }
  min_detect_count_ = yaml["min_detect_count"].as<int>();
  max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();
  outpost_max_temp_lost_count_ = yaml["outpost_max_temp_lost_count"].as<int>();
#ifdef NOVA_OUTPOST_V2
  outpost_min_detect_count_ =
    yaml["outpost_min_detect_count"] ? yaml["outpost_min_detect_count"].as<int>() : 3;
  outpost_detect_fail_tolerance_ =
    yaml["outpost_detect_fail_tolerance"] ? yaml["outpost_detect_fail_tolerance"].as<int>() : 8;

  Target::OutpostV2Params outpost_v2_params;
  outpost_v2_params.h_max_reasonable =
    yaml["outpost_h_max_reasonable"] ? yaml["outpost_h_max_reasonable"].as<double>() : 0.25;
  outpost_v2_params.match_gate =
    yaml["outpost_match_gate"] ? yaml["outpost_match_gate"].as<double>() : 10.0;
  outpost_v2_params.q_h =
    yaml["outpost_q_h"] ? yaml["outpost_q_h"].as<double>() : 0.0006;
  outpost_v2_params.h_converged_variance = yaml["outpost_h_converged_variance"] ?
      yaml["outpost_h_converged_variance"].as<double>() :
      0.05;
  outpost_v2_params.converged_pos_p_max = yaml["outpost_converged_pos_p_max"] ?
      yaml["outpost_converged_pos_p_max"].as<double>() :
      0.5;
  outpost_v2_params.converged_vel_p_max = yaml["outpost_converged_vel_p_max"] ?
      yaml["outpost_converged_vel_p_max"].as<double>() :
      10.0;
  Target::set_outpost_v2_params(outpost_v2_params);
#endif

  outpost_correction_enable_ =
    yaml["outpost_correction_enable"] ? yaml["outpost_correction_enable"].as<bool>() : false;
  outpost_correction_min_detect_count_ = yaml["outpost_correction_min_detect_count"] ?
    yaml["outpost_correction_min_detect_count"].as<int>() :
    outpost_correction_min_detect_count_;
  outpost_correction_cancel_count_ = yaml["outpost_correction_cancel_count"] ?
    yaml["outpost_correction_cancel_count"].as<int>() :
    outpost_correction_cancel_count_;

  outpost_top_filter_enable_ =
    yaml["outpost_top_filter_enable"] ? yaml["outpost_top_filter_enable"].as<bool>() : false;
  outpost_top_pitch_ =
    yaml["outpost_top_pitch"] ? yaml["outpost_top_pitch"].as<double>() : 27.5;
  outpost_front_pitch_ =
    yaml["outpost_front_pitch"] ? yaml["outpost_front_pitch"].as<double>() : -15.0;

  normal_temp_lost_count_ = max_temp_lost_count_;
}

std::string Tracker::state() const { return state_; }

void Tracker::refresh_enemy_color_from_serial()
{
  if (!enemy_color_auto_) return;

  const auto self_color = io::latest_self_color();
  if (self_color == 0) {
    enemy_color_ = Color::blue;
  } else if (self_color == 1) {
    enemy_color_ = Color::red;
  }
}

void Tracker::apply_outpost_correction(std::list<Armor> & armors)
{
  if (!outpost_correction_enable_ || armors.empty()) return;

  const bool has_outpost =
    std::any_of(armors.begin(), armors.end(), [](const Armor & a) {
      return a.name == ArmorName::outpost;
    });

  if (has_outpost) {
    outpost_seen_streak_++;
    non_outpost_seen_streak_ = 0;
  } else {
    non_outpost_seen_streak_++;
    outpost_seen_streak_ = 0;
  }

  if (!outpost_correction_active_ && outpost_seen_streak_ >= outpost_correction_min_detect_count_) {
    outpost_correction_active_ = true;
  }

  if (outpost_correction_active_ &&
      non_outpost_seen_streak_ >= outpost_correction_cancel_count_) {
    outpost_correction_active_ = false;
  }

  if (outpost_correction_active_ && !has_outpost) {
    for (auto & armor : armors) {
      armor.name = ArmorName::outpost;
    }
  }
}

std::list<Target> Tracker::track(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  refresh_enemy_color_from_serial();

  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }
  // 过滤掉非我方装甲板
  armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });

  apply_outpost_correction(armors);

  // 过滤前哨站顶部装甲板
  if (outpost_top_filter_enable_) {
    armors.remove_if([this](const auto_aim::Armor & a) {
      return a.name == ArmorName::outpost &&
             solver_.oupost_reprojection_error(a, outpost_top_pitch_ * CV_PI / 180.0) <
               solver_.oupost_reprojection_error(a, outpost_front_pitch_ * CV_PI / 180.0);
    });
  }

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort(
    [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t);
  }

  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {};
  }

  // 收敛效果检测：（仅在tracking状态执行，避免temp_lost恢复时误触发重置）
  // 参照 Auto_Aim tracker 的设计：NIS失败检查只应在滤波器稳定跟踪时评估
  if (state_ == "tracking") {
    if (
      std::accumulate(
        target_.ekf().recent_nis_failures.begin(), target_.ekf().recent_nis_failures.end(), 0) >=
      (0.4 * target_.ekf().window_size)) {
      tools::logger()->debug("[Target] Bad Converge Found!");
      state_ = "lost";
      return {};
    }
  }

  if (state_ == "lost") return {};

  std::list<Target> targets = {target_};
  return targets;
}

std::tuple<omniperception::DetectionResult, std::list<Target>> Tracker::track(
  const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
  std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  refresh_enemy_color_from_serial();

  omniperception::DetectionResult switch_target{std::list<Armor>(), t, 0, 0};
  omniperception::DetectionResult temp_target{std::list<Armor>(), t, 0, 0};
  if (!detection_queue.empty()) {
    temp_target = detection_queue.front();
  }

  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }

  apply_outpost_correction(armors);

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort([](const Armor & a, const Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  // 此时主相机画面中出现了优先级更高的装甲板，切换目标
  else if (state_ == "tracking" && !armors.empty() && armors.front().priority < target_.priority) {
    found = set_target(armors, t);
    tools::logger()->debug("auto_aim switch target to {}", ARMOR_NAMES[armors.front().name]);
  }

  // 此时全向感知相机画面中出现了优先级更高的装甲板，切换目标
  else if (
    state_ == "tracking" && !temp_target.armors.empty() &&
    temp_target.armors.front().priority < target_.priority && target_.convergened()) {
    state_ = "switching";
    switch_target = omniperception::DetectionResult{
      temp_target.armors, t, temp_target.delta_yaw, temp_target.delta_pitch};
    omni_target_priority_ = temp_target.armors.front().priority;
    found = false;
    tools::logger()->debug("omniperception find higher priority target");
  }

  else if (state_ == "switching") {
    found = !armors.empty() && armors.front().priority == omni_target_priority_;
  }

  else if (state_ == "detecting" && pre_state_ == "switching") {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t);
  }

  pre_state_ = state_;
  // 更新状态机
  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {switch_target, {}};  // 返回switch_target和空的targets
  }

  if (state_ == "lost") return {switch_target, {}};  // 返回switch_target和空的targets

  std::list<Target> targets = {target_};
  return {switch_target, targets};
}

void Tracker::state_machine(bool found)
{
  if (state_ == "lost") {
    if (!found) return;

    state_ = "detecting";
    detect_count_ = 1;
    #ifdef NOVA_OUTPOST_V2
    detect_fail_count_ = 0;
    #endif
  }

  #ifndef NOVA_OUTPOST_V2
  else if (state_ == "detecting") {
    if (found) {
      detect_count_++;
      if (detect_count_ >= min_detect_count_) state_ = "tracking";
    } else {
      detect_count_ = 0;
      state_ = "lost";
    }
  }
  #endif
  #ifdef NOVA_OUTPOST_V2
    else if (state_ == "detecting") {
    if (found) {
      detect_count_++;
      detect_fail_count_ = 0;
      auto required_count =
        (target_.name == ArmorName::outpost) ? outpost_min_detect_count_ : min_detect_count_;
      if (detect_count_ >= required_count) state_ = "tracking";
    } else {
      if (target_.name == ArmorName::outpost) {
        detect_fail_count_++;
        if (detect_fail_count_ > outpost_detect_fail_tolerance_) {
          detect_count_ = 0;
          detect_fail_count_ = 0;
          state_ = "lost";
        }
      } else {
        detect_count_ = 0;
        state_ = "lost";
      }
    }
  }
  #endif

  else if (state_ == "tracking") {
    if (found) return;

    temp_lost_count_ = 1;
    state_ = "temp_lost";
  }

  else if (state_ == "switching") {
    if (found) {
      state_ = "detecting";
    } else {
      temp_lost_count_++;
      if (temp_lost_count_ > 200) state_ = "lost";
    }
  }

  else if (state_ == "temp_lost") {
    if (found) {
      state_ = "tracking";
      temp_lost_count_ = 0;  // 参照Auto_Aim：恢复时重置丢失计数，避免累积
    } else {
      temp_lost_count_++;
      if (target_.name == ArmorName::outpost)
        //前哨站的temp_lost_count需要设置的大一些
        max_temp_lost_count_ = outpost_max_temp_lost_count_;
      else
        max_temp_lost_count_ = normal_temp_lost_count_;

      if (temp_lost_count_ > max_temp_lost_count_) state_ = "lost";
    }
  }
}

bool Tracker::set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  if (armors.empty()) return false;

  auto & armor = armors.front();
  solver_.solve(armor);

  // 根据兵种优化初始化参数
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);

  if (is_balance) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(armor, t, 0.2, 2, P0_dig);
  }

  else if (armor.name == ArmorName::outpost) {
    // h1(9)和h2(10)初始协方差设为0.1，允许滤波器在初始化后快速学习Z轴高度差
    // 参照Auto_Aim OutpostTarget的初始化逻辑，避免P=0导致Kalman增益为0无法更新
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0.1, 0.1}};
    target_ = Target(armor, t, 0.2765, 3, P0_dig);
  }

  else if (armor.name == ArmorName::base) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(armor, t, 0.3205, 3, P0_dig);
  }

  else {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(armor, t, 0.2, 4, P0_dig);
  }

  return true;
}

bool Tracker::update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  target_.predict(t);

  int found_count = 0;
  double min_x = 1e10;  // 画面最左侧
  for (const auto & armor : armors) {
    if (armor.name != target_.name || armor.type != target_.armor_type) continue;
    found_count++;
    min_x = armor.center.x < min_x ? armor.center.x : min_x;
  }

  if (found_count == 0) return false;

  for (auto & armor : armors) {
    if (
      armor.name != target_.name || armor.type != target_.armor_type
      //  || armor.center.x != min_x
    )
      continue;

    solver_.solve(armor);

    target_.update(armor);
  }

  return true;
}

}  // namespace auto_aim