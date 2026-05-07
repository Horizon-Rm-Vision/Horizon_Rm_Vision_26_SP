#include "buff_tracker.hpp"

#include <limits>

namespace auto_buff
{
namespace
{
std::unique_ptr<Target> make_target(PowerRune_type type)
{
  if (type == BIG) {
    return std::make_unique<BigTarget>();
  }
  return std::make_unique<SmallTarget>();
}
}

BuffTracker::BuffTracker(const std::string & config_path, PowerRune_type type) : type_(type)
{
  auto yaml = YAML::LoadFile(config_path);
  if (yaml["buff_tracker"].IsDefined()) {
    auto node = yaml["buff_tracker"];
    config_.lost_time_thres = node["lost_time_thres"].as<double>(config_.lost_time_thres);
    config_.tracking_thres = node["tracking_thres"].as<int>(config_.tracking_thres);
    config_.match_gate = node["match_gate"].as<double>(config_.match_gate);
    config_.max_dis_diff = node["max_dis_diff"].as<double>(config_.max_dis_diff);
    config_.r_yaw_var = node["r_yaw_var"].as<double>(config_.r_yaw_var);
    config_.r_pitch_var = node["r_pitch_var"].as<double>(config_.r_pitch_var);
    config_.r_dis_var = node["r_dis_var"].as<double>(config_.r_dis_var);
    config_.r_roll_var = node["r_roll_var"].as<double>(config_.r_roll_var);
  }

  target_ = make_target(type_);
}

void BuffTracker::set_type(PowerRune_type type)
{
  if (type_ == type && target_) {
    return;
  }
  type_ = type;
  target_ = make_target(type_);
  state_ = State::LOST;
  detect_count_ = 0;
  lost_count_ = 0;
  has_found_time_ = false;
}

Target & BuffTracker::target()
{
  return *target_;
}

std::unique_ptr<Target> BuffTracker::clone_target() const
{
  if (!target_) {
    return nullptr;
  }
  return target_->clone();
}

bool BuffTracker::update(
  const std::optional<PowerRune> & detected,
  std::chrono::steady_clock::time_point & timestamp,
  Solver & solver)
{
  bool found = false;
  std::optional<PowerRune> selected;

  if (detected.has_value()) {
    // ---- WUST 式多目标匹配 ----
    // 跟踪中 → 概率 ID 匹配; 丢失/初始化 → 回退启发式
    if (is_tracking() && target_ && !target_->is_unsolve()) {
      selected = matchObservations(detected.value(), solver);
    } else {
      selected = select_observation(detected.value(), solver);
    }

    if (selected.has_value()) {
      target_->get_target(selected, timestamp);
      found = !target_->is_unsolve();
    } else {
      target_->get_target(std::nullopt, timestamp);
    }
  } else {
    target_->get_target(std::nullopt, timestamp);
  }

  last_selected_ = selected;

  update_fsm(found, timestamp);
  if (state_ == State::LOST) {
    reset_target();
  }

  return found;
}

std::optional<PowerRune> BuffTracker::select_observation(
  const PowerRune & detected, Solver & solver)
{
  std::vector<int> candidates;
  for (int i = 0; i < static_cast<int>(detected.fanblades.size()); ++i) {
    const auto & blade = detected.fanblades[i];
    if (blade.type == _unlight) {
      continue;
    }
    if (blade.points.size() < 4) {
      continue;
    }
    candidates.push_back(i);
  }

  if (candidates.empty()) {
    return std::nullopt;
  }

  if (!is_tracking() || target_->is_unsolve()) {
    PowerRune selected = detected;
    if (candidates.front() != 0 && selected.fanblades.size() > 0) {
      std::swap(selected.fanblades[0], selected.fanblades[candidates.front()]);
    }
    if (!selected.fanblades.empty()) {
      selected.fanblades[0].type = _target;
    }
    auto opt = std::optional<PowerRune>(selected);
    solver.solve(opt);
    return opt;
  }

  double best_score = std::numeric_limits<double>::infinity();
  std::optional<PowerRune> best;

  for (int idx : candidates) {
    PowerRune candidate = detected;
    if (idx != 0 && candidate.fanblades.size() > 0) {
      std::swap(candidate.fanblades[0], candidate.fanblades[idx]);
    }
    if (!candidate.fanblades.empty()) {
      candidate.fanblades[0].type = _target;
    }
    auto opt = std::optional<PowerRune>(candidate);
    solver.solve(opt);
    if (!opt.has_value()) {
      continue;
    }

    const auto & obs = opt.value();
    if (!passes_gate(obs)) {
      continue;
    }

    double score = compute_gate_distance(obs);
    if (score < best_score) {
      best_score = score;
      best = opt;
    }
  }

  return best;
}

bool BuffTracker::passes_gate(const PowerRune & obs) const
{
  if (target_->is_unsolve()) {
    return true;
  }

  Eigen::Vector3d pred_pos = target_->point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
  if ((pred_pos - obs.xyz_in_world).norm() > config_.max_dis_diff) {
    return false;
  }

  double d2 = compute_gate_distance(obs);
  return d2 < config_.match_gate;
}

double BuffTracker::compute_gate_distance(const PowerRune & obs) const
{
  Eigen::VectorXd x = target_->ekf_x();
  double pred_yaw = x[0];
  double pred_pitch = x[2];
  double pred_dis = x[3];
  double pred_roll = x[5];

  double r_yaw = tools::limit_rad(obs.ypd_in_world[0] - pred_yaw);
  double r_pitch = tools::limit_rad(obs.ypd_in_world[1] - pred_pitch);
  double r_dis = obs.ypd_in_world[2] - pred_dis;
  double r_roll = tools::limit_rad(obs.ypr_in_world[2] - pred_roll);

  double d2 =
    r_yaw * r_yaw / config_.r_yaw_var + r_pitch * r_pitch / config_.r_pitch_var +
    r_dis * r_dis / config_.r_dis_var + r_roll * r_roll / config_.r_roll_var;

  return d2;
}

void BuffTracker::update_fsm(bool found, std::chrono::steady_clock::time_point timestamp)
{
  if (found) {
    last_found_time_ = timestamp;
    has_found_time_ = true;
  }

  switch (state_) {
    case State::LOST:
      if (found) {
        detect_count_ = 1;
        state_ = State::DETECTING;
      }
      break;
    case State::DETECTING:
      if (found) {
        if (++detect_count_ >= config_.tracking_thres) {
          detect_count_ = 0;
          state_ = State::TRACKING;
        }
      } else {
        detect_count_ = 0;
        state_ = State::LOST;
      }
      break;
    case State::TRACKING:
      if (!found) {
        lost_count_ = 1;
        state_ = State::TEMP_LOST;
      }
      break;
    case State::TEMP_LOST:
      if (found) {
        lost_count_ = 0;
        state_ = State::TRACKING;
      }
      break;
  }

  if (has_found_time_) {
    double lost_time =
      std::chrono::duration_cast<std::chrono::duration<double>>(timestamp - last_found_time_)
        .count();
    if (lost_time > config_.lost_time_thres) {
      state_ = State::LOST;
      lost_count_ = 0;
    }
  }
}

void BuffTracker::reset_target()
{
  if (target_) {
    target_->reset();
  }
}

/// WUST式多目标 ID 匹配
///
/// 对检测到的每个可见扇叶:
///   1. PnP 获取扇叶姿态 (roll + 尖端位置)
///   2. 对比 EKF 预测的 roll + k*2π/5 找到最佳 ID (马氏距离思想)
///   3. 门控筛选, 贪心保留, 填入 PowerRune::matched_blades_
///
/// 返回的 PowerRune 中 fanblades[0] 为"主扇叶"(用于 R_center 观测),
/// matched_blades_ 包含所有其它匹配成功的扇叶观测.
std::optional<PowerRune> BuffTracker::matchObservations(
  const PowerRune & detected, Solver & solver)
{
  // ---- 1. 收集可见扇叶, 全部做 PnP 一次 ----
  struct PnPResult {
    int slot;
    Eigen::Vector3d ypd;           // R 中心球坐标 (所有扇叶应一致)
    Eigen::Vector3d ypr;           // roll 用于 ID 匹配
    Eigen::Vector3d blade_xyz;
    Eigen::Vector3d blade_ypd;
  };
  std::vector<PnPResult> pnp_results;

  for (int i = 0; i < static_cast<int>(detected.fanblades.size()); ++i) {
    if (detected.fanblades[i].type == _unlight) continue;
    if (detected.fanblades[i].points.size() < 4) continue;

    Eigen::Vector3d ypd, ypr, bxyz, bypd;
    solver.solveFanBladeCorners(detected.fanblades[i].points, ypd, ypr, bxyz, bypd);
    pnp_results.push_back({i, ypd, ypr, bxyz, bypd});
  }

  if (pnp_results.empty()) return std::nullopt;

  // ---- 2. 预测的基准 roll ----
  double pred_roll = target_->get_roll();  // EKF 状态 x[5]

  // ---- 3. roll 匹配: 找到每个观测对应的 blade_id ----
  // first = blade_id (0..4), second = (error, PnPResult 索引)
  std::map<int, std::pair<double, size_t>> best_per_id;

  for (size_t idx = 0; idx < pnp_results.size(); ++idx) {
    const auto & pr = pnp_results[idx];
    double obs_roll = pr.ypr[2];

    int best_id = -1;
    double best_err = 1e9;
    for (int id = 0; id < 5; ++id) {
      double expected = pred_roll + double(id) * 2.0 * CV_PI / 5.0;
      double err = std::abs(tools::limit_rad(obs_roll - expected));
      if (err < best_err) {
        best_err = err;
        best_id = id;
      }
    }

    if (best_err > config_.blade_match_gate) continue;

    auto it = best_per_id.find(best_id);
    if (it == best_per_id.end() || best_err < it->second.first) {
      best_per_id[best_id] = {best_err, idx};
    }
  }

  if (best_per_id.empty()) return std::nullopt;

  // ---- 4. 选择主扇叶 (最接近预测 roll) ----
  int primary_id = -1;
  size_t primary_idx = 0;
  double min_err = 1e9;
  for (const auto & [id, pair] : best_per_id) {
    if (pair.first < min_err) {
      min_err = pair.first;
      primary_id = id;
      primary_idx = pair.second;
    }
  }

  // ---- 5. 构建输出 PowerRune ----
  int primary_slot = pnp_results[primary_idx].slot;

  PowerRune selected = detected;
  if (primary_slot != 0 && selected.fanblades.size() > 0) {
    std::swap(selected.fanblades[0], selected.fanblades[primary_slot]);
  }
  if (!selected.fanblades.empty()) {
    selected.fanblades[0].type = _target;
  }
  auto opt = std::optional<PowerRune>(selected);
  solver.solve(opt);
  if (!opt.has_value()) return std::nullopt;

  // ---- 6. 填入所有其它匹配观测 ----
  PowerRune & result = opt.value();
  for (const auto & [id, pair] : best_per_id) {
    if (id == primary_id) continue;
    const auto & pr = pnp_results[pair.second];
    result.matched_blades_.push_back(
      {id, pr.blade_ypd, pr.blade_xyz});
  }

  return result;
}

}  // namespace auto_buff
