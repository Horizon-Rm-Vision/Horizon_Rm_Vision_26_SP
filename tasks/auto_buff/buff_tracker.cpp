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
}  // namespace

// ======== BigBuffSelector implementation ========

void BigBuffSelector::reset()
{
  initialized_ = false;
  first_gone_ = false;
  first_miss_count_ = 0;
  both_miss_count_ = 0;
#ifdef BIG_BUFF_FIRE_FIX
  angular_offset_ = 0;
#endif
}

int BigBuffSelector::match_to_memory(
  const cv::Point2f & blade_center, const cv::Point2f & r_center) const
{
  if (!initialized_) return -1;

  float d0 = cv::norm(blade_center - first_blade_.center);
  float d1 = cv::norm(blade_center - last_blade_.center);

  if (d0 < match_dist && d0 < d1) return 0;   // matches first
  if (d1 < match_dist && d1 < d0) return 1;   // matches last

  // Fall back to angle-based matching if position is ambiguous
  double angle =
    std::atan2(blade_center.y - r_center.y, blade_center.x - r_center.x);
  double angle_diff_0 = std::abs(tools::limit_rad(angle - first_blade_.angle));
  double angle_diff_1 = std::abs(tools::limit_rad(angle - last_blade_.angle));

  if (angle_diff_0 < angle_diff_1) return 0;
  if (angle_diff_1 < angle_diff_0) return 1;
  return -1;
}

int BigBuffSelector::select(const PowerRune & detected, const cv::Point2f & img_center)
{
  // Build list of valid (non-unlit, has points) blade indices
  std::vector<int> valid;
  for (int i = 0; i < static_cast<int>(detected.fanblades.size()); ++i) {
    const auto & b = detected.fanblades[i];
    if (b.type == _unlight) continue;
    if (b.points.size() < 4) continue;
    valid.push_back(i);
  }

  int n = static_cast<int>(valid.size());

  // ------ Not initialized: wait for a pair or track single ------
  if (!initialized_) {
    if (n == 2) {
      int i0 = valid[0], i1 = valid[1];
      const auto & b0 = detected.fanblades[i0];
      const auto & b1 = detected.fanblades[i1];

      float d0 = cv::norm(b0.center - img_center);
      float d1 = cv::norm(b1.center - img_center);

      if (d0 <= d1) {
        first_blade_.center = b0.center;
        last_blade_.center = b1.center;
        first_blade_.angle = std::atan2(
          b0.center.y - detected.r_center.y, b0.center.x - detected.r_center.x);
        last_blade_.angle = std::atan2(
          b1.center.y - detected.r_center.y, b1.center.x - detected.r_center.x);
        #ifdef BIG_BUFF_FIRE_FIX
          angular_offset_ = tools::limit_rad(last_blade_.angle - first_blade_.angle);
        #endif
        initialized_ = true;
        return i0;
      } else {
        first_blade_.center = b1.center;
        last_blade_.center = b0.center;
        first_blade_.angle = std::atan2(
          b1.center.y - detected.r_center.y, b1.center.x - detected.r_center.x);
        last_blade_.angle = std::atan2(
          b0.center.y - detected.r_center.y, b0.center.x - detected.r_center.x);
        #ifdef BIG_BUFF_FIRE_FIX
          angular_offset_ = tools::limit_rad(last_blade_.angle - first_blade_.angle);
        #endif
        initialized_ = true;
        return i1;
      }
    }
    // Single blade before pair seen: track it but don't form pair
    if (n == 1) return valid[0];
    return -1;
  }

  // ------ Initialized: match detections to remembered pair ------
  int first_idx = -1, last_idx = -1;
  for (int idx : valid) {
    int m = match_to_memory(detected.fanblades[idx].center, detected.r_center);
    if (m == 0) first_idx = idx;
    else if (m == 1) last_idx = idx;
  }

  bool first_vis = (first_idx >= 0);
  bool last_vis = (last_idx >= 0);

  // Update remembered positions of visible blades
  if (first_vis) {
    first_blade_.center = detected.fanblades[first_idx].center;
    first_blade_.angle = std::atan2(
      detected.fanblades[first_idx].center.y - detected.r_center.y,
      detected.fanblades[first_idx].center.x - detected.r_center.x);
  }
  if (last_vis) {
    last_blade_.center = detected.fanblades[last_idx].center;
    last_blade_.angle = std::atan2(
      detected.fanblades[last_idx].center.y - detected.r_center.y,
      detected.fanblades[last_idx].center.x - detected.r_center.x);
  }
  #ifdef BIG_BUFF_FIRE_FIX
  // 用 angular_offset 推算未可见 blade 的当前位置, 防止 memory 随时间变 stale
  if (first_vis && !last_vis) {
    float radius = cv::norm(first_blade_.center - detected.r_center);
    last_blade_.angle = tools::limit_rad(first_blade_.angle + angular_offset_);
    last_blade_.center = cv::Point2f(
      detected.r_center.x + radius * std::cos(last_blade_.angle),
      detected.r_center.y + radius * std::sin(last_blade_.angle));
  } else if (!first_vis && last_vis) {
    float radius = cv::norm(last_blade_.center - detected.r_center);
    first_blade_.angle = tools::limit_rad(last_blade_.angle - angular_offset_);
    first_blade_.center = cv::Point2f(
      detected.r_center.x + radius * std::cos(first_blade_.angle),
      detected.r_center.y + radius * std::sin(first_blade_.angle));
  }
  #endif

  // ------ Decision logic ------
  if (!first_gone_) {
    if (first_vis) {
      first_miss_count_ = 0;
      both_miss_count_ = 0;
      return first_idx;
    }

    first_miss_count_++;
    if (first_miss_count_ >= miss_threshold) {
      first_gone_ = true;
      if (last_vis) {
        both_miss_count_ = 0;
        return last_idx;
      }
    }

    // First not visible but not confirmed gone.
    // Don't jump to last — return -1 so EKF predicts without update.
    if (!last_vis) both_miss_count_++;
    if (both_miss_count_ >= reset_threshold) {
      reset();
    }
    return -1;
  }

  // first_gone_ == true: tracking last blade
  if (last_vis) {
    both_miss_count_ = 0;
    return last_idx;
  }

  both_miss_count_++;
  if (both_miss_count_ >= reset_threshold) {
    reset();
  }
  return -1;
}

BladeRole BigBuffSelector::get_blade_role(
  const cv::Point2f & blade_center, const cv::Point2f & r_center) const
{
  if (!initialized_) return BladeRole::UNKNOWN;

  int m = match_to_memory(blade_center, r_center);
  if (m == 0) return BladeRole::FIRST;
  if (m == 1) return BladeRole::LAST;
  return BladeRole::UNKNOWN;
}

// ======== BuffTracker implementation ========

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
    config_.miss_threshold = node["miss_threshold"].as<int>(config_.miss_threshold);
    config_.reset_threshold = node["reset_threshold"].as<int>(config_.reset_threshold);
    config_.match_dist = node["match_dist"].as<float>(config_.match_dist);
  }

  selector_.miss_threshold = config_.miss_threshold;
  selector_.reset_threshold = config_.reset_threshold;
  selector_.match_dist = config_.match_dist;

  target_ = make_target(type_);
#ifdef BIG_BUFF_DUO_EKF
  if (type_ == BIG) {
    target_second_ = make_target(BIG);
  }
#endif
}

void BuffTracker::set_type(PowerRune_type type)
{
  if (type_ == type && target_) {
    return;
  }
  type_ = type;
  target_ = make_target(type_);
#ifdef BIG_BUFF_DUO_EKF
  if (type_ == BIG) {
    target_second_ = make_target(BIG);
  } else {
    target_second_.reset();
  }
#endif
  state_ = State::LOST;
  detect_count_ = 0;
  lost_count_ = 0;
  has_found_time_ = false;
  selector_.reset();
}

Target & BuffTracker::target()
{
#ifdef BIG_BUFF_DUO_EKF
  // In BIG mode, return the EKF for the actively-tracked blade.
  // Before selector initialization, always return the primary EKF.
  if (type_ == BIG && target_second_ && selector_.is_initialized() &&
      !selector_.is_tracking_first()) {
    return *target_second_;
  }
#endif
  return *target_;
}

std::unique_ptr<Target> BuffTracker::clone_target() const
{
#ifdef BIG_BUFF_DUO_EKF
  if (type_ == BIG && target_second_ && selector_.is_initialized() &&
      !selector_.is_tracking_first()) {
    return target_second_->clone();
  }
#endif
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

#ifdef BIG_BUFF_DUO_EKF
  if (detected.has_value()) {
    if (type_ == BIG) {
      // ---- BIG mode: dual-EKF with independent per-blade tracking ----
      found = update_big_dual(detected.value(), timestamp, solver, selected);
    } else {
      // ---- SMALL mode: original single-EKF path ----
      selected = select_observation(detected.value(), solver);

      if (selected.has_value()) {
        target_->get_target(selected, timestamp);
        found = !target_->is_unsolve();
      } else {
        target_->get_target(std::nullopt, timestamp);
      }
    }
  } else {
    // No detection at all — predict-only for all EKFs
    target_->get_target(std::nullopt, timestamp);
    if (type_ == BIG && target_second_) {
      target_second_->get_target(std::nullopt, timestamp);
    }
  }
#endif
#ifndef BIG_BUFF_DUO_EKF
  if (detected.has_value()) {
    selected = select_observation(detected.value(), solver);

    if (selected.has_value()) {
      target_->get_target(selected, timestamp);
      found = !target_->is_unsolve();
    } else {
      target_->get_target(std::nullopt, timestamp);
    }
  } else {
    target_->get_target(std::nullopt, timestamp);
  }
#endif

  last_selected_ = selected;

  update_fsm(found, timestamp);
  if (state_ == State::LOST) {
    reset_target();
  }

  return found;
}

#ifdef BIG_BUFF_DUO_EKF
std::optional<PowerRune> BuffTracker::build_blade_observation(
  const PowerRune & detected, int blade_idx, Solver & solver)
{
  PowerRune obs = detected;
  if (blade_idx != 0 && static_cast<int>(obs.fanblades.size()) > 0) {
    std::swap(obs.fanblades[0], obs.fanblades[blade_idx]);
  }
  if (!obs.fanblades.empty()) {
    obs.fanblades[0].type = _target;
  }
  auto opt = std::optional<PowerRune>(obs);
  solver.solve(opt);
  return opt;
}

bool BuffTracker::update_big_dual(
  const PowerRune & detected,
  std::chrono::steady_clock::time_point & timestamp,
  Solver & solver,
  std::optional<PowerRune> & selected_out)
{
  // 1. Run BigBuffSelector to update blade memory and get chosen blade index.
  //    This also handles initialization, miss counting, and first_gone_ transitions.
  int chosen = selector_.select(detected, img_center_);

  // 2. Match all valid blades to FIRST / LAST memory.
  //    select() already updated first_blade_ / last_blade_ positions above.
  std::vector<int> valid;
  for (int i = 0; i < static_cast<int>(detected.fanblades.size()); ++i) {
    const auto & b = detected.fanblades[i];
    if (b.type == _unlight) continue;
    if (b.points.size() < 4) continue;
    valid.push_back(i);
  }

  int first_idx = -1, last_idx = -1;
  for (int idx : valid) {
    int m = selector_.match_to_memory(detected.fanblades[idx].center, detected.r_center);
    if (m == 0) first_idx = idx;
    else if (m == 1) last_idx = idx;
  }

  // Pre-initialization fallback: selector hasn't seen a pair yet.
  // Feed any detected blade to the primary EKF without assigning FIRST/LAST identity.
  if (!selector_.is_initialized()) {
    if (chosen >= 0) {
      auto obs = build_blade_observation(detected, chosen, solver);
      target_->get_target(obs, timestamp);
      if (target_second_) {
        target_second_->get_target(std::nullopt, timestamp);
      }
      selected_out = obs;
      return !target_->is_unsolve();
    }
    target_->get_target(std::nullopt, timestamp);
    if (target_second_) {
      target_second_->get_target(std::nullopt, timestamp);
    }
    return false;
  }

  // 3. Build observations and feed to each blade's independent EKF.
  //    This keeps both EKFs converged so switching blades is seamless.
  std::optional<PowerRune> obs_first, obs_last;

  if (first_idx >= 0) {
    obs_first = build_blade_observation(detected, first_idx, solver);
  }
  target_->get_target(
    first_idx >= 0 ? std::optional<PowerRune>(obs_first) : std::nullopt,
    timestamp);

  if (target_second_) {
    if (last_idx >= 0) {
      obs_last = build_blade_observation(detected, last_idx, solver);
    }
    target_second_->get_target(
      last_idx >= 0 ? std::optional<PowerRune>(obs_last) : std::nullopt,
      timestamp);
  }

  // 4. Determine which observation to expose as "selected" (for UI / debug).
  bool tracking_first = selector_.is_tracking_first();
  selected_out = tracking_first ? obs_first : obs_last;

  // 5. Return whether the actively-tracked blade was successfully observed.
  if (chosen < 0) return false;

  if (tracking_first) {
    return first_idx >= 0 && !target_->is_unsolve();
  } else {
    return last_idx >= 0 && target_second_ && !target_second_->is_unsolve();
  }
}
#endif

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

  // ---- BIG mode fallback: use BigBuffSelector (when not using dual-EKF path) ----
  if (type_ == BIG) {
    int chosen = selector_.select(detected, img_center_);

    if (chosen < 0) {
      return std::nullopt;
    }

    PowerRune selected = detected;
    if (chosen != 0 && selected.fanblades.size() > 0) {
      std::swap(selected.fanblades[0], selected.fanblades[chosen]);
    }
    if (!selected.fanblades.empty()) {
      selected.fanblades[0].type = _target;
    }
    auto opt = std::optional<PowerRune>(selected);
    solver.solve(opt);
    return opt;
  }

  // ---- SMALL mode: original gate-distance-based selection ----

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
#ifdef BIG_BUFF_DUO_EKF
  if (target_second_) {
    target_second_->reset();
  }
#endif
  selector_.reset();
}

}  // namespace auto_buff
