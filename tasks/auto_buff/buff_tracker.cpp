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

}  // namespace auto_buff
