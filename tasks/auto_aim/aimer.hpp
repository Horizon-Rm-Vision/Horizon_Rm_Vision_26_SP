#ifndef AUTO_AIM__AIMER_HPP
#define AUTO_AIM__AIMER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>

#include "io/cboard.hpp"
#include "io/command.hpp"
#include "target.hpp"
#include "tools/trajectory.hpp"  // for trajectory model enum

namespace auto_aim
{

struct AimPoint
{
  bool valid;
  Eigen::Vector4d xyza;
};

class Aimer
{
public:
  AimPoint debug_aim_point;
  explicit Aimer(const std::string & config_path);
  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    bool to_now = true);

  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    io::ShootMode shoot_mode, bool to_now = true);

  double get_gyro_speed_threshold() const { return gyro_speed_threshold_; }
  double get_gyro_angle_threshold() const { return gyro_angle_threshold_; }

  #ifdef NOVA_AIM_CENTER
  bool center_tracked() const { return center_tracked_; }
  double aim_center_angle_tolerance() const { return aim_center_angle_tolerance_; }
  bool check_center_fire(const Target & target) const;
  #endif

private:
  double yaw_offset_;
  std::optional<double> left_yaw_offset_, right_yaw_offset_;
  double pitch_offset_;
  double comming_angle_;
  double leaving_angle_;
  double lock_id_ = -1;
  double high_speed_delay_time_;
  double low_speed_delay_time_;
  double decision_speed_;
  tools::Trajectory::Model ballistic_model_ = tools::Trajectory::Model::kNoDrag;
  double gyro_angle_threshold_;
  double gyro_speed_threshold_;

  #ifdef NOVA_AIM_CENTER
  bool track_center_;
  bool center_tracked_ = false;
  double aim_center_palstance_threshold_;
  double switch_trackmode_threshold_;
  double aim_center_angle_tolerance_;
  #endif

  AimPoint choose_aim_point(const Target & target);
  
  #ifdef NOVA_AIM_CENTER
  Eigen::Vector4d compute_facing_armor(const Target & target) const;
  #endif
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIMER_HPP