#include "shooter.hpp"

#ifdef HERO_OUTPOST_FILTER
#include <limits>
#endif
#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Shooter::Shooter(const std::string & config_path) : last_command_{false, false, 0, 0}
{
  auto yaml = YAML::LoadFile(config_path);
  first_tolerance_ = yaml["first_tolerance"].as<double>() / 57.3;    // degree to rad
  second_tolerance_ = yaml["second_tolerance"].as<double>() / 57.3;  // degree to rad
  judge_distance_ = yaml["judge_distance"].as<double>();
  auto_fire_ = yaml["auto_fire"].as<bool>();
  #ifdef HERO_OUTPOST_FILTER
  outpost_top_plate_fire_disable_ = yaml["outpost_top_plate_fire_disable"].as<bool>(false);
  #endif
}

bool Shooter::shoot(
  const io::Command & command, const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos)
{
  if (!command.control || targets.empty() || !auto_fire_) return false;

  auto target_x = targets.front().ekf_x()[0];
  auto target_y = targets.front().ekf_x()[2];
  auto tolerance = std::sqrt(tools::square(target_x) + tools::square(target_y)) > judge_distance_
                     ? second_tolerance_
                     : first_tolerance_;
  // tools::logger()->debug("d(command.yaw) is {:.4f}", std::abs(last_command_.yaw - command.yaw));

  #ifdef HERO_OUTPOST_FILTER
  bool fire_allowed = false;
  #endif

  #ifndef NOVA_AIM_CENTER
  if (
    std::abs(last_command_.yaw - command.yaw) < tolerance * 2 &&  //此时认为command突变不应该射击
    std::abs(gimbal_pos[0] - last_command_.yaw) < tolerance &&    //应该减去上一次command的yaw值
    aimer.debug_aim_point.valid) {
    #ifdef HERO_OUTPOST_FILTER
    fire_allowed = true;
    #endif
    #ifndef HERO_OUTPOST_FILTER
    last_command_ = command;
    return true;
    #endif
  }
  #endif

  #ifdef NOVA_AIM_CENTER
  if (
    std::abs(last_command_.yaw - command.yaw) < tolerance * 2 &&  //此时认为command突变不应该射击
    std::abs(gimbal_pos[0] - last_command_.yaw) < tolerance &&    //应该减去上一次command的yaw值
    aimer.debug_aim_point.valid &&
    aimer.check_center_fire(targets.front())) {                   //锁中心模式下必须装甲板接近中心才开火
    #ifdef HERO_OUTPOST_FILTER
      fire_allowed = true;
    #endif
    #ifndef HERO_OUTPOST_FILTER
    last_command_ = command;
    return true;
    #endif
  }
  #endif

  #ifdef HERO_OUTPOST_FILTER
  if (fire_allowed && outpost_top_plate_fire_disable_) {
    const auto & target = targets.front();
    if (target.name == ArmorName::outpost && aimer.debug_aim_point.valid) {
      auto aim_xyz = aimer.debug_aim_point.xyza.head<3>();
      auto armors = target.armor_xyza_list();
      int closest_id = 0;
      double min_dist = std::numeric_limits<double>::max();
      for (int i = 0; i < static_cast<int>(armors.size()); i++) {
        double dist = (armors[i].head<3>() - aim_xyz).squaredNorm();
        if (dist < min_dist) {
          min_dist = dist;
          closest_id = i;
        }
      }
      if (target.is_outpost_top_plate(closest_id)) fire_allowed = false;
    }
  }

  if (fire_allowed) {
    last_command_ = command;
    return true;
  }
  #endif

  last_command_ = command;
  return false;
}

}  // namespace auto_aim