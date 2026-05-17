#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <string>

#include "armor.hpp"
#include "solver.hpp"
#include "target.hpp"
#include "tasks/omniperception/perceptron.hpp"
#include "tools/thread_safe_queue.hpp"

namespace auto_aim
{
class Tracker
{
public:
  Tracker(const std::string & config_path, Solver & solver);

  std::string state() const;

  std::list<Target> track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
    bool use_enemy_color = true);

  std::tuple<omniperception::DetectionResult, std::list<Target>> track(
    const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
    std::chrono::steady_clock::time_point t, bool use_enemy_color = true);

private:
  Solver & solver_;
  Color enemy_color_;
  bool enemy_color_auto_;
  int min_detect_count_;
  int max_temp_lost_count_;
  int detect_count_;
  int temp_lost_count_;
  int outpost_max_temp_lost_count_;
  int normal_temp_lost_count_;
  std::string state_, pre_state_;
  Target target_;
  std::chrono::steady_clock::time_point last_timestamp_;
  ArmorPriority omni_target_priority_;

  bool outpost_correction_enable_;
  bool outpost_correction_active_;
  int outpost_correction_min_detect_count_;
  int outpost_correction_cancel_count_;
  int outpost_seen_streak_;
  int non_outpost_seen_streak_;

  bool outpost_top_filter_enable_;
  double outpost_top_pitch_;
  double outpost_front_pitch_;

  #ifdef NOVA_OUTPOST_V2
  int outpost_min_detect_count_;
  int outpost_detect_fail_tolerance_;
  int detect_fail_count_;
  #endif

  void state_machine(bool found);

  bool set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);

  bool update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);

  void refresh_enemy_color_from_serial();

  void apply_outpost_correction(std::list<Armor> & armors);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP