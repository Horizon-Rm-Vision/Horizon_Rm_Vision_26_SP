#ifndef AUTO_BUFF__TRACKER_HPP
#define AUTO_BUFF__TRACKER_HPP

#include <chrono>
#include <memory>
#include <optional>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <map>

#include "buff_solver.hpp"
#include "buff_target.hpp"
#include "buff_type.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_buff
{
class BuffTracker
{
public:
  enum class State { LOST, DETECTING, TRACKING, TEMP_LOST };

  struct Config
  {
    double lost_time_thres = 2.0;
    int tracking_thres = 5;
    double match_gate = 15.0;
    double max_dis_diff = 1.0;
    double r_yaw_var = 0.01;
    double r_pitch_var = 0.01;
    double r_dis_var = 0.5;
    double r_roll_var = 0.1;

    /// 4D Mahalanobis 门控复用 match_gate
  };

  BuffTracker(const std::string & config_path, PowerRune_type type = SMALL);

  void set_type(PowerRune_type type);

  bool update(
    const std::optional<PowerRune> & detected,
    std::chrono::steady_clock::time_point & timestamp,
    Solver & solver);

  State state() const { return state_; }
  bool is_tracking() const { return state_ == State::TRACKING || state_ == State::TEMP_LOST; }

  Target & target();
  std::unique_ptr<Target> clone_target() const;
  std::optional<PowerRune> last_observation() const { return last_selected_; }

private:
  /// WUST 式多目标 ID 匹配: PnP+4D Mahalanobis → 编号 0..4
  std::optional<PowerRune> matchObservations(const PowerRune & detected, Solver & solver);

  std::optional<PowerRune> select_observation(const PowerRune & detected, Solver & solver);
  double compute_gate_distance(const PowerRune & obs) const;
  bool passes_gate(const PowerRune & obs) const;
  void update_fsm(bool found, std::chrono::steady_clock::time_point timestamp);
  void reset_target();

  Config config_;
  State state_ = State::LOST;
  PowerRune_type type_ = SMALL;

  std::unique_ptr<Target> target_;
  std::optional<PowerRune> last_selected_;

  int detect_count_ = 0;
  int lost_count_ = 0;
  std::chrono::steady_clock::time_point last_found_time_{};
  bool has_found_time_ = false;
};

}  // namespace auto_buff

#endif  // AUTO_BUFF__TRACKER_HPP
