#ifndef AUTO_BUFF__TRACKER_HPP
#define AUTO_BUFF__TRACKER_HPP

#include <chrono>
#include <memory>
#include <optional>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "buff_solver.hpp"
#include "buff_target.hpp"
#include "buff_type.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_buff
{

enum class BladeRole { UNKNOWN, FIRST, LAST };

class BigBuffSelector
{
public:
  void reset();
  bool is_initialized() const { return initialized_; }

  // Select which blade to track. Returns index into detected.fanblades, or -1 if none.
  // img_center is used for initial "closest to gimbal" selection.
  int select(const PowerRune & detected, const cv::Point2f & img_center);

  // For UI: determine the role of a detected blade by matching to remembered pair.
  BladeRole get_blade_role(const cv::Point2f & blade_center, const cv::Point2f & r_center) const;

  // Check if we're currently tracking the first blade (for debug UI).
  bool is_tracking_first() const { return initialized_ && !first_gone_; }

  // --- 可调参数（由 YAML 配置加载）---
  int miss_threshold = 12;    // 连续丢失帧数阈值，确认 first blade 已消失
  int reset_threshold = 50;   // 两 blade 均丢失帧数阈值，触发完全复位
  float match_dist = 70.0f;   // blade 身份匹配的距离阈值（像素）

  #ifdef BIG_BUFF_DUO_EKF
  // Match a blade center to FIRST (0), LAST (1), or neither (-1).
  // Public so BuffTracker can build dual observations after select() updates memory.
  int match_to_memory(const cv::Point2f & blade_center, const cv::Point2f & r_center) const;
  #endif

private:
  struct BladeMemory {
    cv::Point2f center;
    double angle;  // radians around r_center
  };
#ifndef BIG_BUFF_DUO_EKF
  int match_to_memory(const cv::Point2f & blade_center, const cv::Point2f & r_center) const;
#endif

  BladeMemory first_blade_;
  BladeMemory last_blade_;
  #ifdef BIG_BUFF_FIRE_FIX
  double angular_offset_ = 0;  // last - first 的相对角度 (rad), 用于在跟踪 first 时推算 last 位置
  #endif
  bool initialized_ = false;
  bool first_gone_ = false;
  int first_miss_count_ = 0;
  int both_miss_count_ = 0;
};

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
    // BigBuffSelector 可调参数
    int miss_threshold = 12;    // 连续丢失帧数，确认 first blade 消失（帧数越低越快判定消失）
    int reset_threshold = 50;   // 两 blade 均丢失帧数，触发完全复位（帧数越低越快复位）
    float match_dist = 70.0f;   // blade 身份匹配距离阈值，像素（越大越容易匹配但可能误匹配）
  };

  BuffTracker(const std::string & config_path, PowerRune_type type = SMALL);

  void set_type(PowerRune_type type);

  void set_img_size(int cols, int rows) { img_center_ = cv::Point2f(cols / 2.0f, rows / 2.0f); }

  bool update(
    const std::optional<PowerRune> & detected,
    std::chrono::steady_clock::time_point & timestamp,
    Solver & solver);

  State state() const { return state_; }
  bool is_tracking() const { return state_ == State::TRACKING || state_ == State::TEMP_LOST; }

  Target & target();
  std::unique_ptr<Target> clone_target() const;
  std::optional<PowerRune> last_observation() const { return last_selected_; }

  // Big buff dual-blade selector access (for UI)
  const BigBuffSelector & selector() const { return selector_; }
  BladeRole get_blade_role(const cv::Point2f & blade_center, const cv::Point2f & r_center) const
  {
    return selector_.get_blade_role(blade_center, r_center);
  }

#ifdef BIG_BUFF_DUO_EKF
  // Access the second blade's target (for debug UI).
  // Only valid in BIG mode; returns nullptr in SMALL mode.
  Target * target_second() const { return target_second_.get(); }
#endif

private:
  std::optional<PowerRune> select_observation(const PowerRune & detected, Solver & solver);
#ifdef BIG_BUFF_DUO_EKF
  std::optional<PowerRune> build_blade_observation(
    const PowerRune & detected, int blade_idx, Solver & solver);
  bool update_big_dual(
    const PowerRune & detected,
    std::chrono::steady_clock::time_point & timestamp,
    Solver & solver,
    std::optional<PowerRune> & selected_out);
#endif
  double compute_gate_distance(const PowerRune & obs) const;
  bool passes_gate(const PowerRune & obs) const;
  void update_fsm(bool found, std::chrono::steady_clock::time_point timestamp);
  void reset_target();

  Config config_;
  State state_ = State::LOST;
  PowerRune_type type_ = SMALL;

  std::unique_ptr<Target> target_;         // primary / active target
#ifdef BIG_BUFF_DUO_EKF
  std::unique_ptr<Target> target_second_;  // second blade's EKF (BIG mode only)
#endif
  std::optional<PowerRune> last_selected_;
  BigBuffSelector selector_;
  cv::Point2f img_center_{0, 0};

  int detect_count_ = 0;
  int lost_count_ = 0;
  std::chrono::steady_clock::time_point last_found_time_{};
  bool has_found_time_ = false;
};

}  // namespace auto_buff

#endif  // AUTO_BUFF__TRACKER_HPP
