#pragma once
#include <optional>
#include <chrono>

#include "buff_target.hpp"
#include "buff_detector.hpp"

namespace auto_buff
{
class Tracker
{
public:
  int last_blade_id_ = 0;

  Tracker();
  void track(std::optional<PowerRune> & p, BigTarget &target, std::chrono::steady_clock::time_point t);
  // void update(const std::optional<PowerRune> & p, std::chrono::steady_clock::time_point timestamp);

private:
  BigTarget big_target_;
  enum State{
    lost,
    detecting,
    tracking,
    temp_lost
  } state_;
  int detect_count_;
  int min_detect_count_;
  int max_temp_lost_count_;
  int switch_count_ = 0;
  std::chrono::steady_clock::time_point last_timestamp_;

  void set_blade_target(std::optional<PowerRune> p);

};
}
