#include "buff_tracker.hpp"
#include "tools/math_tools.hpp"


namespace auto_buff
{
Tracker::Tracker()
: big_target_(), 
  state_(lost),
  min_detect_count_(5),
  max_temp_lost_count_(15),
  detect_count_(0), 
  last_timestamp_(std::chrono::steady_clock::now())
{

}

void Tracker::track(std::optional<PowerRune> & p, BigTarget &target, std::chrono::steady_clock::time_point t)
{
  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;
    // tools::logger()->debug("[Tracker] {:.5f}", dt);

  // Handle lost state logic
  if (state_ == lost)
  {
    if (p.has_value()) {
      state_ = detecting;
      big_target_ = target;
      big_target_.get_target(p, t);
      tools::logger()->debug("[Tracker] detecting");
      detect_count_ = 1;
    } else {
      detect_count_ = 0;
    }
  }

  // Handle detecting state logic
  if (state_ == detecting)
  {
    if (p.has_value()) {
      big_target_.get_target(p, t);
      detect_count_++;
      if (detect_count_ >= min_detect_count_) {
        state_ = tracking;
        tools::logger()->debug("[Tracker] tracking");
      }
    } else {
      state_ = lost;
      detect_count_ = 0;
    }
  }
  
  // Handle tracking state logic
  if (state_ == tracking)
  {
    if (p.has_value()) {
      big_target_.get_target(p, t);
    } else {
      state_ = temp_lost;
      detect_count_ = 0;
      big_target_.predict(dt);
      // tools::logger()->debug("[Tracker] temp lost");
    }
  }

  // Handle temporary lost state logic
  if (state_ == temp_lost)
  {
    if (p.has_value()) {
      state_ = tracking;
      big_target_.get_target(p, t);
    } else {
      big_target_.predict(dt);
      detect_count_++;
      if (detect_count_ >= max_temp_lost_count_) {
        state_ = lost;
        detect_count_ = 0;
      }
    }
  }

  if (big_target_.ekf_x().size() > 0) {
    target = big_target_;
  }

  if (p.has_value()){
    // p.value().angle = big_target_.p_r.angle;
    // tools::logger()->debug("[angle] {:.5f}", big_target_.angle * 57.3);
  // p.fanblades = p_r.fanblades[i].type;
  }
  // big_target_.angle = big_target_.p_r.angle;
  

}

void Tracker::set_blade_target(std::optional<PowerRune> last_powerrune)
{
  if (!last_powerrune.has_value()) return;

  
}

}