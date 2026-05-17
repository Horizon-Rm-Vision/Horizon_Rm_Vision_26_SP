#ifndef TOOLS__POSE_BUFFER_HPP
#define TOOLS__POSE_BUFFER_HPP

#include <chrono>
#include <deque>
#include <mutex>
#include <optional>

#include <Eigen/Geometry>

namespace tools
{
class PoseBuffer
{
public:
  explicit PoseBuffer(size_t max_size = 200) : max_size_(max_size) {}

  void push(const Eigen::Quaterniond & q, std::chrono::steady_clock::time_point t)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!queue_.empty() && t <= queue_.back().second) {
      return;
    }
    queue_.emplace_back(q, t);
    while (queue_.size() > max_size_) {
      queue_.pop_front();
    }
  }

  std::optional<Eigen::Quaterniond> sample(std::chrono::steady_clock::time_point t) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) {
      return std::nullopt;
    }

    if (t <= queue_.front().second) {
      return queue_.front().first;
    }

    for (size_t i = 1; i < queue_.size(); ++i) {
      const auto & a = queue_[i - 1];
      const auto & b = queue_[i];
      if (a.second <= t && t <= b.second) {
        double t_ab = std::chrono::duration_cast<std::chrono::duration<double>>(b.second - a.second)
                        .count();
        double t_ac = std::chrono::duration_cast<std::chrono::duration<double>>(t - a.second).count();
        double k = (t_ab > 1e-9) ? (t_ac / t_ab) : 0.0;
        return a.first.slerp(k, b.first).normalized();
      }
    }

    return queue_.back().first;
  }

private:
  size_t max_size_;
  mutable std::mutex mutex_;
  std::deque<std::pair<Eigen::Quaterniond, std::chrono::steady_clock::time_point>> queue_;
};

}  // namespace tools

#endif  // TOOLS__POSE_BUFFER_HPP
