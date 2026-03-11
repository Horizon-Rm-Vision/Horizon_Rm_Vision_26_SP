#ifndef TOOLS__THREAD_SAFE_QUEUE_HPP
#define TOOLS__THREAD_SAFE_QUEUE_HPP

#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <deque>

namespace tools
{
template <typename T, bool PopWhenFull = false>
class ThreadSafeQueue
{
public:
  ThreadSafeQueue(
    size_t max_size, std::function<void(void)> full_handler = [] {})
  : max_size_(max_size), full_handler_(full_handler)
  {
  }

  void push(const T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    if (queue_.size() >= max_size_) {
      if (PopWhenFull) {
        queue_.pop_front();
      } else {
        full_handler_();
        return;
      }
    }

    queue_.push_back(value);
    not_empty_condition_.notify_all();
  }

  void pop(T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

    if (queue_.empty()) {
      std::cerr << "Error: Attempt to pop from an empty queue." << std::endl;
      return;
    }

    value = queue_.front();
    queue_.pop_front();
  }

  T pop()
  {
    std::unique_lock<std::mutex> lock(mutex_);

    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

    T value = std::move(queue_.front());
    queue_.pop_front();
    return std::move(value);
  }

  T peek(size_t index) const
  {
    std::unique_lock<std::mutex> lock(mutex_);
    not_empty_condition_.wait(lock, [this, index] { return queue_.size() > index; });
    return queue_[index];
  }

  std::pair<T, T> peek2() const
{
  std::unique_lock<std::mutex> lock(mutex_);
  not_empty_condition_.wait(lock, [this] { return queue_.size() >= 2; });
  return {queue_[0], queue_[1]};
}

  T front()
  {
    std::unique_lock<std::mutex> lock(mutex_);

    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

    return queue_.front();
  }

  void back(T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    if (queue_.empty()) {
      std::cerr << "Error: Attempt to access the back of an empty queue." << std::endl;
      return;
    }

    value = queue_.back();
  }

  bool empty() const
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  size_t size() const
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return queue_.size();
  }

  void clear()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    queue_.clear();
    not_empty_condition_.notify_all();
  }

private:
  std::deque<T> queue_;
  size_t max_size_;
  mutable std::mutex mutex_;
  mutable std::condition_variable not_empty_condition_;
  std::function<void(void)> full_handler_;
};

}  // namespace tools

#endif  // TOOLS__THREAD_SAFE_QUEUE_HPP