#ifndef IO__SUBSCRIBE2NAV_HPP
#define IO__SUBSCRIBE2NAV_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sp_msgs/msg/detail/autoaim_target_msg__struct.hpp>
#include <vector>
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int8.hpp"
#include "sp_msgs/msg/autoaim_target_msg.hpp"
#include "sp_msgs/msg/enemy_status_msg.hpp"
#include "sp_msgs/sp_msgs/msg/nav_velocity_msg.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
class Subscribe2Nav : public rclcpp::Node
{
public:
  Subscribe2Nav();

  ~Subscribe2Nav();

  void start();

  std::vector<int8_t> subscribe_enemy_status();
  std::vector<int8_t> subscribe_autoaim_target();
  std::optional<geometry_msgs::msg::Twist> get_nav_velocity();
  std_msgs::msg::Int8 subscribe_form();

private:
  void enemy_status_callback(const sp_msgs::msg::EnemyStatusMsg::SharedPtr msg);
  void autoaim_target_callback(const sp_msgs::msg::AutoaimTargetMsg::SharedPtr msg);
  void nav_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void sentry_form_callback(const std_msgs::msg::Int8::SharedPtr msg);

  int enemy_status_counter_;
  int autoaim_target_counter_;
  int nav_velocity_counter_;

  rclcpp::TimerBase::SharedPtr enemy_status_timer_;
  rclcpp::TimerBase::SharedPtr autoaim_target_timer_;
  rclcpp::TimerBase::SharedPtr nav_velocity_timer_;

  rclcpp::Subscription<sp_msgs::msg::EnemyStatusMsg>::SharedPtr enemy_status_subscription_;
  rclcpp::Subscription<sp_msgs::msg::AutoaimTargetMsg>::SharedPtr autoaim_target_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_velocity_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sentry_form_subscription_;

  tools::ThreadSafeQueue<sp_msgs::msg::EnemyStatusMsg> enemy_statue_queue_;
  tools::ThreadSafeQueue<sp_msgs::msg::AutoaimTargetMsg> autoaim_target_queue_;
  tools::ThreadSafeQueue<geometry_msgs::msg::Twist> nav_velocity_queue_;
  tools::ThreadSafeQueue<std_msgs::msg::Int8> form_queue_; 

};
}  // namespace io

#endif  // IO__SUBSCRIBE2NAV_HPP
