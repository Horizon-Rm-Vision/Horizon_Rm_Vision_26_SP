#include "ros2.hpp"
namespace io
{
ROS2::ROS2()
{
  rclcpp::init(0, nullptr);

  publish2nav_ = std::make_shared<Publish2Nav>();

  subscribe2nav_ = std::make_shared<Subscribe2Nav>();

  publish_spin_thread_ = std::make_unique<std::thread>([this]() { publish2nav_->start(); });

  subscribe_spin_thread_ = std::make_unique<std::thread>([this]() { subscribe2nav_->start(); });
}

ROS2::~ROS2()
{
  rclcpp::shutdown();
  publish_spin_thread_->join();
  subscribe_spin_thread_->join();
}

void ROS2::publish_status(uint8_t game_progress,uint16_t stage_remain_time,uint16_t current_hp,uint16_t ally_outpost_hp,uint8_t state,uint8_t energy_state,uint16_t bullets,uint8_t judge)
{
    publish2nav_->send_status(game_progress, stage_remain_time, current_hp, ally_outpost_hp, state, energy_state, bullets, judge);
}

std::optional<geometry_msgs::msg::Twist> ROS2::get_nav_velocity()
{
    return subscribe2nav_->get_nav_velocity();
}

std::optional<std_msgs::msg::Int8> ROS2::get_gimbal_form()
{
    return subscribe2nav_->get_gimbal_form();
}

std_msgs::msg::Int8 ROS2::subscribe_form()
{
    return subscribe2nav_->subscribe_form();
}

void ROS2::publish(const Eigen::Vector4d & target_pos) { publish2nav_->send_data(target_pos); }

std::vector<int8_t> ROS2::subscribe_enemy_status()
{
  return subscribe2nav_->subscribe_enemy_status();
}

std::vector<int8_t> ROS2::subscribe_autoaim_target()
{
  return subscribe2nav_->subscribe_autoaim_target();
}

}  // namespace io
