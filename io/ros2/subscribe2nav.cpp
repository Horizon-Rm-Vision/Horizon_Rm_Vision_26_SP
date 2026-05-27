#include "subscribe2nav.hpp"

#include <sstream>
#include <vector>

namespace io
{

Subscribe2Nav::Subscribe2Nav()
: Node("nav_subscriber"),
  enemy_statue_queue_(1),
  autoaim_target_queue_(1),
  nav_velocity_queue_(1),
  enemy_status_counter_(0),
  autoaim_target_counter_(0),
  gimbal_form_queue_(1),
  form_queue_(1),
  buff_queue_(1)
{
  enemy_status_subscription_ = this->create_subscription<sp_msgs::msg::EnemyStatusMsg>(
    "enemy_status", 10,
    std::bind(&Subscribe2Nav::enemy_status_callback, this, std::placeholders::_1));

  autoaim_target_subscription_ = this->create_subscription<sp_msgs::msg::AutoaimTargetMsg>(
    "autoaim_target", 10,
    std::bind(&Subscribe2Nav::autoaim_target_callback, this, std::placeholders::_1));

  nav_velocity_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&Subscribe2Nav::nav_velocity_callback, this, std::placeholders::_1));

  gimbal_form_subscription_ = this->create_subscription<std_msgs::msg::Int8>(
    "gimbal_dk",10,
    std::bind(&Subscribe2Nav::gimbal_form_callback,this,std::placeholders::_1));
  
  sentry_form_subscription_ = this->create_subscription<std_msgs::msg::Int8>(
    "form",10,
    std::bind(&Subscribe2Nav::sentry_form_callback,this,std::placeholders::_1));

  buff_subscription_ = this->create_subscription<std_msgs::msg::Int8>(
    "fu",10,
    std::bind(&Subscribe2Nav::buff_callback,this,std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "nav_subscriber node initialized.");
}

Subscribe2Nav::~Subscribe2Nav()
{
  RCLCPP_INFO(this->get_logger(), "nav_subscriber node shutting down.");
}

void Subscribe2Nav::enemy_status_callback(const sp_msgs::msg::EnemyStatusMsg::SharedPtr msg)
{
  enemy_statue_queue_.clear();
  enemy_statue_queue_.push(*msg);

  enemy_status_counter_++;

  if (enemy_status_counter_ >= 2) {
    if (enemy_status_timer_) {
      enemy_status_timer_->cancel();
    }
    enemy_status_timer_ = this->create_wall_timer(std::chrono::milliseconds(1500), [this]() {
      enemy_statue_queue_.clear();
      enemy_status_counter_ = 0;
      RCLCPP_INFO(
        this->get_logger(), "Enemy status queue cleared due to inactivity after two messages.");
    });
  }
}

void Subscribe2Nav::autoaim_target_callback(const sp_msgs::msg::AutoaimTargetMsg::SharedPtr msg)
{
  autoaim_target_queue_.clear();
  autoaim_target_queue_.push(*msg);

  autoaim_target_counter_++;

  if (autoaim_target_counter_ >= 2) {
    if (autoaim_target_timer_) {
      autoaim_target_timer_->cancel();
    }
    autoaim_target_timer_ = this->create_wall_timer(std::chrono::milliseconds(1500), [this]() {
      autoaim_target_queue_.clear();
      autoaim_target_counter_ = 0;
      RCLCPP_INFO(
        this->get_logger(), "Autoaim target queue cleared due to inactivity after two messages.");
    });
  }
}

void Subscribe2Nav::nav_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    nav_velocity_queue_.clear();  // 只保留最新消息（类似原有逻辑）
    nav_velocity_queue_.push(*msg);
    // 如果需要超时清除，可参照 enemy_status 添加定时器，此处省略
}

void Subscribe2Nav::gimbal_form_callback(const std_msgs::msg::Int8::SharedPtr msg)
{
    gimbal_form_queue_.clear(); 
    gimbal_form_queue_.push(*msg);
}

void Subscribe2Nav::sentry_form_callback(const std_msgs::msg::Int8::SharedPtr msg){
  form_queue_.clear();
  form_queue_.push(*msg);
}

void Subscribe2Nav::buff_callback(const std_msgs::msg::Int8::SharedPtr msg){
  buff_queue_.clear();
  buff_queue_.push(*msg);
}

void Subscribe2Nav::start()
{
  RCLCPP_INFO(this->get_logger(), "nav_subscriber node Starting to spin...");
  rclcpp::spin(this->shared_from_this());
}

std::vector<int8_t> Subscribe2Nav::subscribe_enemy_status()
{
  if (enemy_statue_queue_.empty()) {
    return std::vector<int8_t>();
  }
  sp_msgs::msg::EnemyStatusMsg msg;

  enemy_statue_queue_.back(msg);
  RCLCPP_INFO(
    this->get_logger(), "Subscribe enemy_status at: %d.%09u", msg.timestamp.sec,
    msg.timestamp.nanosec);

  return msg.invincible_enemy_ids;
}

std::vector<int8_t> Subscribe2Nav::subscribe_autoaim_target()
{
  if (autoaim_target_queue_.empty()) {
    return std::vector<int8_t>();
  }
  sp_msgs::msg::AutoaimTargetMsg msg;

  autoaim_target_queue_.back(msg);
  RCLCPP_INFO(
    this->get_logger(), "Subscribe autoaim_target at: %d.%09u", msg.timestamp.sec,
    msg.timestamp.nanosec);

  return msg.target_ids;
}

std::optional<geometry_msgs::msg::Twist> Subscribe2Nav::get_nav_velocity()
{
    if (nav_velocity_queue_.empty()) {
        return std::nullopt;
    }
    geometry_msgs::msg::Twist msg;
    nav_velocity_queue_.back(msg);  // 获取最新消息
    return msg;
}

std::optional<std_msgs::msg::Int8> Subscribe2Nav::get_gimbal_form()
{
    if (gimbal_form_queue_.empty()) {
        return std::nullopt;
    }
    std_msgs::msg::Int8 msg;
    gimbal_form_queue_.back(msg);  // 获取最新消息
    return msg;
}

std_msgs::msg::Int8 Subscribe2Nav::subscribe_form()
{
  std_msgs::msg::Int8 msg;
  if(form_queue_.empty()) {
    return msg;
  }
  form_queue_.back(msg);
  
  return msg;
}

std_msgs::msg::Int8 Subscribe2Nav::subscribe_buff()
{
  std_msgs::msg::Int8 msg;
  if(buff_queue_.empty()) {
    return msg;
  }
  buff_queue_.back(msg);
  
  return msg;
}

}  // namespace io