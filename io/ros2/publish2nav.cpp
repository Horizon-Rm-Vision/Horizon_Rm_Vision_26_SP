#include "publish2nav.hpp"

#include <Eigen/Dense>
#include <chrono>
#include <memory>
#include <thread>

#include "tools/logger.hpp"

namespace io
{

Publish2Nav::Publish2Nav() : Node("auto_aim_target_pos_publisher")
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("auto_aim_target_pos", 10);

  status_publisher_ = this->create_publisher<sp_msgs::msg::NavStatusMsg>("judge_data", 10);

  RCLCPP_INFO(this->get_logger(), "auto_aim_target_pos_publisher node initialized.");
}

Publish2Nav::~Publish2Nav()
{
  RCLCPP_INFO(this->get_logger(), "auto_aim_target_pos_publisher node shutting down.");
}

void Publish2Nav::send_data(const Eigen::Vector4d & target_pos)
{
  // 创建消息
  auto message = std::make_shared<std_msgs::msg::String>();

  // 将 Eigen::Vector3d 数据转换为字符串并存储在消息中
  message->data = std::to_string(target_pos[0]) + "," + std::to_string(target_pos[1]) + "," +
                  std::to_string(target_pos[2]) + "," + std::to_string(target_pos[3]);

  // 发布消息
  publisher_->publish(*message);

  // RCLCPP_INFO(
  //   this->get_logger(), "auto_aim_target_pos_publisher node sent message: '%s'",
  //   message->data.c_str());
}

void Publish2Nav::send_status(uint8_t game_progress,uint16_t stage_remain_time,uint16_t current_hp,uint16_t ally_outpost_hp,float x,float y,float angle,uint8_t state,uint8_t energy_state,uint16_t bullets)
{
    auto message = sp_msgs::msg::NavStatusMsg();
    message.game_progress = game_progress;
    message.stage_remain_time = stage_remain_time;
    message.current_hp = current_hp;
    message.ally_outpost_hp = ally_outpost_hp;
    message.x = x;
    message.y = y;
    message.angle = angle;
    message.state = state;
    message.energy_state = energy_state;
    message.bullets = bullets;
    status_publisher_->publish(message);
}

void Publish2Nav::start()
{
  RCLCPP_INFO(this->get_logger(), "auto_aim_target_pos_publisher node starting to spin...");
  rclcpp::spin(this->shared_from_this());
}

}  // namespace io
