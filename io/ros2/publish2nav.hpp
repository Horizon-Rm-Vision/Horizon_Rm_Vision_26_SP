#ifndef IO__PBLISH2NAV_HPP
#define IO__PBLISH2NAV_HPP

#include <Eigen/Dense>  // For Eigen::Vector3d
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sp_msgs/msg/nav_status_msg.hpp"

namespace io
{
class Publish2Nav : public rclcpp::Node
{
public:
  Publish2Nav();

  ~Publish2Nav();

  void start();

  void send_data(const Eigen::Vector4d & data);

  void send_status(uint8_t game_progress,uint16_t stage_remain_time,uint16_t current_hp,uint16_t ally_outpost_hp,uint8_t state,uint8_t energy_state,uint16_t bullets);

private:
  // ROS2 发布者
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  rclcpp::Publisher<sp_msgs::msg::NavStatusMsg>::SharedPtr status_publisher_;
};

}  // namespace io

#endif  // Publish2Nav_HPP_
