#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Geometry>

#include <mutex>
#include <thread>

class ROS2Interface
{
public:
    ROS2Interface();
    ~ROS2Interface();

    // 图像
    cv::Mat getLastImage(bool clone = true);
    rclcpp::Time getLastTimestamp();

    // 云台
    Eigen::Quaterniond getGimbalQuaternion();
    bool isGimbalDataValid();

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void updateGimbalPose();

private:
    // ROS
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::thread spin_thread_;

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr tf_timer_;

    // Image
    std::mutex image_mutex_;
    cv::Mat last_image_;
    rclcpp::Time last_timestamp_;

    // Gimbal
    struct GimbalState {
        double roll{0}, pitch{0}, yaw{0};
        Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
        bool valid{false};
    };

    std::mutex gimbal_mutex_;
    GimbalState gimbal_state_;
};
