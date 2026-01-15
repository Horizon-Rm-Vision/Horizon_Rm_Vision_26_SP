#include "ros2_interface/ros2_interface.hpp"

ROS2Interface::ROS2Interface()
{
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);

    node_ = rclcpp::Node::make_shared("ros2_interface_node");

    image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw",
        rclcpp::SensorDataQoS(),
        std::bind(&ROS2Interface::imageCallback, this, std::placeholders::_1)
    );

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    tf_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(20),   // 50Hz
        std::bind(&ROS2Interface::updateGimbalPose, this)
    );

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);

    spin_thread_ = std::thread([this]() {
        executor_->spin();
    });
}

ROS2Interface::~ROS2Interface()
{
    if (executor_) executor_->cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
    rclcpp::shutdown();
}

void ROS2Interface::imageCallback(
    const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");

        std::lock_guard<std::mutex> lock(image_mutex_);
        last_image_ = cv_ptr->image;   // 零拷贝
        last_timestamp_ = msg->header.stamp;
    }
    catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(node_->get_logger(),
                     "cv_bridge error: %s", e.what());
    }
}

cv::Mat ROS2Interface::getLastImage(bool clone)
{
    std::lock_guard<std::mutex> lock(image_mutex_);
    if (last_image_.empty()) return cv::Mat();
    // std::cout << "getLastImage called" << std::endl;
    return clone ? last_image_.clone() : last_image_;
}

rclcpp::Time ROS2Interface::getLastTimestamp()
{
    std::lock_guard<std::mutex> lock(image_mutex_);
    return last_timestamp_;
}

void ROS2Interface::updateGimbalPose()
{
    try {
        auto tf = tf_buffer_->lookupTransform(
            "odom",
            "gimbal_link",
            tf2::TimePointZero,
            std::chrono::milliseconds(5)
        );

        Eigen::Quaterniond q(
            tf.transform.rotation.w,
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z
        );

        Eigen::Vector3d rpy =
            q.toRotationMatrix().eulerAngles(0, 1, 2);

        GimbalState state;
        state.roll  = rpy[0];
        state.pitch = rpy[1];
        state.yaw   = rpy[2];
        state.q     = q;
        state.valid = true;

        std::lock_guard<std::mutex> lock(gimbal_mutex_);
        gimbal_state_ = state;
    }
    catch (...) {
        std::lock_guard<std::mutex> lock(gimbal_mutex_);
        gimbal_state_.valid = false;
    }
}

Eigen::Quaterniond ROS2Interface::getGimbalQuaternion()
{
    std::lock_guard<std::mutex> lock(gimbal_mutex_);
    return gimbal_state_.q;
}

bool ROS2Interface::isGimbalDataValid()
{
    std::lock_guard<std::mutex> lock(gimbal_mutex_);
    return gimbal_state_.valid;
}
