#ifndef JOY_TO_CMD_VEL_NODE_HPP
#define JOY_TO_CMD_VEL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>

class JoyToCmdVelNode : public rclcpp::Node
{
public:
    JoyToCmdVelNode();
    
private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void broadcast_transform();

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    double x_position_;
    double y_position_;
    double z_position_;
    double theta_;
    double current_linear_velocity_x_{0.0};
    double current_linear_velocity_y_{0.0};
    double current_linear_velocity_z_{0.0};
    double current_angular_velocity_z_{0.0};
    const double max_velocity_;
};

#endif 
