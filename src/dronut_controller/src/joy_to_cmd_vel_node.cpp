#include "joy_to_cmd_vel_node.hpp"

JoyToCmdVelNode::JoyToCmdVelNode() 
    : Node("joy_to_cmd_vel"), x_position_(0.0), y_position_(0.0), z_position_(0.0), theta_(0.0), max_velocity_(1.0)
{
    joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&JoyToCmdVelNode::joy_callback, this, std::placeholders::_1));

    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&JoyToCmdVelNode::broadcast_transform, this));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
}

void JoyToCmdVelNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    double linear_velocity_x = max_velocity_ * msg->axes[4];
    double linear_velocity_y = max_velocity_ * msg->axes[3];
    double linear_velocity_z = max_velocity_ * msg->axes[1];
    double angular_velocity_z = max_velocity_ * msg->axes[0];

    if (z_position_ <= 0.0)
    {
        angular_velocity_z = 0.0;
        linear_velocity_x = 0.0;
        linear_velocity_y = 0.0;
    }

    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = linear_velocity_x;
    twist_msg.linear.y = linear_velocity_y;
    twist_msg.linear.z = linear_velocity_z;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = angular_velocity_z;
    velocity_publisher_->publish(twist_msg);

    current_linear_velocity_x_ = linear_velocity_x;
    current_linear_velocity_y_ = linear_velocity_y;
    current_linear_velocity_z_ = linear_velocity_z;
    current_angular_velocity_z_ = angular_velocity_z;
}

void JoyToCmdVelNode::broadcast_transform()
{
    double dt = 0.1;

    x_position_ += current_linear_velocity_x_ * dt;
    y_position_ += current_linear_velocity_y_ * dt;
    z_position_ += current_linear_velocity_z_ * dt;
    
    if (z_position_ < 0.0)
    {
        z_position_ = 0.0;
    }

    if (z_position_ > 0.0)
    {
        theta_ += current_angular_velocity_z_ * dt;

        if (theta_ > 3.14159)
            theta_ -= 2 * 3.14159;
        else if (theta_ < -3.14159)
            theta_ += 2 * 3.14159;
    }

    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->get_clock()->now();
    transformStamped.header.frame_id = "world_frame";
    transformStamped.child_frame_id = "base_link";

    transformStamped.transform.translation.x = x_position_;
    transformStamped.transform.translation.y = y_position_;
    transformStamped.transform.translation.z = z_position_;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(transformStamped);
}
