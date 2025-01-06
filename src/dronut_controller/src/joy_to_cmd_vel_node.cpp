#include "joy_to_cmd_vel_node.hpp"

JoyToCmdVelNode::JoyToCmdVelNode() 
    : Node("joy_to_cmd_vel"), x_position_(0.0), y_position_(0.0), z_position_(0.0), theta_(0.0), max_velocity_(1.0)
{
    joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&JoyToCmdVelNode::joy_callback, this, std::placeholders::_1));

    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(400), std::bind(&JoyToCmdVelNode::broadcast_transform, this));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    last_transform_ = geometry_msgs::msg::TransformStamped();
}

void JoyToCmdVelNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{   
    if (msg->axes[2] == -1 && msg->axes[5] == -1) {
    //RCLCPP_INFO(this->get_logger(), "Resetting part_9 to world frame");
    reset_to_world_frame();
    }

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

void JoyToCmdVelNode::reset_to_world_frame() {
    x_position_ = 0.0;
    y_position_ = 0.0;
    z_position_ = 0.0;
    theta_ = 0.0;

    geometry_msgs::msg::TransformStamped reset_transform;
    reset_transform.header.stamp = this->get_clock()->now();
    reset_transform.header.frame_id = "world";
    reset_transform.child_frame_id = "part_9";

    reset_transform.transform.translation.x = x_position_;
    reset_transform.transform.translation.y = y_position_;
    reset_transform.transform.translation.z = z_position_;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    reset_transform.transform.rotation.x = q.x();
    reset_transform.transform.rotation.y = q.y();
    reset_transform.transform.rotation.z = q.z();
    reset_transform.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(reset_transform);
}


void JoyToCmdVelNode::broadcast_transform()
{
    double dt = 0.1;

    x_position_ += current_linear_velocity_x_ * dt;
    y_position_ += current_linear_velocity_y_ * dt;
    z_position_ += current_linear_velocity_z_ * dt;
    if (z_position_ < 0.0) {
        z_position_ = 0.0;
    }

    if (z_position_ > 0.0) {
        theta_ += current_angular_velocity_z_ * dt;

        if (theta_ > 3.14159)
            theta_ -= 2 * 3.14159;
        else if (theta_ < -3.14159)
            theta_ += 2 * 3.14159;
    }

    geometry_msgs::msg::TransformStamped current_transform;
    current_transform.header.stamp = this->get_clock()->now();
    current_transform.header.frame_id = "world";
    current_transform.child_frame_id = "part_9";

    current_transform.transform.translation.x = x_position_;
    current_transform.transform.translation.y = y_position_;
    current_transform.transform.translation.z = z_position_;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    current_transform.transform.rotation.x = q.x();
    current_transform.transform.rotation.y = q.y();
    current_transform.transform.rotation.z = q.z();
    current_transform.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(current_transform);

    if (last_transform_.transform.translation.x == current_transform.transform.translation.x &&
        last_transform_.transform.translation.y == current_transform.transform.translation.y &&
        last_transform_.transform.translation.z == current_transform.transform.translation.z &&
        last_transform_.transform.rotation.x == current_transform.transform.rotation.x &&
        last_transform_.transform.rotation.y == current_transform.transform.rotation.y &&
        last_transform_.transform.rotation.z == current_transform.transform.rotation.z &&
        last_transform_.transform.rotation.w == current_transform.transform.rotation.w) 
    {
        return;
    }
    last_transform_ = current_transform;

    std::string gz_command = "gz service -s /world/empty/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 300 --req 'name: \"dronut\", position: {x: " 
                         + std::to_string(x_position_) + ", y: " + std::to_string(y_position_) + ", z: " + std::to_string(z_position_) + "}, orientation: {x: " 
                         + std::to_string(q.x()) + ", y: " + std::to_string(q.y()) + ", z: " + std::to_string(q.z()) + ", w: " + std::to_string(q.w()) + "}' > /dev/null 2>&1";
    int ret_code = system(gz_command.c_str());
    if (ret_code != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to execute command. Return code: %d", ret_code);
    }
}
