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
    JoyToCmdVelNode() 
        : Node("joy_to_cmd_vel"), x_position_(0.0), y_position_(0.0), z_position_(0.0), theta_(0.0), max_velocity_(1.0)
    {
        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyToCmdVelNode::joy_callback, this, std::placeholders::_1));

        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&JoyToCmdVelNode::broadcast_transform, this));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Reverse the joystick axes by negating the input values
        double linear_velocity_x = -max_velocity_ * msg->axes[4];
        double linear_velocity_y = -max_velocity_ * msg->axes[3];
        double linear_velocity_z = max_velocity_ * msg->axes[1];
        double angular_velocity_z = max_velocity_ * msg->axes[0];

        // Stop movement if z_position_ <= 0.0
        if (z_position_ <= 0.0)
        {
            angular_velocity_z = 0.0;
            linear_velocity_x = 0.0;
            linear_velocity_y = 0.0;
        }

        // Publish the reversed twist message
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = linear_velocity_x;
        twist_msg.linear.y = linear_velocity_y;
        twist_msg.linear.z = linear_velocity_z;
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = angular_velocity_z;
        velocity_publisher_->publish(twist_msg);

        // Update current velocities for transform calculations
        current_linear_velocity_x_ = linear_velocity_x;
        current_linear_velocity_y_ = linear_velocity_y;
        current_linear_velocity_z_ = linear_velocity_z;
        current_angular_velocity_z_ = angular_velocity_z;
    }


    void broadcast_transform()
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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToCmdVelNode>());
    rclcpp::shutdown();
    return 0;
}
