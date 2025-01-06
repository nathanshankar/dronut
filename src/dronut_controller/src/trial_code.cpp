#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "controller_manager/controller_manager.hpp"

class DroneControllerNode : public rclcpp::Node
{
public:
  DroneControllerNode()
  : Node("drone_controller_node")
  {
    // Create a publisher for joint states
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // Create a publisher for joint trajectory commands
    joint_command_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/drone_controller/command", 10);

    // Initialize the JointState message
    joint_state_msg_.name = {"bottom_prop_continuous", "top_prop_continuous", "slider_front", "slider_back", "slider_left", "slider_right"};
    joint_state_msg_.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // Initial positions (could be set dynamically)
    joint_state_msg_.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // Assuming zero velocity
    joint_state_msg_.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // Assuming zero effort

    // Create a timer to publish joint states at the given update rate
    state_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),  // 100Hz update rate
      std::bind(&DroneControllerNode::publish_joint_state, this)
    );

    // Create a timer to send joint trajectory commands (example: command a new position for each joint)
    command_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),  // 10Hz update rate for sending commands
      std::bind(&DroneControllerNode::send_joint_commands, this)
    );
  }

private:
  void publish_joint_state()
  {
    joint_state_msg_.header.stamp = this->get_clock()->now();
    joint_state_pub_->publish(joint_state_msg_);
  }

  void send_joint_commands()
  {
    // Create a trajectory message for the drone controller
    trajectory_msgs::msg::JointTrajectory joint_trajectory_msg;
    joint_trajectory_msg.header.stamp = this->get_clock()->now();
    joint_trajectory_msg.joint_names = joint_state_msg_.name;

    // Example: setting a trajectory point (you can update this with dynamic values)
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {0.0, 0.0, 0.041, -0.019, 0.041, -0.018};  // Example desired positions for the joints
    point.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // Assuming zero velocity for this example
    point.time_from_start = rclcpp::Duration::from_seconds(1.0);  // Set duration for this trajectory point
    
    // Add the point to the trajectory message
    joint_trajectory_msg.points.push_back(point);

    // Publish the joint trajectory command
    joint_command_pub_->publish(joint_trajectory_msg);
  }

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_command_pub_;

  // Joint state message
  sensor_msgs::msg::JointState joint_state_msg_;

  // Timers for publishing joint states and sending commands
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr command_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneControllerNode>());
  rclcpp::shutdown();
  return 0;
}
