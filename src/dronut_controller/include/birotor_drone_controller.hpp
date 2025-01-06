#ifndef BIROTOR_DRONE_CONTROLLER_HPP
#define BIROTOR_DRONE_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <cmath>
#include <map>

class BirotorDroneController : public rclcpp::Node
{
public:
    BirotorDroneController();

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void set_sliders_max();
    void set_sliders_min();
    void set_sliders_mid();
    void move_forward();
    void move_backward();
    void move_left();
    void move_right();
    void set_propellers_spin(double rate);
    void publish_joint_trajectory();
    void publish_joint_states();

    struct JointLimit
    {
        double lower;
        double upper;
    };

    double z_position_;
    std::map<std::string, JointLimit> joint_limits_;
    std::map<std::string, double> joint_states_;
    std::map<std::string, double> propeller_velocities_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
};

#endif
