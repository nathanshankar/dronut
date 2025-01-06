#include "birotor_drone_controller.hpp"

BirotorDroneController::BirotorDroneController() : Node("birotor_drone_controller"), z_position_(0.0)
{
    joint_limits_ = {
        {"slider_front", {0.015, 0.041}},
        {"slider_back", {-0.019, 0.008}},
        {"slider_left", {0.015, 0.041}},
        {"slider_right", {-0.018, 0.009}},
    };

    joint_states_ = {
        {"slider_front", 0.015 + (0.041 - 0.015) / 2},
        {"slider_back", -0.019 + (0.008 - -0.019) / 2},
        {"slider_left", 0.015 + (0.041 - 0.015) / 2},
        {"slider_right", -0.018 + (0.009 - -0.018) / 2},
    };

    propeller_velocities_ = {
        {"top_prop_continuous", 0.0},
        {"bottom_prop_continuous", 0.0},
    };

    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    joint_trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/drone_controller/joint_trajectory", 10);
    
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&BirotorDroneController::cmd_vel_callback, this, std::placeholders::_1));

    //RCLCPP_INFO(this->get_logger(), "Birotor Drone Controller Node Initialized");
}

void BirotorDroneController::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    double x = msg->linear.x;
    double y = msg->linear.y;
    double z = msg->linear.z;

    if (x > 0.0)
    {
        move_forward();
        //RCLCPP_INFO(this->get_logger(), "Moving forward");
    }
    else if (x < 0.0)
    {
        move_backward();
        //RCLCPP_INFO(this->get_logger(), "Moving backward");
    }

    if (y > 0.0)
    {
        move_left();
        //RCLCPP_INFO(this->get_logger(), "Moving left");
    }
    else if (y < 0.0)
    {
        move_right();
        //RCLCPP_INFO(this->get_logger(), "Moving right");
    }

    if (z > 0.0)
    {
        z_position_ += 0.1;
        set_sliders_max();
        set_propellers_spin(22147483647.0);
        //RCLCPP_INFO(this->get_logger(), "Ascending");
    }
    else if (z < 0.0)
    {
        z_position_ -= 0.1;
        set_sliders_min();
        set_propellers_spin(22147483647.0);
        //RCLCPP_INFO(this->get_logger(), "Descending");
    }

    if (z_position_ <= 0.0)
    {
        set_sliders_mid();
        set_propellers_spin(0.0);
        z_position_ = 0.0;
        //RCLCPP_INFO(this->get_logger(), "Propellers stopped");
    }

    publish_joint_trajectory();
    publish_joint_states();
}

void BirotorDroneController::set_sliders_max()
{
    for (auto &joint : joint_limits_)
    {
        joint_states_[joint.first] = joint.second.upper;
    }
}

void BirotorDroneController::set_sliders_min()
{
    for (auto &joint : joint_limits_)
    {
        joint_states_[joint.first] = joint.second.lower;
    }
}

void BirotorDroneController::set_sliders_mid()
{
    for (auto &joint : joint_limits_)
    {
        joint_states_[joint.first] = joint.second.lower + (joint.second.upper - joint.second.lower) / 2;
    }
}

void BirotorDroneController::move_forward()
{
    set_sliders_mid();
    joint_states_["slider_back"] = joint_limits_["slider_back"].upper;
}

void BirotorDroneController::move_backward()
{
    set_sliders_mid();
    joint_states_["slider_front"] = joint_limits_["slider_front"].upper;
}

void BirotorDroneController::move_left()
{
    set_sliders_mid();
    joint_states_["slider_right"] = joint_limits_["slider_right"].upper;
}

void BirotorDroneController::move_right()
{
    set_sliders_mid();
    joint_states_["slider_left"] = joint_limits_["slider_left"].upper;
}

void BirotorDroneController::set_propellers_spin(double rate)
{
    propeller_velocities_["top_prop_continuous"] = rate;
    propeller_velocities_["bottom_prop_continuous"] = rate;
}

void BirotorDroneController::publish_joint_trajectory()
{
    auto joint_trajectory_msg = trajectory_msgs::msg::JointTrajectory();

    // Populate joint names
    joint_trajectory_msg.joint_names = {
        "bottom_prop_continuous",
        "top_prop_continuous",
        "slider_front",
        "slider_back",
        "slider_left",
        "slider_right"
    };

    // Populate trajectory points
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {
        propeller_velocities_["bottom_prop_continuous"],
        propeller_velocities_["top_prop_continuous"],
        joint_states_["slider_front"],
        joint_states_["slider_back"],
        joint_states_["slider_left"],
        joint_states_["slider_right"]
    };
    point.time_from_start.sec = 0;
    point.time_from_start.nanosec = 5000;

    joint_trajectory_msg.points.push_back(point);

    // Publish the joint trajectory
    joint_trajectory_publisher_->publish(joint_trajectory_msg);
}


void BirotorDroneController::publish_joint_states()
{
    auto joint_state_msg = sensor_msgs::msg::JointState();
    joint_state_msg.header.stamp = this->get_clock()->now();

    for (auto &joint : joint_states_)
    {
        joint_state_msg.name.push_back(joint.first);
        joint_state_msg.position.push_back(joint.second);
    }

    for (auto &propeller : propeller_velocities_)
    {
        joint_state_msg.name.push_back(propeller.first);
        joint_state_msg.position.push_back(
            std::fmod(this->get_clock()->now().nanoseconds() * propeller.second * 1e-9, 2 * M_PI));
    }

    joint_state_msg.velocity.resize(propeller_velocities_.size(), 0.0);
    joint_state_publisher_->publish(joint_state_msg);
}
