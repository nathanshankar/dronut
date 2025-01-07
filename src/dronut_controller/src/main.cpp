#include <rclcpp/rclcpp.hpp>
#include "joy_to_cmd_vel_node.hpp"
#include "birotor_drone_controller.hpp"
#include "frame_remapper.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto birotor_drone_controller = std::make_shared<BirotorDroneController>();
    auto joy_to_cmd_vel_node = std::make_shared<JoyToCmdVelNode>();
    auto frame_remapper = std::make_shared<FrameRemapper>();

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(birotor_drone_controller);
    exec.add_node(joy_to_cmd_vel_node);
    exec.add_node(frame_remapper);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
