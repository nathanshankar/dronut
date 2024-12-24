import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
import xacro


def generate_launch_description():
    ld=LaunchDescription()
    node_joy_runner = Node(
        package='joy', executable='joy_node', name='joy_node'
    )
    node_move_runner = Node(
        package='joy_test',
        executable='joy_test_node',
        name='joy_test_node',
        output='screen')
    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=False))
    ld.add_action(node_joy_runner)
    ld.add_action(node_move_runner)
    return ld