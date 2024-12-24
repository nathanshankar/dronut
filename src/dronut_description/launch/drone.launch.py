import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
import xacro


def generate_launch_description():
    ld = LaunchDescription()
    

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'dronut_description'
    file_subpath = 'urdf/x1.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )
    node_joy = Node(
        package='joy', executable='joy_node', name='joy_node'
    )


    # joint state publisher (GUI) node
    node_joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    teleop_twist_keyboard_node = Node(
            package='teleop_twist_keyboard', executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard_node', 
            output='screen',
            prefix = 'xterm -e',
            )

    node_move = Node(
        package='dronut_description',
        executable='move',
        name='move',
        output='screen')

    # Rviz node
    node_rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory(pkg_name), 'rviz', 'x1_view.rviz')]
    )


    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=False))
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_joy)
    #ld.add_action(teleop_twist_keyboard_node)
    #ld.add_action(node_joint_state_publisher)
    ld.add_action(node_move)
    ld.add_action(node_rviz)
    return ld