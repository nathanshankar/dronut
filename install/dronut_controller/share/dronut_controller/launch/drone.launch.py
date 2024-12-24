import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
import xacro


def generate_launch_description():
    ld = LaunchDescription()
    pkg_name = 'dronut_w_cam'
    file_subpath = 'urdf/x1.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    gz_start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('ros_gz_sim'), '/launch', '/gz_sim.launch.py']),
        launch_arguments={
            'gz_args' : '-r ' + 'empty.sdf'
            }.items(),
    )

    node_spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', '/robot_description',
                                   '-z', '0.5'],
                        output='screen')


    # robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )
    node_joy = Node(
        package='joy', executable='joy_node', name='joy_node'
    )

    node_move = Node(
        package='dronut_controller',
        executable='drone_control_node',
        name='drone_control_node',
        output='screen')

    # Rviz node
    node_rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory(pkg_name), 'rviz', 'x1_view.rviz')]
    )

    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=  [
                    # ROS -> Ignition Gazebo bridge
                    '/clock'                           + '@rosgraph_msgs/msg/Clock'   + '[' + 'ignition.msgs.Clock',
                    '/model/dronut/tf'       + '@tf2_msgs/msg/TFMessage' + '[' + 'ignition.msgs.Pose_V',
                    ],
        parameters= [{'qos_overrides./leo_v1.subscriber.reliability': 'reliable'},{'qos_overrides./leo_v1.subscriber.durability': 'transient_local'}],
        remappings= [
                    # Remap Ignition Gazebo tf topic to ROS tf topic
                    ('/model/dronut/tf',       '/tf'     ),
                    # If required, add reverse remapping for other topics here
                    ],
        output='screen'
    )



    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=False))
    ld.add_action(gz_start_world)
    ld.add_action(node_spawn_entity)

    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_joy)
    ld.add_action(node_move)
    ld.add_action(node_rviz)
    return ld