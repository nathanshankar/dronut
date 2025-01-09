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
    sdf_path = os.path.join(get_package_share_directory('dronut_controller'), 'worlds', 'trial.sdf')

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    gz_start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('ros_gz_sim'), '/launch', '/gz_sim.launch.py']),
        launch_arguments={
            'gz_args' : '-r ' + sdf_path,
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

    node_controller_manager = Node(
        package='controller_manager',
        executable='spawner',
        name='controller_manager',
        arguments=['drone_controller', 'joint_state_broadcaster'],
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=  [
                    '/clock'                           + '@rosgraph_msgs/msg/Clock'   + '[' + 'ignition.msgs.Clock',
                    '/world/empty/model/dronut/joint_state' + '@sensor_msgs/msg/JointState' + '[' + 'ignition.msgs.Model',
                    '/camera_info1'                         + '@sensor_msgs/msg/CameraInfo' + '[' + 'ignition.msgs.CameraInfo',
                    '/ircam1'                               + '@sensor_msgs/msg/Image'      + '[' + 'ignition.msgs.Image',
                    '/ircam1/points'                    + '@sensor_msgs/msg/PointCloud2' + '[' + 'ignition.msgs.PointCloudPacked',
                    # '/camera_info2'                         + '@sensor_msgs/msg/CameraInfo' + '[' + 'ignition.msgs.CameraInfo',
                    # '/ircam2'                               + '@sensor_msgs/msg/Image'      + '[' + 'ignition.msgs.Image',
                    # '/ircam2/points'                    + '@sensor_msgs/msg/PointCloud2' + '[' + 'ignition.msgs.PointCloudPacked',
                    ],
        parameters= [{'qos_overrides./dronut_controller.subscriber.reliability': 'reliable'},{'qos_overrides./dronut_controller.subscriber.durability': 'transient_local'}],
        remappings= [
                    ('/world/empty/model/dronut/joint_state', 'joint_states'),
                    ('/camera_info1', '/ircam1/camera_info'),
                    ('/ircam1', '/ircam1/image_raw'),
                    ('/ircam1/points', '/ircam1/points'),
                    # ('/camera_info2', '/ircam2/camera_info'),
                    # ('/ircam2', '/ircam2/image_raw'),
                    # ('/ircam2/points', '/ircam2/points'),
                    ],
        output='screen'
    )

    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=False))
    ld.add_action(gz_start_world)
    ld.add_action(node_spawn_entity)
    ld.add_action(node_ros_gz_bridge)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_joy)
    ld.add_action(node_controller_manager)
    ld.add_action(node_move)
    ld.add_action(node_rviz)
    # ld.add_action(control_node)
    return ld