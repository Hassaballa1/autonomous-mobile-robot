import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('amr_sim')
    world = os.path.join(pkg, 'worlds', 'amr_world.sdf')
    rviz_config = LaunchConfiguration('rviz_config')
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value=os.path.join(pkg, 'rviz', 'amr_sim.rviz'))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world}'}.items(),
    )

    # Gazebo <-> ROS 2 topic bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Lidar mount position relative to the chassis (matches amr_world.sdf)
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--z', '0.10',
                   '--frame-id', 'base_link', '--child-frame-id', 'lidar_link'],
        parameters=[{'use_sim_time': True}],
    )

    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0.4', '--z', '0.05', '--pitch', '0.35',
                   '--frame-id', 'base_link', '--child-frame-id', 'camera_link'],
        parameters=[{'use_sim_time': True}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    # Keyboard control in its own terminal window
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        prefix='xterm -title "Keyboard control" -geometry 70x20 -fa Monospace -fs 11 -e',
        parameters=[{'speed': 0.5, 'turn': 1.0}],
        output='screen',
    )

    return LaunchDescription([rviz_config_arg, gazebo, bridge, lidar_tf, camera_tf, rviz, teleop])
