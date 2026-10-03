"""Robot drivers + live mapping with SLAM Toolbox. Drive around with teleop, then save the map:
    ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory('amr_bringup')

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'robot.launch.py')),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': os.path.join(pkg, 'config', 'slam_params.yaml'),
            'use_sim_time': 'false',
        }.items(),
    )

    return LaunchDescription([robot, slam])
