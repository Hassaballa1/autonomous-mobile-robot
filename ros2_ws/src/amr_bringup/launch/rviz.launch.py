"""Run on a laptop on the same network as the robot (same ROS_DOMAIN_ID) to watch the map and scan."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('amr_bringup'), 'rviz', 'robot.rviz')
    return LaunchDescription([
        Node(package='rviz2', executable='rviz2', arguments=['-d', config]),
    ])
