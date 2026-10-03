"""Bring up the physical robot: model, motor controller, lidar, camera and hazard detection.

Velocity commands flow through the hazard guard before reaching the motors:
    teleop / Nav2 -> cmd_vel -> hazard_guard -> cmd_vel_safe -> base_driver
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    base_config = os.path.join(get_package_share_directory('amr_base'), 'config', 'base.yaml')
    perception_config = os.path.join(
        get_package_share_directory('amr_perception'), 'config', 'perception.yaml')

    use_perception = LaunchConfiguration('use_perception')

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('amr_description'), 'launch', 'description.launch.py')),
    )

    base = Node(
        package='amr_base',
        executable='base_driver',
        parameters=[base_config],
        remappings=[('cmd_vel', 'cmd_vel_safe')],
        output='screen',
    )

    lidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        parameters=[{
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': 115200,      # RPLidar A1
            'frame_id': 'lidar_link',
            'inverted': False,
            'angle_compensate': True,
        }],
        output='screen',
    )

    camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        namespace='camera',
        parameters=[{
            'video_device': LaunchConfiguration('camera_device'),
            'image_size': [640, 480],
            'camera_frame_id': 'camera_optical_frame',
        }],
        condition=IfCondition(use_perception),
    )

    detector = Node(
        package='amr_perception',
        executable='yolo_detector',
        parameters=[perception_config],
        remappings=[('image', 'camera/image_raw')],
        condition=IfCondition(use_perception),
        output='screen',
    )

    # Always in the command path; with perception off it simply passes commands through
    guard = Node(
        package='amr_perception',
        executable='hazard_guard',
        parameters=[perception_config],
        remappings=[('cmd_vel_in', 'cmd_vel'), ('cmd_vel_out', 'cmd_vel_safe')],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_perception', default_value='true',
                              description='Run the camera and YOLOv8 hazard detector'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/rplidar'),
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        description, base, lidar, camera, detector, guard,
    ])
