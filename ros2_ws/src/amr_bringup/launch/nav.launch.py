"""Robot drivers + SLAM + Nav2: send goals from RViz on a laptop, or run amr_sim's patrol.py."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory('amr_bringup')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'slam.launch.py')),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': os.path.join(pkg, 'config', 'nav2_params.yaml'),
            'use_sim_time': 'false',
        }.items(),
    )

    return LaunchDescription([slam, nav2])
