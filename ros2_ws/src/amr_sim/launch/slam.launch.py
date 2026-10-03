import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory('amr_sim')

    # The robot sim + keyboard control, with an RViz view that shows the map
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sim.launch.py')),
        launch_arguments={'rviz_config': os.path.join(pkg, 'rviz', 'slam.rviz')}.items(),
    )

    # slam_toolbox is a lifecycle node in Jazzy; its own launch file configures + activates it
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': os.path.join(pkg, 'config', 'slam_params.yaml'),
            'use_sim_time': 'true',
        }.items(),
    )

    return LaunchDescription([sim, slam])
