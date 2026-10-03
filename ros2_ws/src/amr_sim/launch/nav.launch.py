import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory('amr_sim')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    # Sim + keyboard control, with an RViz view of the map, costmaps, plan and goal tool
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sim.launch.py')),
        launch_arguments={
            'rviz_config': os.path.join(pkg, 'rviz', 'nav.rviz'),
        }.items(),
    )

    # Live mapping provides the map and the map -> odom transform
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': os.path.join(pkg, 'config', 'slam_params.yaml'),
            'use_sim_time': 'true',
        }.items(),
    )

    # Planner, controller, behaviors, waypoint follower
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': os.path.join(pkg, 'config', 'nav2_params.yaml'),
            'use_sim_time': 'true',
        }.items(),
    )

    return LaunchDescription([sim, slam, nav2])
