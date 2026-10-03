import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('amr_sim')
    world = os.path.join(pkg, 'worlds', 'amr_world.sdf')
    perception_config = os.path.join(
        get_package_share_directory('amr_perception'), 'config', 'perception.yaml')
    rviz_config = LaunchConfiguration('rviz_config')
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value=os.path.join(pkg, 'rviz', 'amr_sim.rviz'))
    teleop_arg = DeclareLaunchArgument(
        'teleop', default_value='true', description='Open a keyboard teleop window')
    perception = LaunchConfiguration('perception')
    perception_arg = DeclareLaunchArgument(
        'perception', default_value='false',
        description='Run the YOLOv8 stairs detector on the simulated camera')

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
            '/world/amr_world/model/amr/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        # The wheels only obey commands that have passed the hazard guard, as on the real robot
        remappings=[('/cmd_vel', '/cmd_vel_safe'),
                    ('/world/amr_world/model/amr/joint_state', '/joint_states')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Always in the command path; without the detector it simply passes commands through
    guard = Node(
        package='amr_perception',
        executable='hazard_guard',
        parameters=[perception_config, {'use_sim_time': True}],
        remappings=[('cmd_vel_in', 'cmd_vel'), ('cmd_vel_out', 'cmd_vel_safe')],
        output='screen',
    )

    detector = Node(
        package='amr_perception',
        executable='yolo_detector',
        parameters=[perception_config, {'use_sim_time': True}],
        remappings=[('image', 'camera/image_raw')],
        condition=IfCondition(perception),
        output='screen',
    )

    # Same URDF as the real robot: fixed sensor frames plus wheel joints from Gazebo
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('amr_description'), 'launch', 'description.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
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
        parameters=[{'speed': 0.3, 'turn': 0.8}],
        condition=IfCondition(LaunchConfiguration('teleop')),
        output='screen',
    )

    return LaunchDescription([
        rviz_config_arg, teleop_arg, perception_arg, gazebo, bridge, guard, detector,
        description, rviz, teleop,
    ])
