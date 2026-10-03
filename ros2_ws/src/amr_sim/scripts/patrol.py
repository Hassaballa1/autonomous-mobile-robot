#!/usr/bin/env python3
"""Drive the robot through a list of waypoints using Nav2.

Usage:
  ros2 run amr_sim patrol.py          # visit each waypoint once
  ros2 run amr_sim patrol.py --loop   # keep patrolling until Ctrl+C
"""
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# (x, y, heading in degrees) in the map frame; the robot starts at (0, 0)
WAYPOINTS = [
    (3.0, -3.0, 90),
    (3.0, 3.0, 180),
    (-3.0, 3.0, -90),
    (-3.0, -3.0, 0),
    (0.0, 0.0, 0),
]


def make_pose(nav, x, y, yaw_deg):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    yaw = math.radians(yaw_deg)
    pose.pose.orientation.z = math.sin(yaw / 2)
    pose.pose.orientation.w = math.cos(yaw / 2)
    return pose


def main():
    loop = '--loop' in sys.argv
    rclpy.init()
    nav = BasicNavigator()
    nav.waitUntilNav2Active(localizer='robot_localization')  # SLAM provides localization, skip AMCL check

    lap = 1
    while rclpy.ok():
        poses = [make_pose(nav, *wp) for wp in WAYPOINTS]
        nav.info(f'Lap {lap}: following {len(poses)} waypoints')
        nav.followWaypoints(poses)

        last = -1
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            if feedback and feedback.current_waypoint != last:
                last = feedback.current_waypoint
                x, y, _ = WAYPOINTS[last]
                nav.info(f'  heading to waypoint {last + 1}/{len(poses)} at ({x}, {y})')

        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            nav.info('All waypoints reached')
        else:
            nav.info(f'Waypoint following ended: {result}')
            break
        if not loop:
            break
        lap += 1

    rclpy.shutdown()


if __name__ == '__main__':
    main()
