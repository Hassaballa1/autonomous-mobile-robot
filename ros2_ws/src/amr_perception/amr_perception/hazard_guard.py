"""Velocity gate that stops the robot driving towards stairs and drop-offs.

Sits between every velocity source (Nav2, teleop) and the base driver:

    cmd_vel ──> hazard_guard ──> cmd_vel_safe ──> base_driver

Subscribes:  detections (vision_msgs/Detection2DArray), cmd_vel_in (geometry_msgs/Twist)
Publishes:   cmd_vel_out (geometry_msgs/Twist), hazard (std_msgs/Bool)
"""
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray

from amr_perception.hazard_logic import find_hazards, gate_velocity


class HazardGuard(Node):

    def __init__(self):
        super().__init__('hazard_guard')
        self.hazard_classes = set(self.declare_parameter(
            'hazard_classes', ['Stairs', 'Drop']).value)
        self.min_score = self.declare_parameter('min_score', 0.5).value
        self.image_height = self.declare_parameter('image_height', 480).value
        self.near_fraction = self.declare_parameter('near_fraction', 0.55).value
        self.hold_time = self.declare_parameter('hold_time', 1.5).value

        self.last_hazard_time = None
        self.last_hazard_names = ''
        self.was_active = False

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_out', 10)
        self.hazard_pub = self.create_publisher(Bool, 'hazard', 10)
        self.create_subscription(Detection2DArray, 'detections', self.on_detections, 10)
        self.create_subscription(Twist, 'cmd_vel_in', self.on_cmd_vel, 10)
        # Publish the state on a timer so it clears even if detections stop arriving
        self.create_timer(0.2, self.publish_state)

    def hazard_active(self):
        if self.last_hazard_time is None:
            return False
        age = (self.get_clock().now() - self.last_hazard_time).nanoseconds / 1e9
        return age < self.hold_time

    def on_detections(self, msg):
        detections = [
            (r.hypothesis.class_id, r.hypothesis.score,
             (d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.size_x, d.bbox.size_y))
            for d in msg.detections for r in d.results
        ]
        hazards = find_hazards(detections, self.hazard_classes, self.min_score,
                               self.image_height, self.near_fraction)
        if hazards:
            self.last_hazard_time = self.get_clock().now()
            self.last_hazard_names = ', '.join(sorted({h[0] for h in hazards}))
        self.publish_state()

    def publish_state(self):
        active = self.hazard_active()
        if active != self.was_active:
            if active:
                self.get_logger().warn(
                    f'Hazard ahead ({self.last_hazard_names}): forward motion blocked')
            else:
                self.get_logger().info('Path clear')
            self.was_active = active
        self.hazard_pub.publish(Bool(data=active))

    def on_cmd_vel(self, msg):
        linear, angular = gate_velocity(msg.linear.x, msg.angular.z, self.hazard_active())
        out = Twist()
        out.linear.x = linear
        out.angular.z = angular
        self.cmd_pub.publish(out)


def main():
    rclpy.init()
    node = HazardGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
