"""ROS 2 node that drives the AMR base through the motor-controller firmware.

Subscribes:  cmd_vel (geometry_msgs/Twist)
Publishes:   odom (nav_msgs/Odometry), tf odom -> base_link,
             joint_states (sensor_msgs/JointState), battery (sensor_msgs/BatteryState)
"""
import math
import threading

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, JointState
import serial
from tf2_ros import TransformBroadcaster

from amr_base.protocol import (
    BatteryReport, DiffDriveOdometry, EncoderReport, ErrorReport, InfoReport,
    parse_line, twist_to_wheels, velocity_command,
)


class BaseDriver(Node):

    def __init__(self):
        super().__init__('base_driver')
        self.port = self.declare_parameter('port', '/dev/amr_base').value
        self.baud = self.declare_parameter('baud', 115200).value
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.075).value
        self.wheel_separation = self.declare_parameter('wheel_separation', 0.30).value
        self.ticks_per_rev = self.declare_parameter('ticks_per_rev', 1200).value
        self.max_wheel_speed = self.declare_parameter('max_wheel_speed', 6.0).value
        self.cmd_timeout = self.declare_parameter('cmd_vel_timeout', 0.5).value
        self.publish_tf = self.declare_parameter('publish_tf', True).value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.left_joint = self.declare_parameter('left_wheel_joint', 'left_wheel_joint').value
        self.right_joint = self.declare_parameter('right_wheel_joint', 'right_wheel_joint').value

        self.odometry = DiffDriveOdometry(
            self.wheel_radius, self.wheel_separation, self.ticks_per_rev)
        self.target = (0.0, 0.0)
        self.last_cmd_time = None
        self.serial = None
        self.serial_lock = threading.Lock()

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.battery_pub = self.create_publisher(BatteryState, 'battery', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, qos_profile_sensor_data)

        # Resend the target at 20 Hz so the firmware watchdog stays fed while we are alive
        self.create_timer(0.05, self.send_command)

        self.running = True
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()

    # ------------------------------------------------------------ commands
    def on_cmd_vel(self, msg):
        left, right = twist_to_wheels(
            msg.linear.x, msg.angular.z, self.wheel_radius, self.wheel_separation)
        # Scale both wheels together so the turning radius is kept when saturating
        peak = max(abs(left), abs(right))
        if peak > self.max_wheel_speed:
            left *= self.max_wheel_speed / peak
            right *= self.max_wheel_speed / peak
        self.target = (left, right)
        self.last_cmd_time = self.get_clock().now()

    def send_command(self):
        if self.last_cmd_time is None:
            return
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        left, right = self.target if age < self.cmd_timeout else (0.0, 0.0)
        self.write(velocity_command(left, right))

    def write(self, data):
        with self.serial_lock:
            if self.serial is None:
                return
            try:
                self.serial.write(data)
            except serial.SerialException as e:
                self.get_logger().warn(f'Serial write failed: {e}')

    # ------------------------------------------------------------ serial reading
    def open_serial(self):
        try:
            port = serial.Serial(self.port, self.baud, timeout=0.2)
        except serial.SerialException as e:
            self.get_logger().warn(f'Cannot open {self.port}: {e}', throttle_duration_sec=5.0)
            return False
        port.reset_input_buffer()
        with self.serial_lock:
            self.serial = port
        self.odometry._last = None  # tick counts may have restarted with the MCU
        self.write(b'?\n')
        self.get_logger().info(f'Connected to motor controller on {self.port}')
        return True

    def read_loop(self):
        while self.running and rclpy.ok():
            if self.serial is None and not self.open_serial():
                threading.Event().wait(1.0)
                continue
            try:
                raw = self.serial.readline()
            except serial.SerialException as e:
                self.get_logger().error(f'Lost motor controller: {e}')
                with self.serial_lock:
                    self.serial.close()
                    self.serial = None
                continue
            if raw:
                self.on_report(parse_line(raw.decode(errors='ignore')))

    def on_report(self, report):
        if isinstance(report, EncoderReport):
            if self.odometry.update(report):
                self.publish_odometry()
        elif isinstance(report, BatteryReport):
            self.publish_battery(report)
        elif isinstance(report, InfoReport):
            self.get_logger().info(
                f'Firmware: {report.firmware}, {report.ticks_per_rev} ticks/rev')
            if report.ticks_per_rev != self.ticks_per_rev:
                self.get_logger().warn(
                    f'ticks_per_rev parameter is {self.ticks_per_rev} but firmware reports '
                    f'{report.ticks_per_rev}; odometry will be scaled wrong')
        elif isinstance(report, ErrorReport):
            self.get_logger().warn(f'Firmware: {report.text}')

    # ------------------------------------------------------------ publishing
    def publish_odometry(self):
        now = self.get_clock().now().to_msg()
        o = self.odometry
        qz, qw = math.sin(o.theta / 2.0), math.cos(o.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = o.x
        odom.pose.pose.position.y = o.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = o.linear
        odom.twist.twist.angular.z = o.angular
        # Wheel odometry drifts in x, y, yaw; z, roll, pitch are fixed for a planar robot
        for i, var in enumerate([0.01, 0.01, 1e6, 1e6, 1e6, 0.03]):
            odom.pose.covariance[i * 7] = var
            odom.twist.covariance[i * 7] = var
        self.odom_pub.publish(odom)

        if self.tf_broadcaster:
            t = TransformStamped()
            t.header = odom.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x = o.x
            t.transform.translation.y = o.y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

        joints = JointState()
        joints.header.stamp = now
        joints.name = [self.left_joint, self.right_joint]
        joints.position = [o.left_angle, o.right_angle]
        self.joint_pub.publish(joints)

    def publish_battery(self, report):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = report.volts
        msg.present = True
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.power_supply_health = (
            BatteryState.POWER_SUPPLY_HEALTH_DEAD if report.low
            else BatteryState.POWER_SUPPLY_HEALTH_GOOD)
        self.battery_pub.publish(msg)
        if report.low:
            self.get_logger().warn(
                f'Battery low ({report.volts:.2f} V): firmware has disabled the motors',
                throttle_duration_sec=10.0)

    def destroy_node(self):
        self.running = False
        self.write(b'S\n')
        super().destroy_node()


def main():
    rclpy.init()
    node = BaseDriver()
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
