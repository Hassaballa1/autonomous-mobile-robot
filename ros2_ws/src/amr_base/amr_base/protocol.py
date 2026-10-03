"""Serial protocol and diff-drive kinematics, kept free of ROS so it can be unit tested.

Pi -> MCU:   V <left rad/s> <right rad/s> | S | R | P <kp> <ki> <kd> <kf> | ?
MCU -> Pi:   E <ms> <left ticks> <right ticks>
             B <volts> <low flag>
             I <firmware id...> <ticks per rev>
             ! <error text>
"""
from dataclasses import dataclass
import math


@dataclass
class EncoderReport:
    stamp_ms: int
    left_ticks: int
    right_ticks: int


@dataclass
class BatteryReport:
    volts: float
    low: bool


@dataclass
class InfoReport:
    firmware: str
    ticks_per_rev: int


@dataclass
class ErrorReport:
    text: str


def parse_line(line):
    """Parse one line from the MCU. Returns a report object, or None if malformed."""
    parts = line.strip().split()
    if not parts:
        return None
    kind, args = parts[0], parts[1:]
    try:
        if kind == 'E' and len(args) == 3:
            return EncoderReport(int(args[0]), int(args[1]), int(args[2]))
        if kind == 'B' and len(args) == 2:
            return BatteryReport(float(args[0]), args[1] == '1')
        if kind == 'I' and len(args) >= 2:
            return InfoReport(' '.join(args[:-1]), int(args[-1]))
        if kind == '!':
            return ErrorReport(' '.join(args))
    except ValueError:
        pass
    return None


def velocity_command(left, right):
    return f'V {left:.3f} {right:.3f}\n'.encode()


def twist_to_wheels(linear, angular, wheel_radius, wheel_separation):
    """Body velocity (m/s, rad/s) -> wheel angular velocities (rad/s)."""
    left = (linear - angular * wheel_separation / 2.0) / wheel_radius
    right = (linear + angular * wheel_separation / 2.0) / wheel_radius
    return left, right


class DiffDriveOdometry:
    """Integrates encoder ticks into a planar pose and body velocity."""

    def __init__(self, wheel_radius, wheel_separation, ticks_per_rev):
        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_separation
        self.ticks_per_rev = ticks_per_rev
        self.reset()

    def reset(self):
        self.x = self.y = self.theta = 0.0
        self.linear = self.angular = 0.0
        self.left_angle = self.right_angle = 0.0
        self._last = None

    def update(self, report):
        """Feed an EncoderReport. Returns True once the pose has been updated."""
        if self._last is None:
            self._last = report
            return False

        dt = (report.stamp_ms - self._last.stamp_ms) / 1000.0
        if dt <= 0:
            # MCU reset or millis() wrap: start over from this report
            self._last = report
            return False

        rad_per_tick = 2.0 * math.pi / self.ticks_per_rev
        d_left = (report.left_ticks - self._last.left_ticks) * rad_per_tick
        d_right = (report.right_ticks - self._last.right_ticks) * rad_per_tick
        self._last = report

        self.left_angle += d_left
        self.right_angle += d_right

        ds = (d_left + d_right) / 2.0 * self.wheel_radius
        dtheta = (d_right - d_left) * self.wheel_radius / self.wheel_separation

        # Midpoint integration is exact for constant-curvature motion over small steps
        heading = self.theta + dtheta / 2.0
        self.x += ds * math.cos(heading)
        self.y += ds * math.sin(heading)
        self.theta = math.atan2(math.sin(self.theta + dtheta), math.cos(self.theta + dtheta))

        self.linear = ds / dt
        self.angular = dtheta / dt
        return True
