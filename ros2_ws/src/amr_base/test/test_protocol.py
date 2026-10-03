import math

import pytest

from amr_base.protocol import (
    BatteryReport, DiffDriveOdometry, EncoderReport, InfoReport,
    parse_line, twist_to_wheels, velocity_command,
)

TPR = 660


def test_parse_reports():
    assert parse_line('E 1200 15 -3\n') == EncoderReport(1200, 15, -3)
    assert parse_line('B 11.84 0') == BatteryReport(11.84, False)
    assert parse_line('I amr-base 1.0 660') == InfoReport('amr-base 1.0', 660)


@pytest.mark.parametrize('line', ['', 'E 1 2', 'E a b c', 'X 1 2 3', 'B volts 1'])
def test_parse_rejects_garbage(line):
    assert parse_line(line) is None


def test_velocity_command_format():
    assert velocity_command(1.5, -0.25) == b'V 1.500 -0.250\n'


def test_twist_to_wheels():
    assert twist_to_wheels(0.5, 0.0, 0.1, 0.36) == pytest.approx((5.0, 5.0))
    left, right = twist_to_wheels(0.0, 1.0, 0.1, 0.36)
    assert left == pytest.approx(-1.8) and right == pytest.approx(1.8)


def test_straight_line():
    odom = DiffDriveOdometry(0.1, 0.36, TPR)
    odom.update(EncoderReport(0, 0, 0))
    assert odom.update(EncoderReport(1000, TPR, TPR))  # one wheel revolution in 1 s
    assert odom.x == pytest.approx(2 * math.pi * 0.1)
    assert odom.y == pytest.approx(0.0)
    assert odom.linear == pytest.approx(2 * math.pi * 0.1)
    assert odom.angular == pytest.approx(0.0)


def test_spin_in_place_full_turn():
    odom = DiffDriveOdometry(0.1, 0.36, TPR)
    odom.update(EncoderReport(0, 0, 0))
    # A full turn in place needs each wheel to travel pi * separation
    ticks = round(math.pi * 0.36 / (2 * math.pi * 0.1) * TPR)
    for step in range(1, 101):
        odom.update(EncoderReport(step * 20, -ticks * step // 100, ticks * step // 100))
    assert odom.x == pytest.approx(0.0, abs=1e-3)
    assert odom.y == pytest.approx(0.0, abs=1e-3)
    assert abs(odom.theta) == pytest.approx(0.0, abs=0.02)


def test_quarter_circle_arc():
    # Drive an arc of radius 1 m through 90 degrees; should end near (1, 1)
    r, sep, wr = 1.0, 0.36, 0.1
    arc = math.pi / 2
    left_dist, right_dist = arc * (r - sep / 2), arc * (r + sep / 2)
    odom = DiffDriveOdometry(wr, sep, TPR)
    odom.update(EncoderReport(0, 0, 0))
    steps = 200
    for i in range(1, steps + 1):
        lt = round(left_dist / (2 * math.pi * wr) * TPR * i / steps)
        rt = round(right_dist / (2 * math.pi * wr) * TPR * i / steps)
        odom.update(EncoderReport(i * 20, lt, rt))
    assert odom.x == pytest.approx(1.0, abs=0.01)
    assert odom.y == pytest.approx(1.0, abs=0.01)
    assert odom.theta == pytest.approx(math.pi / 2, abs=0.01)


def test_mcu_reset_is_ignored():
    odom = DiffDriveOdometry(0.1, 0.36, TPR)
    odom.update(EncoderReport(5000, 1000, 1000))
    assert not odom.update(EncoderReport(10, 0, 0))  # millis went backwards
    assert odom.x == 0.0
