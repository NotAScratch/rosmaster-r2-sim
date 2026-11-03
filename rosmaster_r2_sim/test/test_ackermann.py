"""Unit tests for scripts/ackermann.py (run: python3 -m pytest rosmaster_r2_sim/test)."""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from ackermann import (BicycleOdometry, bicycle_angle_for, bicycle_angle_from,  # noqa: E402
                       rear_wheel_speeds, step_toward, wheel_angles)

L = 0.23529
K = 0.130
SEP = 0.1685
R = 0.0345
LIM = 0.6


def test_straight():
    assert bicycle_angle_for(0.2, 0.0, L) == 0.0
    assert wheel_angles(0.0, L, K, LIM) == (0.0, 0.0, 0.0)
    left, right = rear_wheel_speeds(0.2, 0.0, L, SEP, R)
    assert left == pytest.approx(0.2 / R) and right == pytest.approx(0.2 / R)


def test_phase2_turn_geometry():
    # Phase 2 measured these exact values for a 0.6 m rear-axle radius
    d = bicycle_angle_for(0.2, 0.2 / 0.6, L)
    left, right, used = wheel_angles(d, L, K, LIM)
    assert left == pytest.approx(0.414, abs=1e-3) and right == pytest.approx(0.340, abs=1e-3)
    assert bicycle_angle_from(left, right, K, L) == pytest.approx(used, abs=1e-9)


def test_inner_rear_wheel_slower():
    left, right = rear_wheel_speeds(0.2, bicycle_angle_for(0.2, 0.2 / 0.6, L), L, SEP, R)
    # Rear-axle radius 0.6 m: wheels on radii 0.6 -/+ SEP/2
    assert left * R == pytest.approx(0.2 * (0.6 - SEP / 2) / 0.6)
    assert right * R == pytest.approx(0.2 * (0.6 + SEP / 2) / 0.6)


def test_zero_speed_turn_request():
    assert bicycle_angle_for(0.0, 0.5, L) == 0.0
    assert rear_wheel_speeds(0.0, 0.0, L, SEP, R) == (0.0, 0.0)


def test_reverse_keeps_yaw_sign():
    d = bicycle_angle_for(-0.2, 0.3, L)
    odo = BicycleOdometry(L)
    odo.update(-0.2, d, 0.0)
    odo.update(-0.2, d, 1.0)
    assert odo.w == pytest.approx(0.3, rel=1e-6) and odo.yaw > 0


def test_saturation_inner_wheel():
    left, right, used = wheel_angles(bicycle_angle_for(0.01, 1.0, L), L, K, LIM)
    assert left == pytest.approx(LIM) and 0 < right < LIM and not math.isnan(used)
    left, right, _ = wheel_angles(-1.5, L, K, LIM)
    assert right == pytest.approx(-LIM)


def test_step_toward():
    assert step_toward(0.0, 0.5, 5.0, 0.02) == pytest.approx(0.1)
    assert step_toward(0.45, 0.5, 5.0, 0.02) == pytest.approx(0.5)
    assert step_toward(0.0, -0.5, 5.0, 0.02) == pytest.approx(-0.1)


def test_odometry_matches_base_node():
    odo = BicycleOdometry(L)
    for i in range(101):                       # 10 s at 10 Hz, 0.2 m/s straight
        odo.update(0.2, 0.0, i * 0.1)
    assert (odo.x, odo.y, odo.yaw) == pytest.approx((2.0, 0.0, 0.0), abs=1e-9)


def test_odometry_ignores_nonpositive_dt():
    odo = BicycleOdometry(L)
    odo.update(0.2, 0.0, 1.0)
    odo.update(0.2, 0.0, 2.0)
    assert odo.update(0.2, 0.0, 2.0) is False and odo.update(0.2, 0.0, 1.5) is False
    assert odo.x == pytest.approx(0.2)
