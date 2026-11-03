"""Ackermann geometry and base_node_R2-style odometry for r2_drive.py.

No ROS imports, so it can be unit tested. Angles in radians, positive = left turn.
"Bicycle angle" is the steering angle of one virtual front wheel on the centre line,
the angle the real base_node_R2 integrates.
"""
import math

MIN_SPEED = 1e-3   # below this a turn request has no meaning for a car: steer straight


def bicycle_angle_for(v, w, wheel_base):
    """Steering angle that gives yaw rate w at speed v (the /cmd_vel meaning)."""
    if abs(v) < MIN_SPEED:
        return 0.0
    return math.atan(wheel_base * w / v)


def _wheel_angle(delta, wheel_base, offset):
    """Angle of a front wheel `offset` to the left of the centre line."""
    if delta == 0.0:
        return 0.0
    return math.atan(wheel_base / (wheel_base / math.tan(delta) - offset))


def wheel_angles(delta, wheel_base, kingpin_width, limit):
    """(left, right, delta_used): Ackermann wheel angles, with delta clamped so the
    inner wheel stays within +-limit."""
    max_delta = math.atan(wheel_base / (wheel_base / math.tan(limit) + kingpin_width / 2))
    delta = max(-max_delta, min(max_delta, delta))
    return (_wheel_angle(delta, wheel_base, kingpin_width / 2),
            _wheel_angle(delta, wheel_base, -kingpin_width / 2),
            delta)


def rear_wheel_speeds(v, delta, wheel_base, wheel_separation, wheel_radius):
    """(left, right) rear wheel joint speeds in rad/s; the inner wheel turns slower."""
    k = (wheel_separation / 2) * math.tan(delta) / wheel_base
    return v * (1 - k) / wheel_radius, v * (1 + k) / wheel_radius


def bicycle_angle_from(left, right, kingpin_width, wheel_base):
    """Bicycle angle from the two steering joint angles (inverse of wheel_angles)."""
    if abs(left) < 1e-9 and abs(right) < 1e-9:
        return 0.0
    # cot of the bicycle angle is the mean of the two wheels' cots; work with tan
    # so a straight wheel (cot infinite) never divides by zero
    tl, tr = math.tan(left), math.tan(right)
    return math.atan(2 * tl * tr / (tl + tr)) if tl + tr != 0 else 0.0


def step_toward(current, target, max_rate, dt):
    """Move current toward target by at most max_rate * dt."""
    step = max_rate * dt
    return current + max(-step, min(step, target - current))


class BicycleOdometry:
    """Dead reckoning like base_node_R2.cpp: speed and bicycle angle in, pose out."""

    def __init__(self, wheel_base):
        self.wheel_base = wheel_base
        self.x = self.y = self.yaw = self.w = 0.0
        self.stamp = None

    def update(self, v, delta, stamp):
        """Integrate since the last stamp (seconds). False if nothing was integrated:
        the first call, or a stamp that is not newer than the last one."""
        if self.stamp is not None and stamp <= self.stamp:
            return False
        self.w = v * math.tan(delta) / self.wheel_base
        if self.stamp is None:
            self.stamp = stamp
            return False
        dt = stamp - self.stamp
        self.stamp = stamp
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw += self.w * dt
        return True
