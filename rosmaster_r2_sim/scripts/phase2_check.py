#!/usr/bin/env python3
"""Phase 2 checks for the R2 Ackermann drive. Needs the sim running with its default drive.

    ros2 launch rosmaster_r2_sim sim.launch.py headless:=true
    ros2 run rosmaster_r2_sim phase2_check.py

Commands go to /cmd_vel exactly as teleop or Nav2 would send them. The robot's motion is
measured with the ground-truth pose, and compared with the robot's own /odom_raw.
"""
import math
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

WHEEL_BASE = 0.23529
KINGPIN_WIDTH = 0.130
REAR_AXLE_X = -0.11607   # rear axle position in base_footprint
STEER_LIMIT = 0.6
MAX_SPEED = 1.0


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def fit_circle(points):
    """Least-squares circle through (x, y) points. Returns the radius."""
    p = np.asarray(points)
    a = np.column_stack([2 * p[:, 0], 2 * p[:, 1], np.ones(len(p))])
    b = (p ** 2).sum(axis=1)
    cx, cy, c = np.linalg.lstsq(a, b, rcond=None)[0]
    return math.sqrt(c + cx * cx + cy * cy)


class Phase2Check(Node):
    def __init__(self):
        super().__init__('phase2_check', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.truth = None
        self.odom = None
        self.joints = {}
        self.odom_tf_seen = False
        self.create_subscription(Odometry, '/ground_truth/odom', lambda m: setattr(self, 'truth', m), 10)
        self.create_subscription(Odometry, '/odom_raw', lambda m: setattr(self, 'odom', m), 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 10)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.results = []

    def on_joints(self, msg):
        self.joints = dict(zip(msg.name, msg.position))

    def on_tf(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == 'odom' and t.child_frame_id == 'base_footprint':
                self.odom_tf_seen = True

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def drive(self, seconds, v=0.0, w=0.0, on_tick=None):
        """Send /cmd_vel for `seconds` of sim time, calling on_tick() every 0.05 s."""
        end = self.now() + seconds
        msg = Twist()
        msg.linear.x, msg.angular.z = v, w
        last_pub = 0.0
        while rclpy.ok() and self.now() < end:
            if self.now() - last_pub >= 0.1:
                self.cmd.publish(msg)
                last_pub = self.now()
            rclpy.spin_once(self, timeout_sec=0.05)
            if on_tick:
                on_tick()

    def stop(self, seconds=1.5):
        self.drive(seconds)

    def pose(self, msg):
        p = msg.pose.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)

    def speed(self):
        return self.truth.twist.twist.linear.x

    def check(self, name, ok, detail):
        self.results.append(ok)
        self.get_logger().info(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}")

    def steering(self):
        return self.joints['front_left_steer_joint'], self.joints['front_right_steer_joint']

    def settle_steering(self, v, w, timeout=10.0):
        """Keep driving until the steering stops moving. Returns the time it took.
        Gazebo Fortress's AckermannSteering turns the wheels slowly (fixed gain, ~1.2 s time
        constant), so a turn is only steady after several seconds."""
        start = self.now()
        last = self.steering()
        while rclpy.ok() and self.now() - start < timeout:
            self.drive(0.5, v, w)
            cur = self.steering()
            if max(abs(cur[0] - last[0]), abs(cur[1] - last[1])) < 0.002:
                break
            last = cur
        return self.now() - start

    def turn_test(self, v, w):
        """Drive a steady turn; return the measured radius of base_footprint's path and the yaw rate."""
        self.settle_steering(v, w)
        points, t0, yaw0 = [], self.now(), self.pose(self.truth)[2]
        total_yaw = 0.0
        last_yaw = yaw0

        def sample():
            nonlocal total_yaw, last_yaw
            x, y, yaw = self.pose(self.truth)
            points.append((x, y))
            total_yaw += math.remainder(yaw - last_yaw, math.tau)
            last_yaw = yaw

        self.drive(4.0, v, w, on_tick=sample)
        radius = fit_circle(points)
        yaw_rate = total_yaw / (self.now() - t0)
        left, right = self.joints['front_left_steer_joint'], self.joints['front_right_steer_joint']
        self.stop()
        return radius, yaw_rate, left, right

    def run(self):
        self.get_logger().info('Waiting for /ground_truth/odom, /odom_raw and /joint_states ...')
        while rclpy.ok() and (self.truth is None or self.odom is None or not self.joints or self.now() == 0.0):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.stop(1.0)

        # Frames and TF
        self.check('odom_raw: frames', self.odom.header.frame_id == 'odom' and self.odom.child_frame_id == 'base_footprint',
                   f'{self.odom.header.frame_id} -> {self.odom.child_frame_id}')
        self.odom_tf_seen = False
        self.stop(1.0)
        self.check('odom_raw: no odom TF by default', not self.odom_tf_seen,
                   'odom -> base_footprint ' + ('was published (should be left to the EKF)' if self.odom_tf_seen else 'not published'))

        # Odometry is judged like a real calibration: path length and total turning,
        # each summed separately for /odom_raw and the ground truth.
        sums = {'odom': [0.0, 0.0], 'truth': [0.0, 0.0]}   # [metres travelled, radians turned]
        last = {'odom': self.pose(self.odom), 'truth': self.pose(self.truth)}

        def track():
            for key, msg in (('odom', self.odom), ('truth', self.truth)):
                cur, prev = self.pose(msg), last[key]
                sums[key][0] += math.hypot(cur[0] - prev[0], cur[1] - prev[1])
                sums[key][1] += abs(math.remainder(cur[2] - prev[2], math.tau))
                last[key] = cur

        # Straight at 0.2 m/s
        x0, y0, yaw0 = self.pose(self.truth)
        self.drive(1.0, 0.2, on_tick=track)
        speeds = []
        self.drive(2.0, 0.2, on_tick=lambda: (track(), speeds.append(self.speed())))
        self.stop()
        track()
        x1, y1, yaw1 = self.pose(self.truth)
        lateral = -(x1 - x0) * math.sin(yaw0) + (y1 - y0) * math.cos(yaw0)
        mean_speed = sum(speeds) / len(speeds)
        self.check('straight: speed', abs(mean_speed - 0.2) < 0.01, f'{mean_speed:.3f} m/s (commanded 0.200)')
        self.check('straight: stays straight', abs(lateral) < 0.01 and abs(math.degrees(yaw1 - yaw0)) < 1.0,
                   f'lateral {lateral * 1000:.1f} mm, yaw {math.degrees(yaw1 - yaw0):+.2f} deg')

        # Stop
        self.check('stop: zero command stops it', abs(self.speed()) < 0.01, f'{self.speed():.4f} m/s after 1.5 s')

        # Steering speed: not pass/fail, because Fortress's plugin has no setting for it.
        # Recorded so the gap with the real servo stays visible (see docs/research/02-phase2-drive.md).
        v, r = 0.2, 0.6
        inner = math.atan(WHEEL_BASE / (r - KINGPIN_WIDTH / 2))
        start = self.now()
        reached = None
        while rclpy.ok() and self.now() - start < 10.0:
            self.drive(0.05, v, v / r, on_tick=track)
            if self.steering()[0] >= 0.9 * inner:
                reached = self.now() - start
                break
        self.stop()
        track()
        self.get_logger().info(
            f'INFO  steering speed: 0 -> 90% of {inner:.3f} rad took '
            + (f'{reached:.2f} s' if reached else 'over 10 s') + ' (a hobby servo takes ~0.2 s)')

        # Steady left turn, rear-axle radius 0.6 m
        radius, yaw_rate, left, right = self.turn_test(v, v / r)
        track()
        expected_radius = math.hypot(r, REAR_AXLE_X)   # base_footprint is ahead of the rear axle
        inner = math.atan(WHEEL_BASE / (r - KINGPIN_WIDTH / 2))
        outer = math.atan(WHEEL_BASE / (r + KINGPIN_WIDTH / 2))
        self.check('turn: radius', abs(radius - expected_radius) < 0.05 * expected_radius,
                   f'{radius:.3f} m (expected {expected_radius:.3f} m)')
        self.check('turn: yaw rate follows angular.z', abs(yaw_rate - v / r) < 0.05 * v / r,
                   f'{yaw_rate:.3f} rad/s (commanded {v / r:.3f})')
        self.check('turn: Ackermann wheel angles', abs(left - inner) < 0.02 and abs(right - outer) < 0.02,
                   f'inner (left) {left:.3f} vs {inner:.3f}, outer (right) {right:.3f} vs {outer:.3f} rad')

        # Right turn mirrors the left one
        radius_r, yaw_rate_r, _, _ = self.turn_test(v, -v / r)
        track()
        self.check('turn: right mirrors left', abs(radius_r - radius) < 0.02 and abs(yaw_rate_r + yaw_rate) < 0.02,
                   f'radius {radius_r:.3f} m, yaw rate {yaw_rate_r:+.3f} rad/s')

        # Asking for a tighter turn than the steering allows
        radius_min, yaw_rate_min, left, right = self.turn_test(0.2, 1.0)
        track()
        tightest = math.hypot(WHEEL_BASE / math.tan(STEER_LIMIT) + KINGPIN_WIDTH / 2, REAR_AXLE_X)
        self.check('limit: steering stays within +-0.6 rad', max(abs(left), abs(right)) <= STEER_LIMIT + 0.01,
                   f'left {left:.3f}, right {right:.3f} rad')
        self.check('limit: tightest turn', radius_min > 0.3,
                   f'asked for 0.2 m, got {radius_min:.3f} m (inner wheel at the limit gives ~{tightest:.3f} m)')

        # Reverse
        x0, y0, yaw0 = self.pose(self.truth)
        self.drive(2.0, -0.2, on_tick=track)
        self.stop()
        track()
        x1, y1, _ = self.pose(self.truth)
        backward = (x1 - x0) * math.cos(yaw0) + (y1 - y0) * math.sin(yaw0)
        self.check('reverse: drives backwards', backward < -0.25, f'{backward:+.3f} m')

        # Speed limit
        speeds = []
        self.drive(2.0, 3.0, on_tick=lambda: (track(), speeds.append(self.speed())))
        self.stop(2.0)
        track()
        top = max(speeds)
        self.check('limit: top speed', top <= MAX_SPEED + 0.05, f'commanded 3.0 m/s, reached {top:.3f} m/s (limit {MAX_SPEED})')

        # Wheel odometry against ground truth over the whole run
        (odom_dist, odom_turn), (truth_dist, truth_turn) = sums['odom'], sums['truth']
        dist_error = (odom_dist - truth_dist) / truth_dist
        turn_error = (odom_turn - truth_turn) / truth_turn
        self.check('odom_raw: distance', abs(dist_error) < 0.03,
                   f'{odom_dist:.3f} m vs {truth_dist:.3f} m true ({100 * dist_error:+.1f}%)')
        self.check('odom_raw: turning', abs(turn_error) < 0.05,
                   f'{math.degrees(odom_turn):.1f} deg vs {math.degrees(truth_turn):.1f} deg true ({100 * turn_error:+.1f}%)')

        passed = sum(self.results)
        self.get_logger().info(f'{passed}/{len(self.results)} checks passed')
        return passed == len(self.results)


def main():
    rclpy.init()
    node = Phase2Check()
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
