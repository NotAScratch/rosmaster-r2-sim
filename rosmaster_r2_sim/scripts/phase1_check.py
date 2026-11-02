#!/usr/bin/env python3
"""Phase 1 checks for the R2 model. Needs the sim running with test_actuators:=true.

    ros2 launch rosmaster_r2_sim sim.launch.py headless:=true test_actuators:=true
    ros2 run rosmaster_r2_sim phase1_check.py

1. Settle:   the robot rests on its wheels, level, and does not creep.
2. Straight: equal rear wheel speeds drive it straight ahead.
3. Steer:    the steering joints follow commands and stop at +-0.6 rad.
4. Turn:     steering left and driving forward turns it left (yaw increases).
"""
import math
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

WHEEL_RADIUS = 0.0345
STEER_LIMIT = 0.6


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def roll_pitch_of(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return roll, pitch


class Phase1Check(Node):
    def __init__(self):
        super().__init__('phase1_check', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.odom = None
        self.joints = {}
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_odom, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.pub = {name: self.create_publisher(Float64, f'/test/{name}', 10) for name in (
            'back_left_joint/cmd_vel', 'back_right_joint/cmd_vel',
            'front_left_steer_joint/cmd_pos', 'front_right_steer_joint/cmd_pos')}
        self.results = []

    def on_odom(self, msg):
        self.odom = msg

    def on_joints(self, msg):
        self.joints = dict(zip(msg.name, msg.position))

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def wait(self, seconds):
        """Spin for `seconds` of sim time."""
        end = self.now() + seconds
        while rclpy.ok() and self.now() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def command(self, wheel=0.0, steer=0.0):
        for name, value in (('back_left_joint/cmd_vel', wheel), ('back_right_joint/cmd_vel', wheel),
                            ('front_left_steer_joint/cmd_pos', steer),
                            ('front_right_steer_joint/cmd_pos', steer)):
            self.pub[name].publish(Float64(data=value))

    def hold(self, seconds, wheel=0.0, steer=0.0):
        """Keep sending a command (the bridge may drop the first message) for `seconds` of sim time."""
        end = self.now() + seconds
        while rclpy.ok() and self.now() < end:
            self.command(wheel, steer)
            self.wait(0.1)

    def pose(self):
        p = self.odom.pose.pose
        return p.position.x, p.position.y, p.position.z, yaw_of(p.orientation), roll_pitch_of(p.orientation)

    def check(self, name, ok, detail):
        self.results.append(ok)
        self.get_logger().info(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}")

    def run(self):
        self.get_logger().info('Waiting for /ground_truth/odom and /joint_states ...')
        while rclpy.ok() and (self.odom is None or not self.joints or self.now() == 0.0):
            rclpy.spin_once(self, timeout_sec=0.1)

        # 1. Settle
        self.hold(2.0)
        x0, y0, z0, _, _ = self.pose()
        self.hold(2.0)
        x1, y1, z1, _, (roll, pitch) = self.pose()
        creep = math.hypot(x1 - x0, y1 - y0)
        self.check('settle: rests on the ground', abs(z1) < 0.005, f'base_footprint z = {z1 * 1000:.1f} mm')
        self.check('settle: level', abs(roll) < 0.02 and abs(pitch) < 0.02,
                   f'roll {math.degrees(roll):.2f} deg, pitch {math.degrees(pitch):.2f} deg')
        self.check('settle: no creep', creep < 0.002, f'moved {creep * 1000:.1f} mm in 2 s')

        # 2. Straight
        wheel_speed = 5.0  # rad/s, about 0.17 m/s
        drive_time = 4.0
        x0, y0, _, yaw0, _ = self.pose()
        self.hold(drive_time, wheel=wheel_speed)
        self.hold(1.0)
        x1, y1, _, yaw1, _ = self.pose()
        forward = (x1 - x0) * math.cos(yaw0) + (y1 - y0) * math.sin(yaw0)
        lateral = -(x1 - x0) * math.sin(yaw0) + (y1 - y0) * math.cos(yaw0)
        expected = wheel_speed * WHEEL_RADIUS * drive_time
        yaw_drift = math.degrees(math.remainder(yaw1 - yaw0, math.tau))
        self.check('straight: drives forward', abs(forward - expected) < 0.15 * expected,
                   f'{forward:.3f} m (expected ~{expected:.3f} m)')
        self.check('straight: stays straight', abs(lateral) < 0.02 and abs(yaw_drift) < 2.0,
                   f'lateral {lateral * 1000:.1f} mm, yaw drift {yaw_drift:.2f} deg')

        # 3. Steer
        for target in (0.4, -0.4):
            self.hold(1.5, steer=target)
            left, right = self.joints['front_left_steer_joint'], self.joints['front_right_steer_joint']
            self.check(f'steer: follows {target:+.1f} rad', abs(left - target) < 0.03 and abs(right - target) < 0.03,
                       f'left {left:+.3f}, right {right:+.3f}')
        self.hold(1.5, steer=1.2)
        left, right = self.joints['front_left_steer_joint'], self.joints['front_right_steer_joint']
        self.check('steer: stops at the limit', max(left, right) < STEER_LIMIT + 0.02,
                   f'commanded 1.2 rad, got left {left:+.3f}, right {right:+.3f} (limit {STEER_LIMIT})')

        # 4. Turn left
        self.hold(1.0, steer=0.4)
        _, _, _, yaw0, _ = self.pose()
        self.hold(3.0, wheel=wheel_speed, steer=0.4)
        self.hold(1.0)
        _, _, _, yaw1, _ = self.pose()
        turned = math.degrees(math.remainder(yaw1 - yaw0, math.tau))
        self.check('turn: steering left turns left', turned > 10.0, f'yaw changed {turned:+.1f} deg')

        passed = sum(self.results)
        self.get_logger().info(f'{passed}/{len(self.results)} checks passed')
        return passed == len(self.results)


def main():
    rclpy.init()
    node = Phase1Check()
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
