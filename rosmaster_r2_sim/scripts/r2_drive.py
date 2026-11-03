#!/usr/bin/env python3
"""Drive for the simulated R2: the stand-in for the real Ackman_driver_R2 + motor board + base_node_R2.

    /cmd_vel (linear.x m/s, angular.z rad/s)
        -> steering angles (rate-limited like a servo) and rear wheel speeds, to Gazebo's joint controllers
    /joint_states (rear wheel speeds, steering angles)
        -> /vel_raw (speed, steering angle in degrees, like the real driver)
        -> /odom_raw (integrated like base_node_R2, same frames and covariances)
        -> odom -> base_footprint on /tf, only with publish_odom_tf (the EKF owns it otherwise)

Like the real driver, it holds the last /cmd_vel: there is no command timeout.
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster

from ackermann import (BicycleOdometry, bicycle_angle_for, bicycle_angle_from,
                       rear_wheel_speeds, step_toward, wheel_angles)

STEER_JOINTS = ('front_left_steer_joint', 'front_right_steer_joint')
WHEEL_JOINTS = ('back_left_joint', 'back_right_joint')

# base_node_R2.cpp's covariances, so the real EKF weighs the simulated odometry the same way
POSE_COVARIANCE = {0: 0.001, 7: 0.001, 35: 0.001}
TWIST_COVARIANCE = {0: 0.0001, 7: 0.0001, 35: 0.0001}


class R2Drive(Node):
    def __init__(self):
        super().__init__('r2_drive')
        p = {name: self.declare_parameter(name, default).value for name, default in (
            ('wheel_base', 0.23529), ('wheel_separation', 0.1685), ('kingpin_width', 0.130),
            ('wheel_radius', 0.0345), ('steer_limit', 0.6), ('max_speed', 1.0), ('max_accel', 1.0),
            ('steer_rate', 5.0), ('control_rate', 50.0), ('odom_rate', 10.0),
            ('publish_odom_tf', False))}
        self.p = p
        self.cmd = Twist()
        self.speed = 0.0      # commanded rear-axle speed, after the acceleration limit
        self.delta = 0.0      # commanded bicycle angle, after the steering rate limit
        self.odometry = BicycleOdometry(p['wheel_base'])
        self.measured_v = 0.0
        self.measured_delta = 0.0
        self.stamp = None     # latest /joint_states stamp

        self.joint_cmd = {j: self.create_publisher(Float64, f'/r2_drive/{j}/cmd_pos', 10) for j in STEER_JOINTS}
        self.joint_cmd.update({j: self.create_publisher(Float64, f'/r2_drive/{j}/cmd_vel', 10) for j in WHEEL_JOINTS})
        self.odom_pub = self.create_publisher(Odometry, '/odom_raw', 50)
        self.vel_pub = self.create_publisher(Twist, '/vel_raw', 50)
        self.tf = TransformBroadcaster(self) if p['publish_odom_tf'] else None

        self.create_subscription(Twist, '/cmd_vel', lambda m: setattr(self, 'cmd', m), 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 50)
        self.control_dt = 1.0 / p['control_rate']
        self.create_timer(self.control_dt, self.control)
        self.create_timer(1.0 / p['odom_rate'], self.publish_odometry)

    def control(self):
        p = self.p
        v = max(-p['max_speed'], min(p['max_speed'], self.cmd.linear.x))
        self.speed = step_toward(self.speed, v, p['max_accel'], self.control_dt)
        target = bicycle_angle_for(v, self.cmd.angular.z, p['wheel_base'])
        _, _, target = wheel_angles(target, p['wheel_base'], p['kingpin_width'], p['steer_limit'])
        self.delta = step_toward(self.delta, target, p['steer_rate'], self.control_dt)
        left, right, _ = wheel_angles(self.delta, p['wheel_base'], p['kingpin_width'], p['steer_limit'])
        wl, wr = rear_wheel_speeds(self.speed, self.delta, p['wheel_base'],
                                   p['wheel_separation'], p['wheel_radius'])
        for joint, value in zip(STEER_JOINTS + WHEEL_JOINTS, (left, right, wl, wr)):
            self.joint_cmd[joint].publish(Float64(data=value))

    def on_joints(self, msg):
        joints = dict(zip(msg.name, range(len(msg.name))))
        if not all(j in joints for j in STEER_JOINTS + WHEEL_JOINTS) or len(msg.velocity) < len(msg.name):
            return
        p = self.p
        self.measured_v = p['wheel_radius'] * sum(msg.velocity[joints[j]] for j in WHEEL_JOINTS) / 2
        self.measured_delta = bicycle_angle_from(*(msg.position[joints[j]] for j in STEER_JOINTS),
                                                 p['kingpin_width'], p['wheel_base'])
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.odometry.update(self.measured_v, self.measured_delta, stamp) or self.stamp is None:
            self.stamp = msg.header.stamp

    def publish_odometry(self):
        if self.stamp is None:
            return
        o = self.odometry
        q_z, q_w = math.sin(o.yaw / 2), math.cos(o.yaw / 2)

        vel = Twist()
        vel.linear.x = self.measured_v
        vel.linear.y = math.degrees(self.measured_delta)
        vel.angular.z = o.w
        self.vel_pub.publish(vel)

        odom = Odometry()
        odom.header.stamp = self.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x, odom.pose.pose.position.y = o.x, o.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = q_z, q_w
        odom.twist.twist.linear.x = self.measured_v
        odom.twist.twist.angular.z = o.w
        for i, c in POSE_COVARIANCE.items():
            odom.pose.covariance[i] = c
        for i, c in TWIST_COVARIANCE.items():
            odom.twist.covariance[i] = c
        self.odom_pub.publish(odom)

        if self.tf:
            t = TransformStamped()
            t.header = odom.header
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x, t.transform.translation.y = o.x, o.y
            t.transform.rotation.z, t.transform.rotation.w = q_z, q_w
            self.tf.sendTransform(t)


def main():
    rclpy.init()
    try:
        rclpy.spin(R2Drive())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass   # Ctrl-C or the launch file stopping us: a normal exit


if __name__ == '__main__':
    main()
