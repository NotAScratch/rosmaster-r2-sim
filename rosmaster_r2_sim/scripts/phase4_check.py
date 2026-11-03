#!/usr/bin/env python3
"""Phase 4 checks: does the simulation look like the real robot to the real stack?

    ros2 launch rosmaster_r2_sim sim.launch.py headless:=true localization:=true
    ros2 run rosmaster_r2_sim phase4_check.py               # the simulation: all checks, drives the robot

    ros2 run rosmaster_r2_sim phase4_check.py --passive     # the real robot (or the sim): no driving,
                                                            # no ground truth, wall-clock time
    add --no-camera when the camera driver is not running.

Parity checks (both modes): topics and types, frames, the TF tree, a single owner of
odom -> base_footprint, /odom_raw's covariances and rate, Madgwick's orientation.
Active mode only: drives a fixed sequence and compares the EKF's /odom with the ground truth.
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

# The real robot's interface (docs/research/00-system-survey.md §3)
TOPICS = {
    '/cmd_vel': 'geometry_msgs/msg/Twist',
    '/vel_raw': 'geometry_msgs/msg/Twist',
    '/odom_raw': 'nav_msgs/msg/Odometry',
    '/imu/data_raw': 'sensor_msgs/msg/Imu',
    '/imu/data': 'sensor_msgs/msg/Imu',
    '/odom': 'nav_msgs/msg/Odometry',
    '/scan': 'sensor_msgs/msg/LaserScan',
    '/joint_states': 'sensor_msgs/msg/JointState',
    '/tf': 'tf2_msgs/msg/TFMessage',
    '/tf_static': 'tf2_msgs/msg/TFMessage',
}
CAMERA_TOPICS = {
    '/camera/color/image_raw': 'sensor_msgs/msg/Image',
    '/camera/depth/image_raw': 'sensor_msgs/msg/Image',
}
TF_CHILDREN = ('laser', 'imu_link', 'camera_link')

# base_node_R2.cpp's /odom_raw covariances (all others 0) and rate (the driver's 0.1 s timer)
ODOM_RAW_POSE_COVARIANCE = {0: 0.001, 7: 0.001, 35: 0.001}
ODOM_RAW_TWIST_COVARIANCE = {0: 0.0001, 7: 0.0001, 35: 0.0001}
ODOM_RAW_RATE = 10.0

# Active mode: (seconds, linear.x, angular.z), the Phase 2 manoeuvres in an open space
MANOEUVRES = [(3.0, 0.2, 0.0), (6.0, 0.2, 0.333), (6.0, 0.2, -0.333), (4.0, 0.3, 0.6),
              (3.0, -0.2, 0.0), (6.0, 0.4, 0.4), (1.5, 0.0, 0.0)]


def stamp_of(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def quat_mul(a, b):
    """Hamilton product of two (w, x, y, z) tuples."""
    return (a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0])


def roll_pitch_of(q):
    """Roll and pitch of a (w, x, y, z) quaternion."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return roll, pitch


class Phase4Check(Node):
    def __init__(self, passive, camera):
        super().__init__('phase4_check', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=not passive)])
        self.passive = passive
        self.topics = dict(TOPICS, **(CAMERA_TOPICS if camera else {}))
        self.latest = {}
        self.odom_raw_stamps = []
        self.odom_by_stamp = {}           # EKF /odom pose by stamp, to compare with its TF
        self.odom_tf = []                 # (stamp, x, y, yaw) of every odom -> base_footprint on /tf
        for topic, kind, qos in (('/odom_raw', Odometry, 50), ('/odom', Odometry, 50),
                                 ('/ground_truth/odom', Odometry, 10), ('/vel_raw', Twist, 10),
                                 ('/imu/data', Imu, qos_profile_sensor_data),
                                 ('/imu/data_raw', Imu, qos_profile_sensor_data),
                                 ('/scan', LaserScan, qos_profile_sensor_data)):
            self.create_subscription(kind, topic, lambda m, t=topic: self.on_msg(t, m), qos)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.results = []

    def on_msg(self, topic, msg):
        self.latest[topic] = msg
        if topic == '/odom_raw':
            self.odom_raw_stamps.append(stamp_of(msg))
        elif topic == '/odom':
            p = msg.pose.pose
            self.odom_by_stamp[stamp_of(msg)] = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_tf(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == 'odom' and t.child_frame_id == 'base_footprint':
                r = t.transform
                self.odom_tf.append((stamp_of(t), r.translation.x, r.translation.y, yaw_of(r.rotation)))

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def spin_for(self, seconds, twist=None, on_tick=None):
        """Spin for `seconds` of node time (sim time unless passive), publishing twist at 10 Hz."""
        end = self.now() + seconds
        last_pub = 0.0
        while rclpy.ok() and self.now() < end:
            if twist is not None and self.now() - last_pub >= 0.1:
                self.cmd.publish(twist)
                last_pub = self.now()
            rclpy.spin_once(self, timeout_sec=0.05)
            if on_tick:
                on_tick()

    def check(self, name, ok, detail):
        self.results.append(ok)
        self.get_logger().info(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}")

    def info(self, name, detail):
        self.get_logger().info(f'INFO  {name}: {detail}')

    def run(self):
        needed = ['/odom_raw', '/odom', '/imu/data', '/imu/data_raw', '/scan', '/vel_raw']
        if not self.passive:
            needed.append('/ground_truth/odom')
        self.get_logger().info('Waiting for ' + ', '.join(needed) + ' ...')
        give_up = time.monotonic() + 60.0
        while rclpy.ok() and time.monotonic() < give_up and (
                self.now() == 0.0 or not all(t in self.latest for t in needed)):
            rclpy.spin_once(self, timeout_sec=0.1)
        missing = [t for t in needed if t not in self.latest]
        if missing:
            self.check('topics: messages arriving', False, f'nothing on {missing}')
            return False

        # Collect for 5 s standing still: rates, TF ownership, Madgwick at rest.
        # imu_link is mounted upside down (rpy 0, pi, pi/2), so Madgwick's orientation of imu_link
        # is judged as the orientation of base_link: world<-imu_link * imu_link<-base_link.
        self.odom_raw_stamps.clear()
        self.odom_tf.clear()
        roll_pitch = []

        def base_roll_pitch():
            if not self.tf_buffer.can_transform('imu_link', 'base_link', rclpy.time.Time()):
                return
            m = self.tf_buffer.lookup_transform('imu_link', 'base_link', rclpy.time.Time()).transform.rotation
            q = self.latest['/imu/data'].orientation
            roll_pitch.append(roll_pitch_of(quat_mul((q.w, q.x, q.y, q.z), (m.w, m.x, m.y, m.z))))

        self.spin_for(5.0, None if self.passive else Twist(), on_tick=base_roll_pitch)

        # Topics and types
        graph = dict(self.get_topic_names_and_types())
        wrong = {t: graph.get(t) for t, kind in self.topics.items() if kind not in graph.get(t, [])}
        self.check('topics: names and types', not wrong,
                   'all of the real robot\'s topics present' if not wrong else f'missing or wrong type: {wrong}')
        self.info('topics: others here', ', '.join(sorted(t for t in graph if t not in self.topics)))

        # Frames
        frames = {
            '/odom_raw': ('odom', 'base_footprint'), '/odom': ('odom', 'base_footprint'),
            '/imu/data_raw': ('imu_link', None), '/imu/data': ('imu_link', None), '/scan': ('laser', None)}
        for topic, (frame, child) in frames.items():
            msg = self.latest[topic]
            got = (msg.header.frame_id, getattr(msg, 'child_frame_id', None) if child else None)
            self.check(f'frames: {topic}', got == (frame, child),
                       f'{got[0]}' + (f' -> {got[1]}' if child else ''))

        # TF tree
        for child in TF_CHILDREN:
            ok = self.tf_buffer.can_transform('odom', child, rclpy.time.Time())
            self.check(f'tf: odom -> base_footprint -> base_link -> {child}', ok, 'found' if ok else 'not resolvable')

        # One owner of odom -> base_footprint: every such transform must be the EKF's /odom at the same stamp
        matched = mismatched = 0
        for stamp, x, y, yaw in self.odom_tf:
            pose = self.odom_by_stamp.get(stamp)
            if pose is None:
                mismatched += 1
            elif math.hypot(x - pose[0], y - pose[1]) < 0.001 and abs(math.remainder(yaw - pose[2], math.tau)) < 0.001:
                matched += 1
            else:
                mismatched += 1
        self.check('tf: odom -> base_footprint comes only from the EKF', matched > 0 and mismatched == 0,
                   f'{matched} transforms match /odom, {mismatched} do not')

        # /odom_raw like base_node_R2
        odom_raw = self.latest['/odom_raw']
        pose_cov = {i: c for i, c in enumerate(odom_raw.pose.covariance) if c != 0.0}
        twist_cov = {i: c for i, c in enumerate(odom_raw.twist.covariance) if c != 0.0}
        self.check('odom_raw: covariances', pose_cov == ODOM_RAW_POSE_COVARIANCE and twist_cov == ODOM_RAW_TWIST_COVARIANCE,
                   f'pose {pose_cov}, twist {twist_cov}')
        stamps = self.odom_raw_stamps
        rate = (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 1 and stamps[-1] > stamps[0] else 0.0
        self.check('odom_raw: rate', abs(rate - ODOM_RAW_RATE) <= 1.0,
                   f'{rate:.1f} Hz in message stamps (real driver: {ODOM_RAW_RATE:.0f} Hz)')

        # Madgwick filter
        q = self.latest['/imu/data'].orientation
        norm = math.sqrt(q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2)
        self.check('imu/data: orientation is a unit quaternion', abs(norm - 1.0) < 1e-3, f'|q| = {norm:.4f}')
        worst = max(max(abs(r), abs(p)) for r, p in roll_pitch) if roll_pitch else float('inf')
        self.check('imu/data: base_link level when still', math.degrees(worst) < 2.0,
                   f'worst roll/pitch {math.degrees(worst):.2f} deg over 5 s')

        if not self.passive:
            self.ekf_vs_truth()

        passed = sum(self.results)
        self.get_logger().info(f'{passed}/{len(self.results)} checks passed')
        return passed == len(self.results)

    def ekf_vs_truth(self):
        """Drive MANOEUVRES; judge /odom (and, for information, /odom_raw) like Phase 2 does:
        total distance and total turning, each summed separately.

        Sums are taken once per second of sim time, the same for every source. Summing every
        message would also count the EKF's yaw zig-zag: the real ekf_x1_x3.yaml fuses absolute yaw
        from both /odom_raw and the IMU, and once those disagree by a few degrees /odom swings
        between them. That swing is reported separately, as the EKF's yaw jitter."""
        keys = ('/odom', '/odom_raw', '/ground_truth/odom')

        def pose(topic):
            p = self.latest[topic].pose.pose
            return p.position.x, p.position.y, yaw_of(p.orientation)

        sums = {k: [0.0, 0.0] for k in keys}
        last = {k: pose(k) for k in keys}
        next_sample = [self.now() + 1.0]
        jitter = {'sum': 0.0, 'net': 0.0}    # /odom yaw while the command is straight
        straight = [False]
        last_ekf_yaw = [pose('/odom')[2]]

        def track(force=False):
            yaw = pose('/odom')[2]
            if yaw != last_ekf_yaw[0]:
                if straight[0]:
                    jitter['sum'] += abs(math.remainder(yaw - last_ekf_yaw[0], math.tau))
                    jitter['net'] += math.remainder(yaw - last_ekf_yaw[0], math.tau)
                last_ekf_yaw[0] = yaw
            if not force and self.now() < next_sample[0]:
                return
            next_sample[0] = self.now() + 1.0
            for k in keys:
                cur, prev = pose(k), last[k]
                sums[k][0] += math.hypot(cur[0] - prev[0], cur[1] - prev[1])
                sums[k][1] += abs(math.remainder(cur[2] - prev[2], math.tau))
                last[k] = cur

        for seconds, v, w in MANOEUVRES:
            twist = Twist()
            twist.linear.x, twist.angular.z = v, w
            straight[0] = w == 0.0
            self.spin_for(seconds, twist, on_tick=track)
        straight[0] = True
        self.spin_for(1.0, Twist(), on_tick=track)
        track(force=True)
        self.info('/odom (EKF): yaw jitter while straight or stopped',
                  f"{math.degrees(jitter['sum']):.1f} deg of back-and-forth for a net "
                  f"{math.degrees(jitter['net']):+.1f} deg (the real ekf_x1_x3.yaml fuses two absolute yaws)")

        truth_dist, truth_turn = sums['/ground_truth/odom']
        for k, dist_tol, turn_tol in (('/odom', 0.02, 0.03), ('/odom_raw', None, None)):
            dist, turn = sums[k]
            dist_err, turn_err = (dist - truth_dist) / truth_dist, (turn - truth_turn) / truth_turn
            d_txt = f'{dist:.3f} m vs {truth_dist:.3f} m true ({100 * dist_err:+.1f}%)'
            t_txt = f'{math.degrees(turn):.1f} deg vs {math.degrees(truth_turn):.1f} deg true ({100 * turn_err:+.1f}%)'
            if dist_tol is None:
                self.info(f'{k}: distance', d_txt)
                self.info(f'{k}: turning', t_txt)
            else:
                self.check(f'{k} (EKF): distance', abs(dist_err) < dist_tol, d_txt)
                self.check(f'{k} (EKF): turning', abs(turn_err) < turn_tol, t_txt)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--passive', action='store_true', help='no driving, no ground truth (for the real robot)')
    parser.add_argument('--no-camera', action='store_true', help='do not require the camera topics')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Phase4Check(args.passive, not args.no_camera)
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
