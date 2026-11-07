#!/usr/bin/env python3
"""Sample the pose a controller would use, at a fixed rate, for offline grading (sim only).

Every 1/rate_hz seconds (wall clock):
  * TF lookup slam_map -> fusion_base   (latest available, as a controller would do it)
  * TF lookup slam_map -> fusion_odom   (the AMCL correction currently in force)
  * simulator truth /ego_racecar/odom interpolated at the sample time (CoG pose, raw;
    the grader converts it to the rear axle). Truth is recorded for GRADING ONLY.
  * latest /amcl_pose covariance and /particle_cloud size
Writes one CSV row per sample (output_csv parameter). Never publishes anything.

CAUTION: with the Gym running, /tf carries ~1200 msg/s (wheel joints). This Python
TransformListener then falls behind by up to ~0.4 s at times, so its POSE columns can be
stale. Use them for covariance / particle trends; grade poses from a rosbag replayed through
a tf2 buffer in receive order (what a C++ controller's lookup would return).
"""
import csv
import math
from bisect import bisect_left
from collections import deque

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.msg import ParticleCloud
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Sampler(Node):
    def __init__(self):
        super().__init__('amcl_shadow_sampler')
        self.declare_parameter('output_csv', 'amcl_shadow_samples.csv')
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('map_frame', 'slam_map')
        self.declare_parameter('odom_frame', 'fusion_odom')
        self.declare_parameter('base_frame', 'fusion_base')
        p = lambda n: self.get_parameter(n).value
        self.map, self.odom, self.base = p('map_frame'), p('odom_frame'), p('base_frame')
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.truth = deque(maxlen=2000)
        self.cov = (math.nan, math.nan, math.nan)
        self.amcl_updates = 0
        self.particles = -1
        self.create_subscription(Odometry, '/ego_racecar/odom', self.on_truth, 50)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 10)
        self.create_subscription(ParticleCloud, '/particle_cloud', self.on_cloud, qos_profile_sensor_data)
        self.fp = open(p('output_csv'), 'w', newline='')
        self.w = csv.writer(self.fp)
        self.w.writerow(['t', 'tf_stamp', 'x', 'y', 'yaw', 'corr_stamp', 'cx', 'cy', 'cyaw',
                         'truth_x', 'truth_y', 'truth_yaw', 'truth_vx', 'truth_age_ms',
                         'cov_xx', 'cov_yy', 'cov_yaw', 'amcl_updates', 'particles'])
        self.create_timer(1.0 / float(p('rate_hz')), self.tick)
        self.get_logger().info(f'sampling {self.map}->{self.base} at {p("rate_hz")} Hz -> {p("output_csv")}')

    def on_truth(self, m):
        t = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
        self.truth.append((t, m.pose.pose.position.x, m.pose.pose.position.y,
                           yaw_of(m.pose.pose.orientation), m.twist.twist.linear.x))

    def on_amcl(self, m):
        c = m.pose.covariance
        self.cov = (c[0], c[7], c[35])
        self.amcl_updates += 1

    def on_cloud(self, m):
        self.particles = len(m.particles)

    def truth_at(self, t):
        if len(self.truth) < 2:
            return None
        ts = [s[0] for s in self.truth]
        i = bisect_left(ts, t)
        if i >= len(ts):                       # newer than the last truth sample: hold it
            s = self.truth[-1]
            return s[1:], 1000 * (t - s[0])
        if i == 0:
            return None
        a, b = self.truth[i - 1], self.truth[i]
        f = (t - a[0]) / max(b[0] - a[0], 1e-9)
        dyaw = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
        return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), a[3] + f * dyaw,
                a[4] + f * (b[4] - a[4])), 0.0

    def tick(self):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        try:
            tb = self.buf.lookup_transform(self.map, self.base, Time())
            tc = self.buf.lookup_transform(self.map, self.odom, Time())
        except TransformException:
            return
        tr = self.truth_at(t)
        if tr is None:
            return
        (gx, gy, gyaw, gv), age = tr
        st = lambda h: h.stamp.sec + 1e-9 * h.stamp.nanosec
        b, c = tb.transform, tc.transform
        self.w.writerow([f'{t:.6f}', f'{st(tb.header):.6f}', b.translation.x, b.translation.y, yaw_of(b.rotation),
                         f'{st(tc.header):.6f}', c.translation.x, c.translation.y, yaw_of(c.rotation),
                         gx, gy, gyaw, gv, round(age, 2), *self.cov, self.amcl_updates, self.particles])

    def close(self):
        self.fp.flush()
        self.fp.close()


def main():
    rclpy.init()
    node = Sampler()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
