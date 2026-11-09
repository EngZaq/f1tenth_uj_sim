#!/usr/bin/env python3
"""Adapter for the F1TENTH / MIT particle filter (Test 4): map->laser  ->  slam_map->fusion_odom.

Why it is needed
  The particle filter (github.com/f1tenth/particle_filter, humble-devel) hard-codes ONE TF output:
  '/map' -> '/laser' (the pose of the LiDAR in the map). It has no parameter to turn that off and
  no map->odom mode. In our tree fusion_odom -> fusion_base belongs to the EKF and
  fusion_base -> fusion_laser is static, so a map -> laser edge would give fusion_laser a second
  parent. pf_shadow.launch.py therefore remaps the filter's /tf to /pf/tf_raw (nothing of it reaches
  /tf) and this node publishes the ONLY localization edge, slam_map -> fusion_odom, like AMCL.

What it does, per particle-filter update (one per EKF message, ~30 Hz)
  1. PF pose of the laser in slam_map, stamped with the stamp of the EKF odometry message the PF used.
  2. base pose = laser pose composed with inverse(fusion_base -> fusion_laser)  (static TF, looked up).
  3. map->odom = base pose composed with inverse(EKF odom->base pose of THAT SAME odometry message),
     so the correction is exact for the odometry the filter used (no time interpolation).
  4. TF slam_map -> fusion_odom, stamp = that odometry stamp + transform_tolerance (AMCL's convention).
  5. /pf/pose/odom carries the weighted particle covariance, but the package packs the 3x3 (x, y, theta)
     matrix into covariance[0:9]. Here it is moved to the standard 6x6 slots (x=0, y=7, yaw=35, ...)
     and propagated from the laser point to the rear axle (Jacobian of step 2). The base pose with that
     covariance is published as geometry_msgs/PoseWithCovarianceStamped on pose_topic (frame slam_map),
     default /amcl_pose so the existing controller / sampler can be used unchanged. Run ONE localizer
     at a time.
  Also relays /fusion/scan (BEST_EFFORT) 1:1 to /pf/scan (RELIABLE): the PF subscribes with default
  (reliable) QoS and would otherwise receive no scans.
  Nothing is published before a pose arrives on /fusion/initialpose (never /initialpose: that topic
  teleports the Gym car) plus skip_after_seed PF updates.
"""
import csv
import math
from collections import OrderedDict

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def compose(a, b):
    """SE(2) a * b, poses as (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


class Adapter(Node):
    def __init__(self):
        super().__init__('pf_map_odom_adapter')
        for k, v in {'pf_tf_topic': '/pf/tf_raw', 'pf_odom_topic': '/pf/pose/odom',
                     'odom_topic': '/odometry/filtered', 'initialpose_topic': '/fusion/initialpose',
                     'pose_topic': '/amcl_pose', 'scan_in': '/fusion/scan', 'scan_out': '/pf/scan',
                     'map_frame': 'slam_map', 'odom_frame': 'fusion_odom', 'base_frame': 'fusion_base',
                     'laser_frame': 'fusion_laser', 'transform_tolerance': 0.3, 'skip_after_seed': 5,
                     'log_csv': ''}.items():
            self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value
        self.mf, self.of, self.bf, self.lf = p('map_frame'), p('odom_frame'), p('base_frame'), p('laser_frame')
        self.tol_ns = int(float(p('transform_tolerance')) * 1e9)
        self.skip_after_seed = int(p('skip_after_seed'))
        self.odom = OrderedDict()            # stamp ns -> (x, y, yaw) of fusion_base in fusion_odom
        self.laser = None                    # fusion_base -> fusion_laser (x, y, yaw)
        self.seeded = False
        self.updates_since_seed = 0
        self.last = None                     # (stamp msg, laser pose, base pose) of the last PF update
        self.n_pub = self.n_nomatch = 0
        # fusion_base -> fusion_laser is read from /tf_static only (no full TransformListener: subscribing to
        # /tf at the simulator's ~1400 msg/s cost ~70 % of a core in Python for one static transform).
        self.create_subscription(TFMessage, '/tf_static', self.on_tf_static,
                                 QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                            reliability=ReliabilityPolicy.RELIABLE))
        self.tfb = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, p('pose_topic'), 10)
        self.scan_pub = self.create_publisher(LaserScan, p('scan_out'),
                                              QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(LaserScan, p('scan_in'), self.scan_pub.publish, qos_profile_sensor_data)
        self.create_subscription(Odometry, p('odom_topic'), self.on_odom, 50)
        self.create_subscription(TFMessage, p('pf_tf_topic'), self.on_pf_tf, 50)
        self.create_subscription(Odometry, p('pf_odom_topic'), self.on_pf_odom, 50)
        self.create_subscription(PoseWithCovarianceStamped, p('initialpose_topic'), self.on_seed, 10)
        self.log = None
        if p('log_csv') not in ('', 'none'):
            self.log_fp = open(p('log_csv'), 'w', newline='')
            self.log = csv.writer(self.log_fp)
            self.log.writerow(['wall', 'stamp', 'x', 'y', 'yaw', 'mo_x', 'mo_y', 'mo_yaw'])
        self.create_timer(5.0, self.report)
        self.get_logger().info(f'waiting for {p("initialpose_topic")}; publishing ONLY {self.mf}->{self.of}')

    def report(self):
        self.get_logger().info(f'seeded={self.seeded} map->odom published={self.n_pub} '
                               f'PF updates without matching odometry={self.n_nomatch}')

    def on_seed(self, msg):
        self.seeded = True
        self.updates_since_seed = 0
        self.get_logger().info(f'seed received ({msg.pose.pose.position.x:.3f}, {msg.pose.pose.position.y:.3f})')

    def on_odom(self, m):
        self.odom[ns(m.header.stamp)] = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))
        while len(self.odom) > 300:
            self.odom.popitem(last=False)

    def on_tf_static(self, msg):
        for t in msg.transforms:
            if self.laser is None and (t.header.frame_id, t.child_frame_id) == (self.bf, self.lf):
                tr = t.transform
                self.laser = (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))
                self.get_logger().info(f'{self.bf}->{self.lf} = {self.laser}')

    def laser_offset(self):
        return self.laser

    def on_pf_tf(self, msg):
        for t in msg.transforms:
            off = self.laser_offset()
            if off is None or not self.seeded:
                return
            self.updates_since_seed += 1
            if self.updates_since_seed <= self.skip_after_seed:
                return
            k = ns(t.header.stamp)
            ob = self.odom.get(k)
            if ob is None:
                self.n_nomatch += 1
                return
            lp = (t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation))
            bp = compose(lp, inverse(off))
            mo = compose(bp, inverse(ob))
            out = TransformStamped()
            stamp = k + self.tol_ns
            out.header.stamp.sec, out.header.stamp.nanosec = stamp // 1_000_000_000, stamp % 1_000_000_000
            out.header.frame_id, out.child_frame_id = self.mf, self.of
            out.transform.translation.x, out.transform.translation.y = mo[0], mo[1]
            out.transform.rotation.z, out.transform.rotation.w = math.sin(mo[2] / 2), math.cos(mo[2] / 2)
            self.tfb.sendTransform(out)
            self.n_pub += 1
            self.last = (t.header.stamp, lp, bp)
            if self.log:
                self.log.writerow([f'{self.get_clock().now().nanoseconds * 1e-9:.6f}', f'{k * 1e-9:.6f}',
                                   *[f'{v:.6f}' for v in (*bp, *mo)]])

    def on_pf_odom(self, m):
        if self.last is None:
            return
        stamp, lp, bp = self.last
        if abs(m.pose.pose.position.x - lp[0]) > 1e-6 or abs(m.pose.pose.position.y - lp[1]) > 1e-6:
            return                                   # not the update we just converted
        c = np.array(m.pose.covariance[:9], dtype=float).reshape(3, 3)   # package layout: (x, y, theta) 3x3
        dx, dy, dyaw = self.laser
        th = lp[2] - dyaw                                                 # base yaw
        # base_xy = laser_xy - R(base_yaw) [dx, dy], base_yaw = laser_yaw - dyaw
        J = np.array([[1.0, 0.0, math.sin(th) * dx + math.cos(th) * dy],
                      [0.0, 1.0, -math.cos(th) * dx + math.sin(th) * dy],
                      [0.0, 0.0, 1.0]])
        cb = J @ c @ J.T
        out = PoseWithCovarianceStamped()
        out.header.stamp, out.header.frame_id = stamp, self.mf
        out.pose.pose.position.x, out.pose.pose.position.y = bp[0], bp[1]
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = math.sin(bp[2] / 2), math.cos(bp[2] / 2)
        cov = [0.0] * 36
        idx = (0, 1, 5)
        for i in range(3):
            for j in range(3):
                cov[idx[i] * 6 + idx[j]] = float(cb[i, j])
        out.pose.covariance = cov
        self.pose_pub.publish(out)


def main():
    rclpy.init()
    n = Adapter()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if n.log:
            n.log_fp.close()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
