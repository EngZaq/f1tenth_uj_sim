#!/usr/bin/env python3
"""SIM-ONLY, TEST TOOL: stand-in localizer that uses SIMULATOR TRUTH (Test 6, Step 3).

Replaces AMCL so that the unchanged driver is steered by the true pose. It separates controller /
vehicle limits from localization limits. Never use it to grade a localizer.

  * Truth: /ego_racecar/odom (Gym CoG pose in the sim `map` frame) -> rear axle (lr = 0.17145 m) ->
    slam_map with the fixed SE(2) fitted on calibration run CAL_NT (same file the graders use).
  * For each EKF message (/odometry/filtered, pose of fusion_base in fusion_odom) it publishes
    slam_map -> fusion_odom = truth(slam_map <- fusion_base) * inverse(EKF fusion_odom <- fusion_base),
    stamped like AMCL (odometry stamp + transform_tolerance, default 0.3 s). Then slam_map -> fusion_base
    equals the truth rear-axle pose.
  * /amcl_pose (frame slam_map) at ~10 Hz with a small fixed covariance, so the driver's confidence
    gate and pose checks see a healthy localizer.
Publishes TF with a TransformBroadcaster on /tf; remap it like the EKF (e.g. -r /tf:=/fusion/tf).
Exactly one map->odom owner is required: do not run AMCL at the same time.
"""
import json
import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformBroadcaster

LR = 0.17145


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def stamp_s(s):
    return s.sec + 1e-9 * s.nanosec


class TruthPoseRelay(Node):
    def __init__(self):
        super().__init__('moatasem_truth_pose_relay')
        se2 = self.declare_parameter(
            'se2_json', '/home/motasem/Data/f1tenth_results_archive/raw_logs/amcl/se2_CAL_NT.json').value
        self.tol = float(self.declare_parameter('transform_tolerance', 0.3).value)
        self.sigma_xy = float(self.declare_parameter('pose_sigma_m', 0.02).value)
        self.sigma_yaw = math.radians(float(self.declare_parameter('pose_sigma_deg', 1.0).value))
        self.pose_period = float(self.declare_parameter('pose_period_s', 0.1).value)
        T = json.load(open(se2))
        self.R, self.t, self.th = np.array(T['R']), np.array(T['t']), math.radians(T['theta_deg'])
        self.truth = deque(maxlen=400)          # (stamp, x_slam, y_slam, yaw_slam) of the rear axle
        self.last_pose_pub = 0.0
        self.br = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.on_truth, 50)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_ekf, 50)
        self.get_logger().warn('TRUTH POSE RELAY ACTIVE (sim only): slam_map->fusion_odom and /amcl_pose from truth')

    def on_truth(self, m):
        h = yaw_q(m.pose.pose.orientation)
        rear = np.array([m.pose.pose.position.x - LR * math.cos(h), m.pose.pose.position.y - LR * math.sin(h)])
        p = self.R.T @ (rear - self.t)
        self.truth.append((stamp_s(m.header.stamp), p[0], p[1], h - self.th))

    def truth_at(self, t):
        if len(self.truth) < 2 or not self.truth[0][0] <= t <= self.truth[-1][0] + 0.05:
            return None
        a = np.array(self.truth)
        yaw = np.unwrap(a[:, 3])
        return (float(np.interp(t, a[:, 0], a[:, 1])), float(np.interp(t, a[:, 0], a[:, 2])),
                float(np.interp(t, a[:, 0], yaw)))

    def on_ekf(self, m):
        t = stamp_s(m.header.stamp)
        tp = self.truth_at(t)
        if tp is None:
            return
        mx, my, myaw = tp
        ox, oy, oyaw = m.pose.pose.position.x, m.pose.pose.position.y, yaw_q(m.pose.pose.orientation)
        # map->odom = map->base * (odom->base)^-1
        yaw = myaw - oyaw
        c, s = math.cos(yaw), math.sin(yaw)
        tx, ty = mx - (c * ox - s * oy), my - (s * ox + c * oy)
        tf = TransformStamped()
        tf.header.stamp = (Time.from_msg(m.header.stamp) + rclpy.duration.Duration(seconds=self.tol)).to_msg()
        tf.header.frame_id, tf.child_frame_id = 'slam_map', 'fusion_odom'
        tf.transform.translation.x, tf.transform.translation.y = tx, ty
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.br.sendTransform(tf)
        if t - self.last_pose_pub >= self.pose_period:
            self.last_pose_pub = t
            p = PoseWithCovarianceStamped()
            p.header.stamp, p.header.frame_id = m.header.stamp, 'slam_map'
            p.pose.pose.position.x, p.pose.pose.position.y = mx, my
            p.pose.pose.orientation.z, p.pose.pose.orientation.w = math.sin(myaw / 2), math.cos(myaw / 2)
            cov = [0.0] * 36
            cov[0] = cov[7] = self.sigma_xy ** 2
            cov[35] = self.sigma_yaw ** 2
            p.pose.covariance = cov
            self.pose_pub.publish(p)


def main():
    rclpy.init()
    node = TruthPoseRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
