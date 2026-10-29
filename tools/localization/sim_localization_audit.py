#!/usr/bin/env python3
"""SIMULATION-ONLY passive localization evaluation with simulator truth.

Subscriber-only: publishes no drive/TF/pose commands and DOES NOT provide
truth to AMCL, EKF or the controller.

Calibrates a single rigid 2-D transform at its first synchronized observation
between simulator map and SLAM map. Later errors are RELATIVE DRIFT from the
initial alignment (not independently verified absolute map accuracy).
"""
import argparse
import csv
import math
import time
from collections import deque
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


def stamp(msg):
    t = msg.header.stamp
    return float(t.sec) + float(t.nanosec) * 1e-9


def yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))


def wrap(t):
    return math.atan2(math.sin(t), math.cos(t))


def pose3(p):
    return float(p.position.x), float(p.position.y), yaw(p.orientation)


def transform_pose(original, rotation, translation):
    c, s = math.cos(rotation), math.sin(rotation)
    x, y, heading = original
    return (c*x-s*y+translation[0], s*x+c*y+translation[1], wrap(heading+rotation))


def rigid_alignment(source, destination):
    rotation = wrap(destination[2]-source[2])
    c, s = math.cos(rotation), math.sin(rotation)
    tx = destination[0] - (c*source[0] - s*source[1])
    ty = destination[1] - (s*source[0] + c*source[1])
    return rotation, (tx, ty)


class Audit(Node):
    def __init__(self, output):
        super().__init__('sim_localization_audit')
        self.truth = deque(maxlen=100)
        self.ekf = deque(maxlen=100)
        self.amcl_alignment = None
        self.ekf_alignment = None
        self.start_timestamp = None
        self.last_log = 0.
        self.path = Path(output)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.output_fp = self.path.open('w', newline='')
        self.csv = csv.writer(self.output_fp)
        self.csv.writerow([
            'stamp_s', 'elapsed_s',
            'truth_x_sim', 'truth_y_sim', 'truth_yaw_deg',
            'amcl_x_map', 'amcl_y_map', 'amcl_yaw_deg',
            'amcl_relative_pos_error_m', 'amcl_relative_yaw_error_deg',
            'ekf_relative_pos_error_m', 'ekf_relative_yaw_error_deg',
            'amcl_pos_std_m', 'amcl_yaw_std_deg',
            'truth_pair_lag_s', 'ekf_pair_lag_s',
            'truth_stamp_s', 'ekf_stamp_s',
            'aligned_truth_x_map', 'aligned_truth_y_map', 'aligned_truth_yaw_deg',
            'ekf_x_odom', 'ekf_y_odom', 'ekf_yaw_deg',
            'amcl_cov_xx', 'amcl_cov_xy', 'amcl_cov_x_yaw',
            'amcl_cov_yx', 'amcl_cov_yy', 'amcl_cov_y_yaw',
            'amcl_cov_yaw_x', 'amcl_cov_yaw_y', 'amcl_cov_yaw_yaw'])
        self.output_fp.flush()
        self.create_subscription(Odometry, '/ego_racecar/odom', self.on_truth, 30)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_ekf, 30)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 30)
        self.get_logger().warn(
            'PASSIVE ORACLE EVALUATION ONLY: truth never feeds driving. '
            'Errors are relative to first pose alignment, not absolute mapping error.')

    def on_truth(self, msg):
        self.truth.append((stamp(msg), pose3(msg.pose.pose)))

    def on_ekf(self, msg):
        self.ekf.append((stamp(msg), pose3(msg.pose.pose)))

    @staticmethod
    def closest(history, timestamp, max_lag=0.18):
        if not history:
            return None
        sample = min(history, key=lambda x: abs(x[0]-timestamp))
        if abs(sample[0]-timestamp) > max_lag:
            return None
        return sample

    def on_amcl(self, msg):
        if msg.header.frame_id != 'slam_map':
            return
        t = stamp(msg)
        truth = self.closest(self.truth, t)
        if truth is None:
            return
        ekf = self.closest(self.ekf, t)
        amcl_pose = pose3(msg.pose.pose)
        true_pose = truth[1]

        if self.amcl_alignment is None:
            self.amcl_alignment = rigid_alignment(true_pose, amcl_pose)
            self.start_timestamp = t
            self.get_logger().info('Initial true->slam_map rigid alignment fixed; tracking subsequent drift')
        aligned_truth = transform_pose(true_pose, *self.amcl_alignment)
        amcl_pos_error = math.hypot(amcl_pose[0]-aligned_truth[0],
                                    amcl_pose[1]-aligned_truth[1])
        amcl_yaw_error = math.degrees(abs(wrap(amcl_pose[2]-aligned_truth[2])))

        ekf_pos_error = ekf_yaw_error = float('nan')
        if ekf is not None:
            if self.ekf_alignment is None:
                self.ekf_alignment = rigid_alignment(true_pose, ekf[1])
            aligned_ekf_truth = transform_pose(true_pose, *self.ekf_alignment)
            ekf_pos_error = math.hypot(ekf[1][0]-aligned_ekf_truth[0],
                                       ekf[1][1]-aligned_ekf_truth[1])
            ekf_yaw_error = math.degrees(abs(wrap(ekf[1][2]-aligned_ekf_truth[2])))

        covariance = msg.pose.covariance
        pos_std = (math.sqrt(max(covariance[0],covariance[7]))
                   if all(math.isfinite(covariance[i]) and covariance[i]>=0 for i in (0,7))
                   else float('nan'))
        yaw_std = (math.degrees(math.sqrt(covariance[35]))
                   if math.isfinite(covariance[35]) and covariance[35]>=0
                   else float('nan'))
        self.csv.writerow([
            f'{t:.6f}', f'{t-self.start_timestamp:.3f}',
            f'{true_pose[0]:.5f}', f'{true_pose[1]:.5f}', f'{math.degrees(true_pose[2]):.3f}',
            f'{amcl_pose[0]:.5f}', f'{amcl_pose[1]:.5f}', f'{math.degrees(amcl_pose[2]):.3f}',
            f'{amcl_pos_error:.5f}', f'{amcl_yaw_error:.3f}',
            f'{ekf_pos_error:.5f}', f'{ekf_yaw_error:.3f}',
            f'{pos_std:.5f}', f'{yaw_std:.3f}',
            f'{abs(truth[0]-t):.3f}', f'{abs(ekf[0]-t):.3f}' if ekf is not None else 'nan',
            f'{truth[0]:.9f}', f'{ekf[0]:.9f}' if ekf is not None else 'nan',
            f'{aligned_truth[0]:.9f}', f'{aligned_truth[1]:.9f}',
            f'{math.degrees(aligned_truth[2]):.6f}',
            *([f'{ekf[1][0]:.9f}', f'{ekf[1][1]:.9f}',
               f'{math.degrees(ekf[1][2]):.6f}'] if ekf is not None else ['nan']*3),
            *[f'{covariance[i]:.12g}' for i in (0, 1, 5, 6, 7, 11, 30, 31, 35)]])
        mono=time.monotonic()
        if mono-self.last_log>2:
            self.output_fp.flush()
            self.get_logger().info(
                f'AMCL relative error={amcl_pos_error:.3f}m / '
                f'{amcl_yaw_error:.1f}deg; filter std={pos_std:.3f}m / '
                f'{yaw_std:.1f}deg; EKF relative err={ekf_pos_error:.3f}m')
            self.last_log=mono

    def close(self):
        if not self.output_fp.closed:
            self.output_fp.flush()
            self.output_fp.close()


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--output',required=True)
    args=ap.parse_args()
    rclpy.init(args=[])
    node=Audit(args.output)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    main()
