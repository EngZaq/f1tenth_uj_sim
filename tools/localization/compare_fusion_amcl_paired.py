#!/usr/bin/env python3
"""Compare EKF and AMCL at identical ROS timestamps and one common anchor.

This is an evaluation-only node. Ground truth is never used for navigation.
AMCL is sampled at its pose publication stamps, and both EKF and simulator
truth are interpolated to that same stamp. Relative paths are aligned to the
first *common* sample; this is NOT a test of absolute map-frame alignment.
"""
import csv
import math
import time
from bisect import bisect_left
from collections import deque
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


def ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def unpack(pose):
    q = pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    return (pose.position.x, pose.position.y, yaw)


def relative(pose, origin):
    dx, dy = pose[0] - origin[0], pose[1] - origin[1]
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return (c * dx + s * dy, -s * dx + c * dy, wrap(pose[2] - origin[2]))


def interpolate(buffer, timestamp, tolerance_ns=80_000_000):
    """No extrapolation. Returns None if nearby samples don't bracket stamp."""
    if len(buffer) < 2 or timestamp < buffer[0][0] or timestamp > buffer[-1][0]:
        return None
    stamps = [s for s, _ in buffer]
    i = bisect_left(stamps, timestamp)
    if i == 0:
        return buffer[0][1]
    if i == len(buffer):
        return buffer[-1][1]
    before_t, before = buffer[i-1]
    after_t, after = buffer[i]
    if timestamp - before_t > tolerance_ns or after_t - timestamp > tolerance_ns:
        return None
    if after_t == before_t:
        return after
    a = (timestamp - before_t) / (after_t - before_t)
    return (before[0] + a * (after[0]-before[0]),
            before[1] + a * (after[1]-before[1]),
            wrap(before[2] + a * wrap(after[2]-before[2])))


class PairedMonitor(Node):
    def __init__(self):
        super().__init__('fusion_amcl_paired_monitor')
        self.truth = deque(maxlen=5000)
        self.ekf = deque(maxlen=2000)
        self.pending_amcl = deque(maxlen=400)
        self.origins = None
        self.first_t = None
        self.previous = None
        self.last_report = 0.0
        self.errors = {'EKF': [], 'AMCL': []}
        self.yaw_errors = {'EKF': [], 'AMCL': []}
        self.spikes = {'EKF': 0, 'AMCL': 0}
        self.samples = 0
        self.path = Path.home() / 'f1tenth_sim/results/localization/fusion_amcl_paired.csv'
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_handle = self.path.open('w', newline='')
        self.writer = csv.writer(self.csv_handle)
        self.writer.writerow([
            'elapsed_s', 'stamp_ns', 'ekf_position_error_m',
            'amcl_position_error_m', 'ekf_heading_error_deg',
            'amcl_heading_error_deg', 'ekf_speed_mps', 'amcl_speed_mps',
            'true_speed_mps', 'amcl_covariance_xx', 'amcl_covariance_yy',
            'amcl_covariance_yaw', 'true_x', 'true_y', 'amcl_x', 'amcl_y',
        ])
        self.create_subscription(Odometry, '/ego_racecar/odom', self.on_truth, 30)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_ekf, 30)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 30)
        self.get_logger().info(f'Paired evaluator ready: {self.path}')
        self.get_logger().info('Wait for COMMON reference captured before driving if possible.')

    def on_truth(self, msg):
        stamp = ns(msg.header.stamp)
        if self.truth and stamp < self.truth[-1][0]:
            self.get_logger().warn('Ground-truth clock reset. Restart evaluator to reset anchors.')
            return
        self.truth.append((stamp, unpack(msg.pose.pose)))
        self.process()

    def on_ekf(self, msg):
        stamp = ns(msg.header.stamp)
        if self.ekf and stamp < self.ekf[-1][0]:
            self.get_logger().warn('EKF clock reset. Restart evaluator to reset anchors.')
            return
        self.ekf.append((stamp, unpack(msg.pose.pose)))
        self.process()

    def on_amcl(self, msg):
        self.pending_amcl.append((ns(msg.header.stamp), unpack(msg.pose.pose), msg.pose.covariance))
        self.process()

    def process(self):
        if len(self.truth) < 2 or len(self.ekf) < 2:
            return
        while self.pending_amcl:
            stamp, amcl, covariance = self.pending_amcl[0]
            if stamp > min(self.truth[-1][0], self.ekf[-1][0]):
                return  # Wait for interpolatable poses at this timestamp.
            self.pending_amcl.popleft()
            true_pose = interpolate(self.truth, stamp)
            ekf_pose = interpolate(self.ekf, stamp)
            if true_pose is None or ekf_pose is None:
                continue
            if self.origins is None:
                self.origins = (true_pose, ekf_pose, amcl)
                self.first_t = stamp
                self.previous = (stamp, true_pose, ekf_pose, amcl)
                self.get_logger().info('COMMON reference captured for EKF and AMCL — start driving.')
                continue

            truth_r = relative(true_pose, self.origins[0])
            ekf_r = relative(ekf_pose, self.origins[1])
            amcl_r = relative(amcl, self.origins[2])

            ekf_err = math.hypot(ekf_r[0]-truth_r[0], ekf_r[1]-truth_r[1])
            amcl_err = math.hypot(amcl_r[0]-truth_r[0], amcl_r[1]-truth_r[1])
            ekf_yaw = math.degrees(abs(wrap(ekf_r[2]-truth_r[2])))
            amcl_yaw = math.degrees(abs(wrap(amcl_r[2]-truth_r[2])))
            elapsed = (stamp - self.first_t) / 1e9
            last_stamp, last_true, last_ekf, last_amcl = self.previous
            dt = (stamp-last_stamp)/1e9
            if dt <= 0:
                continue
            true_v = math.hypot(true_pose[0]-last_true[0], true_pose[1]-last_true[1])/dt
            ekf_v = math.hypot(ekf_pose[0]-last_ekf[0], ekf_pose[1]-last_ekf[1])/dt
            amcl_v = math.hypot(amcl[0]-last_amcl[0], amcl[1]-last_amcl[1])/dt
            self.previous = (stamp, true_pose, ekf_pose, amcl)

            for name, err, heading, speed in (
                ('EKF', ekf_err, ekf_yaw, ekf_v),
                ('AMCL', amcl_err, amcl_yaw, amcl_v),
            ):
                self.errors[name].append(err)
                self.yaw_errors[name].append(heading)
                if dt < 0.5 and speed > 3.0:
                    self.spikes[name] += 1
                    self.get_logger().warn(
                        f'JUMP {name}: apparent={speed:.2f} m/s, truth={true_v:.2f} m/s'
                    )
            self.samples += 1
            self.writer.writerow([
                round(elapsed, 3), stamp, round(ekf_err, 4), round(amcl_err, 4),
                round(ekf_yaw, 3), round(amcl_yaw, 3), round(ekf_v, 3),
                round(amcl_v, 3), round(true_v, 3),
                covariance[0], covariance[7], covariance[35],
                true_pose[0], true_pose[1], amcl[0], amcl[1],
            ])
            if elapsed - self.last_report >= 10.0:
                self.last_report = elapsed
                self.get_logger().info(
                    f't={elapsed:.1f}s | EKF={ekf_err:.3f}m, '
                    f'AMCL={amcl_err:.3f}m | yaw EKF={ekf_yaw:.2f}deg, '
                    f'AMCL={amcl_yaw:.2f}deg')
            if self.samples % 20 == 0:
                self.csv_handle.flush()

    def finish(self):
        import statistics
        for name in ('EKF', 'AMCL'):
            values = self.errors[name]
            if not values:
                print(f'{name}: NO PAIRED DATA. Check topics and ROS timestamps.')
                continue
            print(f'\n{name}: paired samples={len(values)}')
            print(f'  mean error: {statistics.mean(values):.3f} m')
            print(f'  RMSE:       {math.sqrt(statistics.mean(v*v for v in values)):.3f} m')
            print(f'  max error:  {max(values):.3f} m')
            print(f'  max yaw:    {max(self.yaw_errors[name]):.2f} deg')
            print(f'  jumps >3m/s: {self.spikes[name]}')
        self.csv_handle.close()
        print(f'CSV: {self.path}', flush=True)


def main():
    rclpy.init()
    monitor = PairedMonitor()
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.finish()
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
