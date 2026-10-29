#!/usr/bin/env python3
"""Compare EKF vs AMCL relative displacement against time-matched sim truth.

Ground truth is used for evaluation only. Each source is anchored at its first
successful synchronized sample; results do not measure absolute map alignment.
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


def stamp(s):
    return s.sec * 1000000000 + s.nanosec


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def xyh(p):
    q = p.orientation
    return (p.position.x, p.position.y, math.atan2(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y*q.y + q.z*q.z)))


def rel(p, origin):
    dx, dy = p[0]-origin[0], p[1]-origin[1]
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return (c*dx+s*dy, -s*dx+c*dy, wrap(p[2]-origin[2]))


class Monitor(Node):
    def __init__(self):
        super().__init__('fusion_amcl_accuracy_monitor')
        self.truth = deque(maxlen=5000)
        self.pending = {'EKF': deque(maxlen=200), 'AMCL': deque(maxlen=200)}
        self.origins = {}
        self.last = {}
        self.errors = {'EKF': [], 'AMCL': []}
        self.start = time.monotonic()
        path = Path.home() / 'f1tenth_sim/results/localization/fusion_amcl_comparison.csv'
        path.parent.mkdir(parents=True, exist_ok=True)
        self.fp = path.open('w', newline='')
        self.csv = csv.writer(self.fp)
        self.csv.writerow(['source', 'wall_elapsed_s', 'stamp_ns', 'position_error_m',
                           'heading_error_deg', 'est_speed_mps', 'truth_speed_mps',
                           'timestamp_mismatch_ms'])
        self.create_subscription(Odometry, '/ego_racecar/odom', self.on_truth, 20)
        self.create_subscription(Odometry, '/odometry/filtered',
                                 lambda m: self.on_est('EKF', m.header.stamp, m.pose.pose), 20)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 lambda m: self.on_est('AMCL', m.header.stamp, m.pose.pose), 20)
        self.get_logger().info(f'Evaluation started: {path}')

    def on_truth(self, msg):
        t = stamp(msg.header.stamp)
        if self.truth and t < self.truth[-1][0]:
            self.get_logger().warn('Truth timestamps moved backwards; resetting comparisons')
            self.truth.clear()
            self.origins.clear()
            self.last.clear()
            for q in self.pending.values(): q.clear()
        self.truth.append((t, xyh(msg.pose.pose)))
        self.match()

    def on_est(self, name, ts, p):
        self.pending[name].append((stamp(ts), xyh(p)))
        self.match()

    def match(self):
        if len(self.truth) < 2:
            return
        data = list(self.truth)
        stamps = [t for t, _ in data]
        for name, q in self.pending.items():
            while q and q[0][0] <= stamps[-1] - 5000000:
                t, est = q.popleft()
                if t < stamps[0]:
                    continue
                i = bisect_left(stamps, t)
                gt_stamp, truth = min(data[max(0, i-1):min(len(data), i+1)],
                                       key=lambda x: abs(x[0]-t))
                lag = abs(gt_stamp-t) / 1e6
                if lag > 60:
                    continue
                if name not in self.origins:
                    self.origins[name] = (truth, est)
                    self.get_logger().info(f'{name} initial reference captured')
                    continue
                true_rel = rel(truth, self.origins[name][0])
                est_rel = rel(est, self.origins[name][1])
                err = math.hypot(est_rel[0]-true_rel[0], est_rel[1]-true_rel[1])
                heading = math.degrees(abs(wrap(est_rel[2]-true_rel[2])))
                espeed = tspeed = 0.0
                if name in self.last:
                    old_t, old_e, old_true = self.last[name]
                    dt = (t-old_t) / 1e9
                    if dt > 0:
                        espeed = math.hypot(est[0]-old_e[0], est[1]-old_e[1]) / dt
                        tspeed = math.hypot(truth[0]-old_true[0], truth[1]-old_true[1]) / dt
                        if dt < 0.5 and espeed > 3:
                            self.get_logger().warn(
                                f'{name} apparent speed jump: {espeed:.2f}m/s, '
                                f'truth={tspeed:.2f}m/s')
                self.last[name] = (t, est, truth)
                self.errors[name].append(err)
                elapsed = time.monotonic()-self.start
                self.csv.writerow([name, round(elapsed, 3), t, round(err, 4),
                                   round(heading, 3), round(espeed, 3), round(tspeed, 3),
                                   round(lag, 1)])
        self.fp.flush()

    def finish(self):
        for name in ('EKF', 'AMCL'):
            errors = self.errors[name]
            if errors:
                rmse = math.sqrt(sum(e*e for e in errors)/len(errors))
                print(f'{name}: samples={len(errors)}, mean={sum(errors)/len(errors):.3f}m, '
                      f'RMSE={rmse:.3f}m, max={max(errors):.3f}m', flush=True)
            else:
                print(f'{name}: NO synchronized samples; check topics/TF', flush=True)
        self.fp.close()


def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
