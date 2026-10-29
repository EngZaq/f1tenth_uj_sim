#!/usr/bin/env python3
"""Evaluate EKF odometry vs simulator truth, aligned by ROS timestamp.

Evaluation-only truth subscription. Reports relative drift; does NOT create map
localization, feed truth to EKF, or certify deployment readiness.
"""
import csv
import math
from bisect import bisect_left
from collections import deque
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def stamp_ns(m):
    return m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def get_pose(m):
    p = m.pose.pose
    q = p.orientation
    yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
    return p.position.x, p.position.y, yaw


def relative(p, origin):
    dx, dy = p[0]-origin[0], p[1]-origin[1]
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return c*dx + s*dy, -s*dx + c*dy, wrap(p[2]-origin[2])


class FusionEvaluator(Node):
    def __init__(self):
        super().__init__('fusion_accuracy_evaluator')
        self.truth = deque(maxlen=5000)
        self.filtered = deque(maxlen=2000)
        self.truth_origin = None
        self.fusion_origin = None
        self.last_print_sec = -1
        self.pos_errors = []
        self.yaw_errors = []
        self.last_stamp = None
        self.start_stamp = None
        self.out = Path.home() / 'f1tenth_sim/results/fusion/fusion_evaluation.csv'
        self.out.parent.mkdir(parents=True, exist_ok=True)
        self.fp = self.out.open('w', newline='')
        self.csv = csv.writer(self.fp)
        self.csv.writerow(['elapsed_s','position_error_m','heading_error_deg',
                           'true_rel_x','true_rel_y','ekf_rel_x','ekf_rel_y',
                           'match_time_error_ms'])
        self.create_subscription(Odometry, '/ego_racecar/odom', self.on_truth, 20)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_fusion, 20)
        self.get_logger().info(f'Evaluating EKF against truth: {self.out}')

    def on_truth(self, msg):
        t = stamp_ns(msg)
        if self.truth and t < self.truth[-1][0]:
            self.get_logger().warn('Clock reset: evaluator initial reference cleared')
            self.truth.clear()
            self.filtered.clear()
            self.truth_origin = self.fusion_origin = None
        self.truth.append((t, get_pose(msg)))
        self.match()

    def on_fusion(self, msg):
        self.filtered.append((stamp_ns(msg), get_pose(msg)))
        self.match()

    def match(self):
        if len(self.truth) < 2:
            return
        truth = list(self.truth)
        stamps = [item[0] for item in truth]
        while self.filtered and self.filtered[0][0] <= stamps[-1] - 5_000_000:
            t, est = self.filtered.popleft()
            if t < stamps[0]:
                continue
            i = bisect_left(stamps, t)
            options = truth[max(0, i-1):min(len(truth), i+1)]
            matched_t, ref = min(options, key=lambda x: abs(x[0] - t))
            lag = abs(matched_t - t) / 1e9
            if lag > 0.06:
                continue

            if self.truth_origin is None:
                self.truth_origin, self.fusion_origin = ref, est
                self.start_stamp = t
                self.get_logger().info('Initial reference captured; start the racing controller')
                continue

            a = relative(ref, self.truth_origin)
            b = relative(est, self.fusion_origin)
            ep = math.hypot(a[0] - b[0], a[1] - b[1])
            eh = math.degrees(abs(wrap(a[2] - b[2])))
            elapsed = (t - self.start_stamp)/1e9
            self.pos_errors.append(ep)
            self.yaw_errors.append(eh)
            self.csv.writerow([round(elapsed,3),round(ep,5),round(eh,4),
                               round(a[0],4),round(a[1],4),round(b[0],4),round(b[1],4),
                               round(1000*lag,2)])
            sec = int(elapsed)
            if sec >= self.last_print_sec+5:
                self.last_print_sec = sec
                print(f't={elapsed:.1f}s | fusion error={ep:.3f}m | yaw error={eh:.2f}deg | '
                      f'max={max(self.pos_errors):.3f}m', flush=True)
            self.fp.flush()

    def summary(self):
        if not self.pos_errors:
            print('No matched EKF measurements! Check topic /odometry/filtered and TF.')
            return
        import statistics
        a = sorted(self.pos_errors)
        p95 = a[min(len(a)-1, math.ceil(.95*len(a))-1)]
        print('\n=== FUSION EVALUATION ===')
        print(f'Samples:            {len(a)}')
        print(f'Mean position error:{statistics.mean(a):.3f} m')
        print(f'RMSE position:      {math.sqrt(statistics.mean(x*x for x in a)):.3f} m')
        print(f'Max position error: {a[-1]:.3f} m')
        print(f'95th percentile:    {p95:.3f} m')
        print(f'Mean heading error: {statistics.mean(self.yaw_errors):.2f} deg')
        print(f'Max heading error:  {max(self.yaw_errors):.2f} deg')
        print(f'Samples > 0.30 m:  {sum(x > .3 for x in a)}')
        print(f'CSV: {self.out}')


def main():
    rclpy.init()
    node = FusionEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.fp.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
