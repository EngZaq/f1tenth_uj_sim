#!/usr/bin/env python3
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


def stamp(msg):
    return msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec


def yaw(q):
    return math.atan2(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y*q.y + q.z*q.z)
    )


def pose(p):
    return (p.position.x, p.position.y, yaw(p.orientation))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def relative(p, origin):
    dx = p[0] - origin[0]
    dy = p[1] - origin[1]
    c = math.cos(origin[2])
    s = math.sin(origin[2])
    return (
        c*dx + s*dy,
        -s*dx + c*dy,
        wrap(p[2] - origin[2])
    )


class Monitor(Node):
    def __init__(self):
        super().__init__('synchronized_localization_monitor')
        self.truth = deque(maxlen=5000)
        self.pending = {'RF2O': deque(), 'SLAM': deque()}
        self.initial = {}
        self.previous = {}
        self.errors = {'RF2O': [], 'SLAM': []}
        self.last_print = {'RF2O': 0, 'SLAM': 0}
        self.start = time.monotonic()

        path = Path.home() / (
            'f1tenth_sim/results/localization/'
            'synchronized_sources.csv'
        )
        self.file = path.open('w', newline='')
        self.csv = csv.writer(self.file)
        self.csv.writerow([
            'source', 'elapsed_s', 'stamp_ns',
            'position_error_m', 'heading_error_deg',
            'estimated_speed_mps', 'true_speed_mps',
            'match_time_error_ms'
        ])

        self.create_subscription(
            Odometry, '/ego_racecar/odom', self.on_truth, 10
        )
        self.create_subscription(
            Odometry, '/estimated/rf2o_odom',
            lambda m: self.on_estimate('RF2O', m.header, m.pose.pose),
            10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/pose',
            lambda m: self.on_estimate('SLAM', m.header, m.pose.pose),
            10
        )

        print(f'Synchronized monitor ready: {path}', flush=True)

    def on_truth(self, msg):
        t = stamp(msg)
        if self.truth and t < self.truth[-1][0]:
            self.truth.clear()
            self.initial.clear()
            self.previous.clear()
            print('WARNING: Ground-truth clock moved backwards')

        self.truth.append((t, pose(msg.pose.pose)))
        self.process()

    def on_estimate(self, name, header, p):
        t = header.stamp.sec * 1000000000 + header.stamp.nanosec
        self.pending[name].append((t, pose(p)))
        self.process()

    def process(self):
        if len(self.truth) < 2:
            return

        gt = list(self.truth)
        times = [x[0] for x in gt]

        for name, queue in self.pending.items():
            while queue and queue[0][0] <= times[-1] - 10000000:
                t, est = queue.popleft()

                if t < times[0]:
                    continue

                i = bisect_left(times, t)
                candidates = gt[max(0, i-1):min(len(gt), i+1)]
                matched_t, actual = min(
                    candidates, key=lambda x: abs(x[0]-t)
                )

                mismatch = abs(matched_t-t) / 1e9
                if mismatch > 0.06:
                    continue

                if name not in self.initial:
                    self.initial[name] = (actual, est)
                    print(f'{name}: initial reference captured', flush=True)
                    continue

                actual0, est0 = self.initial[name]
                a = relative(actual, actual0)
                b = relative(est, est0)

                error = math.hypot(a[0]-b[0], a[1]-b[1])
                heading = math.degrees(abs(wrap(a[2]-b[2])))

                est_speed = 0.0
                true_speed = 0.0

                if name in self.previous:
                    old_t, old_est, old_actual = self.previous[name]
                    dt = (t-old_t) / 1e9
                    if dt > 0:
                        est_speed = math.hypot(
                            est[0]-old_est[0],
                            est[1]-old_est[1]
                        ) / dt
                        true_speed = math.hypot(
                            actual[0]-old_actual[0],
                            actual[1]-old_actual[1]
                        ) / dt

                        if dt < 0.5 and est_speed > 3.0:
                            print(
                                f'JUMP {name}: '
                                f'est={est_speed:.2f}m/s '
                                f'true={true_speed:.2f}m/s',
                                flush=True
                            )

                self.previous[name] = (t, est, actual)
                self.errors[name].append(error)
                elapsed = time.monotonic() - self.start

                self.csv.writerow([
                    name, round(elapsed, 3), t,
                    round(error, 4),
                    round(heading, 3),
                    round(est_speed, 3),
                    round(true_speed, 3),
                    round(mismatch*1000, 2)
                ])

                if elapsed - self.last_print[name] >= 5:
                    self.last_print[name] = elapsed
                    print(
                        f'{name} | t={elapsed:.1f}s | '
                        f'error={error:.3f}m | '
                        f'heading={heading:.2f}deg',
                        flush=True
                    )

        self.file.flush()

    def close(self):
        for name, errors in self.errors.items():
            if errors:
                print(
                    f'{name}: samples={len(errors)}, '
                    f'mean={sum(errors)/len(errors):.3f}m, '
                    f'max={max(errors):.3f}m'
                )
        self.file.close()


def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
