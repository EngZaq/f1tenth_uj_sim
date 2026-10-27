#!/usr/bin/env python3
"""Compare LiDAR SLAM motion against simulator truth.
Ground truth is used ONLY for evaluation, never for localization.
"""

import csv
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformException


def yaw(q):
    return math.atan2(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y * q.y + q.z * q.z)
    )


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def local_motion(pose, initial):
    dx = pose[0] - initial[0]
    dy = pose[1] - initial[1]
    c = math.cos(initial[2])
    s = math.sin(initial[2])
    return (
        c * dx + s * dy,
        -s * dx + c * dy,
        wrap(pose[2] - initial[2])
    )


class Monitor(Node):
    def __init__(self):
        super().__init__('rf2o_accuracy_monitor')

        self.tf_buffer = Buffer()
        self.listener = TransformListener(
            self.tf_buffer, self
        )

        self.truth = None
        self.truth_time = 0.0
        self.truth_initial = None
        self.estimated_initial = None
        self.errors = []
        self.last_print = 0.0

        path = (
            Path.home() / 'f1tenth_sim/results/localization'
            / 'rf2o_compare.csv'
        )

        self.file = path.open('w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'elapsed_s', 'position_error_m',
            'heading_error_deg',
            'true_x', 'true_y',
            'estimated_x', 'estimated_y'
        ])

        self.sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )

        self.start_time = time.monotonic()
        self.timer = self.create_timer(0.2, self.check)
        self.get_logger().info(
            f'Shadow monitor started: {path}'
        )

    def odom_callback(self, msg):
        p = msg.pose.pose
        self.truth = (
            p.position.x,
            p.position.y,
            yaw(p.orientation)
        )
        self.truth_time = time.monotonic()

    def check(self):
        if self.truth is None:
            return

        if time.monotonic() - self.truth_time > 0.5:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                'estimated_odom',
                'estimated_base',
                Time()
            )
        except TransformException:
            return

        tr = tf.transform
        estimated = (
            tr.translation.x,
            tr.translation.y,
            yaw(tr.rotation)
        )

        if self.truth_initial is None:
            self.truth_initial = self.truth
            self.estimated_initial = estimated
            self.get_logger().info(
                'Initial poses captured. Start driving.'
            )
            return

        true_motion = local_motion(
            self.truth, self.truth_initial
        )
        estimated_motion = local_motion(
            estimated, self.estimated_initial
        )

        position_error = math.hypot(
            true_motion[0] - estimated_motion[0],
            true_motion[1] - estimated_motion[1]
        )

        heading_error = math.degrees(abs(wrap(
            true_motion[2] - estimated_motion[2]
        )))

        elapsed = time.monotonic() - self.start_time
        self.errors.append(position_error)

        self.writer.writerow([
            round(elapsed, 3),
            round(position_error, 4),
            round(heading_error, 3),
            *[round(v, 4) for v in self.truth[:2]],
            *[round(v, 4) for v in estimated[:2]]
        ])
        self.file.flush()

        if elapsed - self.last_print >= 2.0:
            self.last_print = elapsed
            print(
                f'Elapsed={elapsed:.1f}s | '
                f'Position error={position_error:.3f}m | '
                f'Heading error={heading_error:.2f}deg | '
                f'Max error={max(self.errors):.3f}m',
                flush=True
            )

    def close(self):
        if self.errors:
            print(
                f'Mean position error: '
                f'{sum(self.errors)/len(self.errors):.3f} m'
            )
            print(
                f'Max position error: '
                f'{max(self.errors):.3f} m'
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
