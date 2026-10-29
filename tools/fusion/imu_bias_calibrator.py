#!/usr/bin/env python3
"""Stationary gyroscope Z bias calibration for the F1TENTH simulator.

Uses wheel speed only to gate calibration; never reads simulator truth.
Publishes corrected IMU messages after N stationary samples.
"""
import copy
import math
import statistics

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class ImuBiasCalibrator(Node):
    def __init__(self):
        super().__init__('imu_bias_calibrator')
        self.declare_parameter('samples_required', 1000)
        self.declare_parameter('max_stationary_speed_mps', 0.05)
        self.required = int(self.get_parameter('samples_required').value)
        self.speed_threshold = float(self.get_parameter('max_stationary_speed_mps').value)
        if self.required < 10 or self.speed_threshold <= 0:
            raise ValueError('Invalid calibration parameters')
        self.last_speed = None
        self.samples = []
        self.bias = None
        self.pub = self.create_publisher(Imu, '/sim/imu/calibrated', 20)
        self.create_subscription(Odometry, '/sim/vesc/wheel_odom', self.on_wheel, 20)
        self.create_subscription(Imu, '/sim/imu/data_raw', self.on_imu, 50)
        self.get_logger().info(
            f'Keep car stationary for {self.required} IMU samples; '
            'waiting for wheel speed and raw IMU')

    def on_wheel(self, msg):
        self.last_speed = float(msg.twist.twist.linear.x)

    def on_imu(self, msg):
        if self.bias is None:
            if self.last_speed is None or not math.isfinite(self.last_speed):
                return
            if abs(self.last_speed) > self.speed_threshold:
                if self.samples:
                    self.samples.clear()
                    self.get_logger().warn('Movement detected during calibration; restarting')
                return
            z = float(msg.angular_velocity.z)
            if not math.isfinite(z):
                return
            self.samples.append(z)
            if len(self.samples) < self.required:
                return
            self.bias = statistics.fmean(self.samples)
            sigma = statistics.pstdev(self.samples)
            self.get_logger().info(
                f'CALIBRATED gyro_z bias={self.bias:+.6f} rad/s '
                f'(sample std={sigma:.5f}; n={len(self.samples)}). '
                'Start racing now.')

        corrected = copy.deepcopy(msg)
        corrected.angular_velocity.z -= self.bias
        self.pub.publish(corrected)


def main():
    rclpy.init()
    node = ImuBiasCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
