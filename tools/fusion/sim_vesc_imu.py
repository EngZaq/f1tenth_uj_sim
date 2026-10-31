#!/usr/bin/env python3
"""Simulation-only adapter: truth motion -> noisy VESC-like ERPM and IMU.

Only this adapter is allowed to subscribe to simulator truth. EKF consumers
receive *velocity measurements* and yaw-rate measurements, never truth pose.
"""
import math
import random

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


def time_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class SimVescImu(Node):
    def __init__(self):
        super().__init__('sim_vesc_imu')
        self.declare_parameter('seed', 42)
        self.declare_parameter('wheel_diameter_m', 0.1016)
        self.declare_parameter('gear_ratio', 4.092)
        self.declare_parameter('motor_pole_pairs', 7.0)
        self.declare_parameter('vesc_hz', 50.0)
        self.declare_parameter('imu_hz', 100.0)
        self.declare_parameter('vesc_speed_scale', 1.005)
        self.declare_parameter('vesc_erpm_noise_std', 35.0)
        self.declare_parameter('gyro_bias_rad_s', 0.001)
        self.declare_parameter('gyro_noise_std_rad_s', 0.01)

        param = lambda key: self.get_parameter(key).value
        diameter = float(param('wheel_diameter_m'))
        ratio = float(param('gear_ratio'))
        poles = float(param('motor_pole_pairs'))
        self.vesc_hz = float(param('vesc_hz'))
        self.imu_hz = float(param('imu_hz'))
        if min(diameter, ratio, poles, self.vesc_hz, self.imu_hz) <= 0:
            raise ValueError('Wheel geometry, gearing and rates must be positive')

        # Electrical RPM: motor mechanical RPM multiplied by motor pole pairs.
        # Linear speed (m/s) -> wheel rev/s -> motor rev/s -> electrical RPM.
        self.erpm_per_mps = 60.0 * ratio * poles / (math.pi * diameter)
        self.vesc_speed_scale = float(param('vesc_speed_scale'))
        self.erpm_sigma = float(param('vesc_erpm_noise_std'))
        self.gyro_bias = float(param('gyro_bias_rad_s'))
        self.gyro_sigma = float(param('gyro_noise_std_rad_s'))
        self.rng = random.Random(int(param('seed')))

        self.last_vesc_ns = None
        self.last_imu_ns = None
        self.erpm_pub = self.create_publisher(Float64, '/sim/vesc/erpm', 10)
        self.wheel_pub = self.create_publisher(Odometry, '/sim/vesc/wheel_odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/sim/imu/data_raw', 20)
        self.truth_sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.on_truth, 20)
        self.get_logger().info(
            f'Simulation VESC+IMU: 1 m/s={self.erpm_per_mps:.1f} ERPM; '
            'only velocities are emitted, no ground-truth pose')

    def on_truth(self, truth):
        stamp = truth.header.stamp
        now_ns = time_ns(stamp)
        # ST publishes the CoG twist. Translating along body x to the rear
        # axle leaves vx and yaw rate unchanged (only lateral velocity changes).
        # fusion_base is explicitly the rear axle, not a renamed CoG frame.
        vx = float(truth.twist.twist.linear.x)
        wz = float(truth.twist.twist.angular.z)  # Simulator body yaw rate

        if self.last_vesc_ns is not None and now_ns < self.last_vesc_ns:
            self.last_vesc_ns = None
            self.last_imu_ns = None
            self.get_logger().warn('Clock reset detected: resetting sensor rate gates')

        if self.last_vesc_ns is None or now_ns - self.last_vesc_ns >= 1e9 / self.vesc_hz:
            self.last_vesc_ns = now_ns
            measured_erpm = (vx * self.erpm_per_mps * self.vesc_speed_scale
                             + self.rng.gauss(0, self.erpm_sigma))
            self.erpm_pub.publish(Float64(data=float(measured_erpm)))

            # This adapter stands in for the future real VESC telemetry converter.
            # Odometry twist is populated; position is deliberately NOT copied.
            wheel = Odometry()
            wheel.header.stamp = stamp
            wheel.header.frame_id = 'fusion_odom'
            wheel.child_frame_id = 'fusion_base'
            wheel.pose.pose.orientation.w = 1.0
            for i in (0, 7, 14, 21, 28, 35):
                wheel.pose.covariance[i] = 1e6  # Pose unavailable / unused by EKF
            wheel.twist.twist.linear.x = measured_erpm / self.erpm_per_mps
            wheel.twist.covariance[0] = max(
                (self.erpm_sigma / self.erpm_per_mps) ** 2, 0.02 ** 2)
            wheel.twist.covariance[7] = 1.0  # Other channels not fused
            wheel.twist.covariance[35] = 1.0
            self.wheel_pub.publish(wheel)

        if self.last_imu_ns is None or now_ns - self.last_imu_ns >= 1e9 / self.imu_hz:
            self.last_imu_ns = now_ns
            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = 'fusion_base'  # Identity orientation for this sim-only device
            imu.orientation.w = 1.0
            imu.orientation_covariance[0] = -1.0  # Orientation NOT available
            imu.linear_acceleration_covariance[0] = -1.0  # Acceleration NOT modeled
            imu.angular_velocity.z = wz + self.gyro_bias + self.rng.gauss(0, self.gyro_sigma)
            imu.angular_velocity_covariance[0] = 1.0
            imu.angular_velocity_covariance[4] = 1.0
            imu.angular_velocity_covariance[8] = max(self.gyro_sigma ** 2, 1e-4)
            self.imu_pub.publish(imu)


def main():
    rclpy.init()
    node = SimVescImu()
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
