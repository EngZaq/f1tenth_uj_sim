#!/usr/bin/env python3
"""Raw VESC odometry for the simulation, like the real car's vesc_to_odom (no EKF).

  speed     <- MEASURED wheel speed  /sim/vesc/wheel_odom  (twist.linear.x, from sim_vesc_imu.py)
  yaw rate  <- COMMANDED steering    /drive (AckermannDriveStamped.drive.steering_angle)
               omega = v * tan(delta) / wheelbase   (kinematic bicycle, fusion_base = rear axle)
  output    -> /odom_raw (nav_msgs/Odometry, fusion_odom -> fusion_base)
               TF fusion_odom -> fusion_base (this node must be its ONLY publisher)

Servo / linkage play (real car: 2-5 deg) is mimicked on the steering actually used:
  delta = delta_cmd + steer_bias_deg + n(t)
  n(t)  = first-order Gauss-Markov noise, stationary std steer_noise_deg, correlation time
          steer_noise_tau_s (play is a slowly wandering offset, not white noise).
Both default to 0 (ideal linkage).

Never reads the simulator truth pose. Integration uses the VESC message stamps.
"""
import math

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class SimVescToOdom(Node):
    def __init__(self):
        super().__init__('sim_vesc_to_odom')
        self.declare_parameter('wheelbase_m', 0.3302)
        self.declare_parameter('steer_bias_deg', 0.0)
        self.declare_parameter('steer_noise_deg', 0.0)
        self.declare_parameter('steer_noise_tau_s', 0.5)
        self.declare_parameter('seed', 7)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'fusion_odom')
        self.declare_parameter('base_frame', 'fusion_base')
        # Twist covariance written into /odom_raw (informational; AMCL reads only the TF).
        self.declare_parameter('var_vx', 0.0009)        # (m/s)^2
        self.declare_parameter('var_vyaw', 0.01)        # (rad/s)^2
        p = lambda n: self.get_parameter(n).value
        self.L = float(p('wheelbase_m'))
        self.bias = math.radians(float(p('steer_bias_deg')))
        self.noise_std = math.radians(float(p('steer_noise_deg')))
        self.tau = max(float(p('steer_noise_tau_s')), 1e-3)
        self.rng = np.random.default_rng(int(p('seed')))
        self.publish_tf = bool(p('publish_tf'))
        self.odom_frame, self.base_frame = str(p('odom_frame')), str(p('base_frame'))
        self.var_vx, self.var_vyaw = float(p('var_vx')), float(p('var_vyaw'))

        self.delta_cmd = 0.0
        self.noise = 0.0
        self.x = self.y = self.yaw = 0.0
        self.last_t = None
        self.pub = self.create_publisher(Odometry, '/odom_raw', 20)
        self.tf = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(AckermannDriveStamped, '/drive', self.on_drive, 20)
        self.create_subscription(Odometry, '/sim/vesc/wheel_odom', self.on_vesc, 50)
        self.get_logger().info(
            f'raw VESC odometry: L={self.L} m, steer bias {math.degrees(self.bias):.2f} deg, '
            f'noise {math.degrees(self.noise_std):.2f} deg (tau {self.tau:.2f} s), TF={self.publish_tf}')

    def on_drive(self, msg):
        self.delta_cmd = float(msg.drive.steering_angle)

    def on_vesc(self, msg):
        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        v = float(msg.twist.twist.linear.x)
        if self.last_t is None or t <= self.last_t:
            self.last_t = t if self.last_t is None else self.last_t
            return
        dt = min(t - self.last_t, 0.2)
        self.last_t = t
        if self.noise_std > 0.0:   # exact discretisation of the Gauss-Markov process
            a = math.exp(-dt / self.tau)
            self.noise = a * self.noise + self.noise_std * math.sqrt(1 - a * a) * self.rng.standard_normal()
        delta = self.delta_cmd + self.bias + self.noise
        w = v * math.tan(delta) / self.L
        # midpoint integration of the rear-axle bicycle model
        ym = self.yaw + 0.5 * w * dt
        self.x += v * math.cos(ym) * dt
        self.y += v * math.sin(ym) * dt
        self.yaw = math.atan2(math.sin(self.yaw + w * dt), math.cos(self.yaw + w * dt))

        qz, qw = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        o = Odometry()
        o.header.stamp = msg.header.stamp
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        o.twist.twist.linear.x = v
        o.twist.twist.angular.z = w
        o.twist.covariance[0] = self.var_vx
        o.twist.covariance[35] = self.var_vyaw
        self.pub.publish(o)
        if self.tf is not None:
            tf = TransformStamped()
            tf.header = o.header
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x, tf.transform.translation.y = self.x, self.y
            tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
            self.tf.sendTransform(tf)


def main():
    rclpy.init()
    node = SimVescToOdom()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
