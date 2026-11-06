#!/usr/bin/env python3
"""cov_relay: write measurement noise R into an Odometry message's twist covariance.

rf2o (and vesc_to_odom on the real car) publish no twist covariance. robot_localization
reads R from the message itself and treats zero as "almost perfect" (clamped to 1e-9).
This node copies every message, writes the configured variances, and republishes it.

twist.covariance is a 6x6 matrix stored row by row (36 values):
  index 0 = vx, 7 = vy, 14 = vz, 21 = vroll, 28 = vpitch, 35 = vyaw
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class CovRelay(Node):
    def __init__(self):
        super().__init__('cov_relay')
        self.declare_parameter('input_topic', '/fusion/rf2o/odom')
        self.declare_parameter('output_topic', '/fusion/rf2o/odom_cov')
        self.declare_parameter('var_vx', 0.01)       # (m/s)^2    -> R for vx
        self.declare_parameter('var_vy', 1.0)        # not fused unless _config says so
        self.declare_parameter('var_vyaw', 0.0025)   # (rad/s)^2  -> R for vyaw
        self.declare_parameter('var_unused', 1.0)    # vz, vroll, vpitch (never fused in 2D)
        self.declare_parameter('pose_var', 1000000.0)    # pose is not fused: mark it "unknown"

        p = lambda name: self.get_parameter(name).value
        for name in ('var_vx', 'var_vy', 'var_vyaw', 'var_unused', 'pose_var'):
            if float(p(name)) <= 0.0:
                raise ValueError(f'{name} must be > 0 (zero means "perfect sensor")')

        self.twist_cov = [0.0] * 36
        self.twist_cov[0] = float(p('var_vx'))
        self.twist_cov[7] = float(p('var_vy'))
        self.twist_cov[14] = float(p('var_unused'))
        self.twist_cov[21] = float(p('var_unused'))
        self.twist_cov[28] = float(p('var_unused'))
        self.twist_cov[35] = float(p('var_vyaw'))

        self.pose_cov = [0.0] * 36
        for i in (0, 7, 14, 21, 28, 35):
            self.pose_cov[i] = float(p('pose_var'))

        self.count = 0
        self.pub = self.create_publisher(Odometry, p('output_topic'), 10)
        self.create_subscription(Odometry, p('input_topic'), self.on_odom, 10)
        self.get_logger().info(
            f"{p('input_topic')} -> {p('output_topic')} | "
            f"R: var_vx={self.twist_cov[0]:g}, var_vyaw={self.twist_cov[35]:g}")

    def on_odom(self, msg):
        msg.twist.covariance = list(self.twist_cov)
        msg.pose.covariance = list(self.pose_cov)
        self.pub.publish(msg)
        self.count += 1
        if self.count == 1:
            self.get_logger().info('First message relayed')


def main():
    rclpy.init()
    node = CovRelay()
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
