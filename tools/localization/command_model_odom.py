#!/usr/bin/env python3
"""Simulation-only Ackermann dead reckoning; no ground-truth subscription."""

import math
import time

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class ModelOdom(Node):
    def __init__(self):
        super().__init__('command_model_odom')

        self.wheelbase = 0.3302
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.speed = 0.0
        self.steering = 0.0

        self.last_command = None
        self.last_tick = time.monotonic()

        self.sub = self.create_subscription(
            AckermannDriveStamped, '/drive', self.on_drive, 10
        )

        self.pub = self.create_publisher(
            Odometry, '/estimated/model_odom', 10
        )

        self.tf_pub = TransformBroadcaster(self)
        self.timer = self.create_timer(0.02, self.tick)

        self.get_logger().info(
            'Simulation model odometry ready (50 Hz)'
        )

    def on_drive(self, msg):
        self.speed = float(msg.drive.speed)
        self.steering = float(msg.drive.steering_angle)
        self.last_command = time.monotonic()

    def tick(self):
        now = time.monotonic()
        dt = min(max(now - self.last_tick, 0.0), 0.1)
        self.last_tick = now

        # Stop integrating if driving commands become stale.
        v = (
            self.speed
            if self.last_command is not None
            and now - self.last_command < 0.30
            else 0.0
        )

        delta = max(-0.4189, min(0.4189, self.steering))
        yaw_rate = v * math.tan(delta) / self.wheelbase

        mid_yaw = self.yaw + 0.5 * yaw_rate * dt
        self.x += v * math.cos(mid_yaw) * dt
        self.y += v * math.sin(mid_yaw) * dt
        self.yaw = math.atan2(
            math.sin(self.yaw + yaw_rate * dt),
            math.cos(self.yaw + yaw_rate * dt)
        )

        stamp = self.get_clock().now().to_msg()
        qz = math.sin(self.yaw / 2)
        qw = math.cos(self.yaw / 2)

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'estimated_odom'
        msg.child_frame_id = 'estimated_base'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = yaw_rate
        self.pub.publish(msg)

        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = 'estimated_base'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_pub.sendTransform(transform)


def main():
    rclpy.init()
    node = ModelOdom()
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
