#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanLimiter(Node):
    def __init__(self):
        super().__init__('slam_scan_rate_limiter')

        self.last_publish = 0.0

        self.publisher = self.create_publisher(
            LaserScan,
            '/estimated/slam_scan',
            qos_profile_sensor_data
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/estimated/scan',
            self.callback,
            qos_profile_sensor_data
        )

        self.get_logger().info('SLAM scan limiter ready: 10 Hz')

    def callback(self, msg):
        now = time.monotonic()

        if now - self.last_publish < 0.1:
            return

        self.last_publish = now
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = ScanLimiter()
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
