#!/usr/bin/env python3

from copy import copy
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class ScanRelay(Node):
    def __init__(self):
        super().__init__('sim_estimated_scan_relay')

        self.pub = self.create_publisher(
            LaserScan, '/estimated/scan',
            qos_profile_sensor_data
        )

        self.sub = self.create_subscription(
            LaserScan, '/scan',
            self.callback,
            qos_profile_sensor_data
        )

        self.tf_pub = StaticTransformBroadcaster(self)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'estimated_base'
        transform.child_frame_id = 'estimated_laser'
        transform.transform.translation.x = 0.275
        transform.transform.rotation.w = 1.0

        self.tf_pub.sendTransform(transform)
        self.get_logger().info('Independent estimated scan ready')

    def callback(self, msg):
        scan = copy(msg)
        scan.header = copy(msg.header)
        scan.header.frame_id = 'estimated_laser'
        self.pub.publish(scan)


def main():
    rclpy.init()
    node = ScanRelay()
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
