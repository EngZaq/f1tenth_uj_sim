#!/usr/bin/env python3
"""Copy simulator LaserScans to an independently named fusion-base laser frame.

Does not subscribe to truth. Sensor extrinsics: fusion_base->fusion_laser x=0.275m.
"""
import copy
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class Relay(Node):
    def __init__(self):
        super().__init__('fusion_scan_relay')
        self.last = 0.0
        self.pub = self.create_publisher(LaserScan, '/fusion/scan', qos_profile_sensor_data)
        self.sub = self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.tf_pub = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'fusion_base'
        tf.child_frame_id = 'fusion_laser'
        tf.transform.translation.x = 0.275
        tf.transform.rotation.w = 1.0
        self.tf_pub.sendTransform(tf)
        self.get_logger().info('Independent /fusion/scan at <=10Hz; fusion_base->fusion_laser x=0.275m')

    def on_scan(self, msg):
        now = time.monotonic()
        if now - self.last < 0.1:
            return
        self.last = now
        scan = copy.copy(msg)
        scan.header = copy.copy(msg.header)
        scan.header.frame_id = 'fusion_laser'
        self.pub.publish(scan)

def main():
    rclpy.init()
    node = Relay()
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
