#!/usr/bin/env python3
"""Copy simulator LaserScans to an independently named fusion-base laser frame.

Does not subscribe to truth pose. Validates the shared laser calibration
against the simulator's body-to-laser TF before relaying any scans.
"""
import copy
import time
import math
import sys
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sim_geometry import LASER, validate_reference_transform

class Relay(Node):
    def __init__(self):
        super().__init__('fusion_scan_relay')
        self.last = 0.0
        self.pub = self.create_publisher(LaserScan, '/fusion/scan', qos_profile_sensor_data)
        self.sub = self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.tf_pub = StaticTransformBroadcaster(self)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.verified = False
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'fusion_base'
        tf.child_frame_id = 'fusion_laser'
        tf.transform.translation.x = LASER['x_m']
        tf.transform.translation.y = LASER['y_m']
        tf.transform.rotation.z = math.sin(LASER['yaw_rad']/2)
        tf.transform.rotation.w = math.cos(LASER['yaw_rad']/2)
        self.laser_transform = tf
        self.get_logger().info(f'Waiting to verify laser calibration: {LASER}')

    def on_scan(self, msg):
        if not self.verified:
            try:
                actual = self.buffer.lookup_transform(LASER['reference_body_frame'],
                                                      msg.header.frame_id, Time()).transform
            except TransformException:
                return
            validate_reference_transform(actual)
            self.tf_pub.sendTransform(self.laser_transform)
            self.verified = True
            self.get_logger().info('LASER GEOMETRY VERIFIED; /fusion/scan enabled')
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
