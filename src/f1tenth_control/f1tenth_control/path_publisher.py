#!/usr/bin/env python3

import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher')

        self.declare_parameter(
            'waypoints_file',
            '~/f1tenth_sim/reference/levine/levine_centerline_safe.csv'
        )

        filename = self.get_parameter(
            'waypoints_file'
        ).get_parameter_value().string_value

        filename = os.path.expanduser(filename)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.path_pub = self.create_publisher(
            Path,
            '/reference_path',
            qos
        )

        self.path = self.load_path(filename)

        self.timer = self.create_timer(
            0.5,
            self.publish_path
        )

        self.get_logger().info(
            f'Loaded {len(self.path.poses)} waypoints from {filename}'
        )

    def load_path(self, filename):

        path = Path()
        path.header.frame_id = 'map'

        with open(filename, 'r') as file:

            for line in file:

                line = line.strip()

                if not line or line.startswith('#'):
                    continue

                values = line.split(';')

                x = float(values[1])
                y = float(values[2])
                yaw = float(values[3])

                pose = PoseStamped()

                pose.header.frame_id = 'map'

                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0

                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)

                path.poses.append(pose)

        return path

    def publish_path(self):

        timestamp = self.get_clock().now().to_msg()

        self.path.header.stamp = timestamp

        for pose in self.path.poses:
            pose.header.stamp = timestamp

        self.path_pub.publish(self.path)

        self.get_logger().info(
            f'Published reference path with {len(self.path.poses)} waypoints'
        )

        self.timer.cancel()


def main(args=None):

    rclpy.init(args=args)

    node = PathPublisher()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
