#!/usr/bin/env python3

import math
import os
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped


class PurePursuit(Node):

    def __init__(self):
        super().__init__('pure_pursuit')

        self.declare_parameter(
            'waypoints_file',
            '~/f1tenth_sim/reference/levine/levine_centerline_original.csv'
        )

        self.declare_parameter('lookahead_distance', 1.0)
        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('target_speed', 0.30)
        self.declare_parameter('max_steering_angle', 0.4189)

        filename = os.path.expanduser(
            self.get_parameter('waypoints_file').value
        )

        self.lookahead_distance = self.get_parameter(
            'lookahead_distance'
        ).value

        self.wheelbase = self.get_parameter(
            'wheelbase'
        ).value

        self.target_speed = self.get_parameter(
            'target_speed'
        ).value

        self.max_steering_angle = self.get_parameter(
            'max_steering_angle'
        ).value

        self.waypoints = self.load_waypoints(filename)

        self.latest_steering = 0.0
        self.have_odom = False
        self.collision = False

        self.debug_counter = 0

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )

        self.collision_sub = self.create_subscription(
            Bool,
            '/ego_racecar/collision',
            self.collision_callback,
            10
        )

        # Send commands at 20 Hz.
        self.control_timer = self.create_timer(
            0.05,
            self.control_callback
        )

        self.get_logger().info(
            f'Loaded {len(self.waypoints)} waypoints'
        )

        self.get_logger().info(
            f'Lookahead={self.lookahead_distance:.2f} m | '
            f'Speed={self.target_speed:.2f} m/s | '
            f'Wheelbase={self.wheelbase:.4f} m'
        )

    def load_waypoints(self, filename):

        waypoints = []

        with open(filename, 'r') as file:

            for line in file:

                line = line.strip()

                if not line or line.startswith('#'):
                    continue

                values = line.split(';')

                waypoints.append({
                    's': float(values[0]),
                    'x': float(values[1]),
                    'y': float(values[2]),
                    'yaw': float(values[3]),
                    'curvature': float(values[4]),
                    'velocity': float(values[5]),
                    'acceleration': float(values[6])
                })

        return waypoints

    def find_nearest_waypoint(self, car_x, car_y):

        nearest_index = 0
        smallest_distance_squared = float('inf')

        for i, waypoint in enumerate(self.waypoints):

            dx = waypoint['x'] - car_x
            dy = waypoint['y'] - car_y

            distance_squared = dx * dx + dy * dy

            if distance_squared < smallest_distance_squared:
                smallest_distance_squared = distance_squared
                nearest_index = i

        return nearest_index

    def find_lookahead_waypoint(self, nearest_index):

        nearest_s = self.waypoints[nearest_index]['s']
        target_s = nearest_s + self.lookahead_distance

        for i in range(nearest_index, len(self.waypoints)):

            if self.waypoints[i]['s'] >= target_s:
                return i

        remaining_distance = (
            target_s - self.waypoints[-1]['s']
        )

        for i, waypoint in enumerate(self.waypoints):

            if waypoint['s'] >= remaining_distance:
                return i

        return 0

    def quaternion_to_yaw(self, q):

        siny_cosp = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )

    def calculate_steering(
        self,
        car_x,
        car_y,
        car_yaw,
        target_x,
        target_y
    ):

        dx = target_x - car_x
        dy = target_y - car_y

        target_x_local = (
            math.cos(car_yaw) * dx +
            math.sin(car_yaw) * dy
        )

        target_y_local = (
            -math.sin(car_yaw) * dx +
            math.cos(car_yaw) * dy
        )

        lookahead_distance = math.sqrt(
            target_x_local ** 2 +
            target_y_local ** 2
        )

        if lookahead_distance < 0.001:
            return 0.0

        curvature = (
            2.0 * target_y_local /
            (lookahead_distance ** 2)
        )

        steering = math.atan(
            self.wheelbase * curvature
        )

        steering = max(
            -self.max_steering_angle,
            min(self.max_steering_angle, steering)
        )

        return steering

    def odom_callback(self, msg):

        car_x = msg.pose.pose.position.x
        car_y = msg.pose.pose.position.y

        car_yaw = self.quaternion_to_yaw(
            msg.pose.pose.orientation
        )

        nearest_index = self.find_nearest_waypoint(
            car_x,
            car_y
        )

        lookahead_index = self.find_lookahead_waypoint(
            nearest_index
        )

        target = self.waypoints[lookahead_index]

        self.latest_steering = self.calculate_steering(
            car_x,
            car_y,
            car_yaw,
            target['x'],
            target['y']
        )

        self.have_odom = True

        self.debug_counter += 1

        if self.debug_counter >= 100:

            self.debug_counter = 0

            self.get_logger().info(
                f'Car=({car_x:.2f}, {car_y:.2f}) | '
                f'Yaw={math.degrees(car_yaw):.1f} deg | '
                f'Nearest={nearest_index} | '
                f'Lookahead={lookahead_index} | '
                f'Steering={math.degrees(self.latest_steering):.1f} deg'
            )

    def collision_callback(self, msg):

        self.collision = msg.data

        if self.collision:
            self.get_logger().error(
                'Collision detected - stopping vehicle'
            )

    def control_callback(self):

        if not self.have_odom:
            return

        msg = AckermannDriveStamped()

        msg.header.stamp = self.get_clock().now().to_msg()

        if self.collision:

            msg.drive.speed = 0.0
            msg.drive.steering_angle = 0.0

        else:

            msg.drive.speed = self.target_speed
            msg.drive.steering_angle = self.latest_steering

        self.drive_pub.publish(msg)

    def stop_vehicle(self):

        msg = AckermannDriveStamped()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0

        self.drive_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = PurePursuit()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        # Publish several stop commands before shutting down.
        for _ in range(5):
            node.stop_vehicle()
            time.sleep(0.02)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
