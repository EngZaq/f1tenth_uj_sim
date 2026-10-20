#!/usr/bin/env python3

import csv
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped


class PurePursuit(Node):

    def __init__(self):
        super().__init__('racing_pure_pursuit')

        self.declare_parameter(
            'waypoints_file',
            '~/f1tenth_sim/reference/levine/levine_centerline_safe.csv'
        )

        self.declare_parameter(
            'results_file',
            '~/f1tenth_sim/results/racing_profile_metrics_v2.csv'
        )

        self.declare_parameter('lookahead_distance', 1.0)
        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('target_speed', 5.0)
        self.declare_parameter('max_steering_angle', 0.4189)

        self.declare_parameter(
            'use_speed_profile',
            True
        )

        self.declare_parameter(
            'speed_profile_file',
            '~/f1tenth_sim/reference/levine/levine_speed_profile.csv'
        )

        waypoint_file = os.path.expanduser(
            self.get_parameter('waypoints_file').value
        )

        self.results_file = os.path.expanduser(
            self.get_parameter('results_file').value
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

        self.use_speed_profile = self.get_parameter(
            'use_speed_profile'
        ).value

        speed_profile_file = os.path.expanduser(
            self.get_parameter(
                'speed_profile_file'
            ).value
        )

        self.waypoints = self.load_waypoints(
            waypoint_file
        )

        self.number_of_waypoints = len(
            self.waypoints
        )

        if self.use_speed_profile:

            self.speed_profile = (
                self.load_speed_profile(
                    speed_profile_file
                )
            )

            if (
                len(self.speed_profile) !=
                self.number_of_waypoints
            ):
                raise RuntimeError(
                    'Speed profile and path '
                    'have different lengths'
                )

        else:

            self.speed_profile = None

        self.current_target_speed = (
            self.target_speed
        )

        # ----------------------------
        # Controller state
        # ----------------------------

        self.latest_steering = 0.0

        self.car_x = 0.0
        self.car_y = 0.0

        self.have_odom = False
        self.collision = False

        self.debug_counter = 0

        # ----------------------------
        # Lap tracking
        # ----------------------------

        self.previous_nearest_index = None

        self.progress_points = 0

        self.lap_number = 0

        self.lap_start_time = None

        # ----------------------------
        # Cross-track error statistics
        # ----------------------------

        self.cte_sum = 0.0
        self.cte_count = 0
        self.cte_max = 0.0

        self.latest_cte = 0.0
        self.lap_speed_samples = []

        self.prepare_results_file()

        # ----------------------------
        # ROS interfaces
        # ----------------------------

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

        # 20 Hz controller output
        self.control_timer = self.create_timer(
            0.05,
            self.control_callback
        )

        self.get_logger().info(
            f'Loaded {self.number_of_waypoints} waypoints'
        )

        self.get_logger().info(
            f'Lookahead={self.lookahead_distance:.2f} m | '
            f'Speed={self.target_speed:.2f} m/s | '
            f'Wheelbase={self.wheelbase:.4f} m'
        )

        self.get_logger().info(
            f'Lap metrics will be saved to: '
            f'{self.results_file}'
        )

    # ============================================================
    # Waypoints
    # ============================================================

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

    # ============================================================
    # Speed profile
    # ============================================================

    def load_speed_profile(
        self,
        filename
    ):

        speeds = []

        with open(
            filename,
            'r',
            newline=''
        ) as file:

            reader = csv.DictReader(file)

            for row in reader:

                speeds.append(
                    float(
                        row['planned_speed_mps']
                    )
                )

        self.get_logger().info(
            f'Loaded speed profile: '
            f'{len(speeds)} points | '
            f'{min(speeds):.2f} -> '
            f'{max(speeds):.2f} m/s'
        )

        return speeds

    # ============================================================
    # Nearest point
    # ============================================================

    def find_nearest_waypoint(
        self,
        car_x,
        car_y
    ):

        nearest_index = 0

        smallest_distance_squared = float('inf')

        for i, waypoint in enumerate(
            self.waypoints
        ):

            dx = waypoint['x'] - car_x
            dy = waypoint['y'] - car_y

            distance_squared = (
                dx * dx +
                dy * dy
            )

            if (
                distance_squared <
                smallest_distance_squared
            ):

                smallest_distance_squared = (
                    distance_squared
                )

                nearest_index = i

        return nearest_index

    # ============================================================
    # Lookahead
    # ============================================================

    def find_lookahead_waypoint(
        self,
        nearest_index
    ):

        nearest_s = self.waypoints[
            nearest_index
        ]['s']

        target_s = (
            nearest_s +
            self.lookahead_distance
        )

        for i in range(
            nearest_index,
            self.number_of_waypoints
        ):

            if (
                self.waypoints[i]['s']
                >= target_s
            ):
                return i

        # Closed-loop wrap-around
        remaining_distance = (
            target_s -
            self.waypoints[-1]['s']
        )

        for i, waypoint in enumerate(
            self.waypoints
        ):

            if (
                waypoint['s']
                >= remaining_distance
            ):
                return i

        return 0

    # ============================================================
    # Quaternion -> yaw
    # ============================================================

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

    # ============================================================
    # Pure Pursuit
    # ============================================================

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

        # Map frame -> vehicle frame
        target_x_local = (
            math.cos(car_yaw) * dx +
            math.sin(car_yaw) * dy
        )

        target_y_local = (
            -math.sin(car_yaw) * dx +
            math.cos(car_yaw) * dy
        )

        actual_lookahead = math.hypot(
            target_x_local,
            target_y_local
        )

        if actual_lookahead < 0.001:
            return 0.0

        curvature = (
            2.0 *
            target_y_local /
            (actual_lookahead ** 2)
        )

        steering = math.atan(
            self.wheelbase *
            curvature
        )

        steering = max(
            -self.max_steering_angle,
            min(
                self.max_steering_angle,
                steering
            )
        )

        return steering

    # ============================================================
    # Cross-track error
    # ============================================================

    def distance_to_segment(
        self,
        px,
        py,
        x1,
        y1,
        x2,
        y2
    ):

        vx = x2 - x1
        vy = y2 - y1

        wx = px - x1
        wy = py - y1

        segment_length_squared = (
            vx * vx +
            vy * vy
        )

        if segment_length_squared < 1e-12:

            return math.hypot(
                wx,
                wy
            )

        t = (
            wx * vx +
            wy * vy
        ) / segment_length_squared

        t = max(
            0.0,
            min(1.0, t)
        )

        projection_x = (
            x1 +
            t * vx
        )

        projection_y = (
            y1 +
            t * vy
        )

        return math.hypot(
            px - projection_x,
            py - projection_y
        )

    def calculate_cross_track_error(
        self,
        car_x,
        car_y
    ):

        minimum_distance = float('inf')

        for i in range(
            self.number_of_waypoints
        ):

            j = (
                i + 1
            ) % self.number_of_waypoints

            p1 = self.waypoints[i]
            p2 = self.waypoints[j]

            distance = self.distance_to_segment(
                car_x,
                car_y,
                p1['x'],
                p1['y'],
                p2['x'],
                p2['y']
            )

            if distance < minimum_distance:

                minimum_distance = distance

        return minimum_distance

    # ============================================================
    # Lap tracking
    # ============================================================

    def update_lap_progress(
        self,
        nearest_index
    ):

        if self.previous_nearest_index is None:

            self.previous_nearest_index = (
                nearest_index
            )

            return

        delta = (
            nearest_index -
            self.previous_nearest_index
        ) % self.number_of_waypoints

        # Ignore backward movement or a bad nearest-point jump.
        if (
            delta > 0 and
            delta < self.number_of_waypoints // 2
        ):

            if self.lap_start_time is None:

                self.lap_start_time = (
                    time.monotonic()
                )

                self.get_logger().info(
                    'Lap timer started'
                )

            self.progress_points += delta

            if (
                self.progress_points >=
                self.number_of_waypoints
            ):

                now = time.monotonic()

                lap_time = (
                    now -
                    self.lap_start_time
                )

                self.lap_number += 1

                self.finish_lap(
                    lap_time
                )

                self.progress_points -= (
                    self.number_of_waypoints
                )

                self.lap_start_time = now

                self.reset_cte_statistics()
                self.lap_speed_samples.clear()

        self.previous_nearest_index = (
            nearest_index
        )

    # ============================================================
    # Metrics
    # ============================================================

    def reset_cte_statistics(self):

        self.cte_sum = 0.0
        self.cte_count = 0
        self.cte_max = 0.0

    def update_cte_statistics(self):

        if self.lap_start_time is None:
            return

        self.latest_cte = (
            self.calculate_cross_track_error(
                self.car_x,
                self.car_y
            )
        )

        self.cte_sum += self.latest_cte

        self.cte_count += 1

        self.cte_max = max(
            self.cte_max,
            self.latest_cte
        )

    def finish_lap(
        self,
        lap_time
    ):

        if self.cte_count > 0:

            mean_cte = (
                self.cte_sum /
                self.cte_count
            )

        else:

            mean_cte = float('nan')

        if self.lap_speed_samples:
            mean_commanded_speed = (
                sum(self.lap_speed_samples) /
                len(self.lap_speed_samples)
            )
            min_commanded_speed = min(
                self.lap_speed_samples
            )
            max_commanded_speed = max(
                self.lap_speed_samples
            )
        else:
            mean_commanded_speed = float('nan')
            min_commanded_speed = float('nan')
            max_commanded_speed = float('nan')

        self.get_logger().info(
            ''
        )

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            f'LAP {self.lap_number} COMPLETE'
        )

        self.get_logger().info(
            f'Lap time: {lap_time:.3f} s'
        )

        self.get_logger().info(
            f'Mean CTE: {mean_cte:.3f} m'
        )

        self.get_logger().info(
            f'Max CTE: {self.cte_max:.3f} m'
        )

        self.get_logger().info(
            f'Configured target speed: '
            f'{self.target_speed:.2f} m/s'
        )

        self.get_logger().info(
            f'Mean commanded speed: '
            f'{mean_commanded_speed:.2f} m/s'
        )

        self.get_logger().info(
            f'Commanded speed range: '
            f'{min_commanded_speed:.2f} - '
            f'{max_commanded_speed:.2f} m/s'
        )

        self.get_logger().info(
            f'Lookahead: '
            f'{self.lookahead_distance:.2f} m'
        )

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            ''
        )

        self.save_lap_result(
            lap_time,
            mean_cte,
            self.cte_max,
            mean_commanded_speed,
            min_commanded_speed,
            max_commanded_speed
        )

    # ============================================================
    # Results CSV
    # ============================================================

    def prepare_results_file(self):

        directory = os.path.dirname(
            self.results_file
        )

        os.makedirs(
            directory,
            exist_ok=True
        )

        if not os.path.exists(
            self.results_file
        ):

            with open(
                self.results_file,
                'w',
                newline=''
            ) as file:

                writer = csv.writer(file)

                writer.writerow([
                    'timestamp',
                    'lap',
                    'configured_target_speed_mps',
                    'mean_commanded_speed_mps',
                    'min_commanded_speed_mps',
                    'max_commanded_speed_mps',
                    'lookahead_m',
                    'lap_time_s',
                    'mean_cte_m',
                    'max_cte_m'
                ])

    def save_lap_result(
        self,
        lap_time,
        mean_cte,
        max_cte,
        mean_commanded_speed,
        min_commanded_speed,
        max_commanded_speed
    ):

        with open(
            self.results_file,
            'a',
            newline=''
        ) as file:

            writer = csv.writer(file)

            writer.writerow([
                datetime.now().isoformat(
                    timespec='seconds'
                ),
                self.lap_number,
                f'{self.target_speed:.3f}',
                f'{mean_commanded_speed:.3f}',
                f'{min_commanded_speed:.3f}',
                f'{max_commanded_speed:.3f}',
                f'{self.lookahead_distance:.3f}',
                f'{lap_time:.3f}',
                f'{mean_cte:.4f}',
                f'{max_cte:.4f}'
            ])

    # ============================================================
    # ROS callbacks
    # ============================================================

    def odom_callback(
        self,
        msg
    ):

        self.car_x = (
            msg.pose.pose.position.x
        )

        self.car_y = (
            msg.pose.pose.position.y
        )

        car_yaw = self.quaternion_to_yaw(
            msg.pose.pose.orientation
        )

        nearest_index = (
            self.find_nearest_waypoint(
                self.car_x,
                self.car_y
            )
        )

        if self.use_speed_profile:

            self.current_target_speed = (
                self.speed_profile[
                    nearest_index
                ]
            )

        else:

            self.current_target_speed = (
                self.target_speed
            )

        self.update_lap_progress(
            nearest_index
        )

        lookahead_index = (
            self.find_lookahead_waypoint(
                nearest_index
            )
        )

        target = self.waypoints[
            lookahead_index
        ]

        self.latest_steering = (
            self.calculate_steering(
                self.car_x,
                self.car_y,
                car_yaw,
                target['x'],
                target['y']
            )
        )

        self.have_odom = True

        self.debug_counter += 1

        if self.debug_counter >= 100:

            self.debug_counter = 0

            self.get_logger().info(
                f'Car=({self.car_x:.2f}, '
                f'{self.car_y:.2f}) | '
                f'Yaw={math.degrees(car_yaw):.1f} deg | '
                f'Nearest={nearest_index} | '
                f'Lookahead={lookahead_index} | '
                f'Steering='
                f'{math.degrees(self.latest_steering):.1f} deg | '
                f'Speed={self.current_target_speed:.2f} m/s | '
                f'CTE={self.latest_cte:.3f} m'
            )

    def collision_callback(
        self,
        msg
    ):

        new_collision = msg.data

        if (
            new_collision and
            not self.collision
        ):

            self.get_logger().error(
                'Collision detected - stopping vehicle'
            )

        elif (
            not new_collision and
            self.collision
        ):

            self.get_logger().info(
                'Collision cleared'
            )

        self.collision = new_collision

    def control_callback(self):

        if not self.have_odom:
            return

        if not self.collision:

            self.update_cte_statistics()

        msg = AckermannDriveStamped()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        if self.collision:

            msg.drive.speed = 0.0

            msg.drive.steering_angle = 0.0

        else:

            msg.drive.speed = (
                self.current_target_speed
            )

            msg.drive.steering_angle = (
                self.latest_steering
            )

        self.drive_pub.publish(
            msg
        )

        # Sample the speed actually sent to /drive while a lap is active.
        if self.lap_start_time is not None:
            self.lap_speed_samples.append(
                float(msg.drive.speed)
            )

    # ============================================================
    # Safe shutdown
    # ============================================================

    def stop_vehicle(self):

        msg = AckermannDriveStamped()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0

        self.drive_pub.publish(
            msg
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = PurePursuit()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        for _ in range(5):

            node.stop_vehicle()

            time.sleep(
                0.02
            )

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
