#!/usr/bin/env python3
"""F1TENTH SIMULATION-ONLY closed-loop AMCL/EKF Pure Pursuit.

Produces real /drive Ackermann messages with no simulator truth used in steering.
Fails safe on disabled, stale localization/sensor data, position uncertainty,
excessive estimated cross-track error, unexpected global pose jump, collision,
and competing drive publishers.  Default mode: disabled; explicit enable required.

DO NOT deploy this unchanged to a physical vehicle: it does not actuate a real
servo/VESC, implement hardware e-stop, or guarantee physical obstacle avoidance.
"""
from __future__ import annotations

import csv
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener, TransformException


def get_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def stamp_s(stamp):
    return float(stamp.sec) + 1e-9 * stamp.nanosec


def dist_to_segment(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    den = dx*dx + dy*dy
    if den < 1e-12:
        return math.hypot(px-ax, py-ay)
    t = min(1.0, max(0.0, ((px-ax)*dx+(py-ay)*dy)/den))
    return math.hypot(px-ax-t*dx, py-ay-t*dy)


class AutonomousDrive(Node):
    def __init__(self):
        super().__init__('amcl_autonomous_drive')
        self.declare_parameter('waypoints_file', str(Path.home() / 'f1tenth_sim/reference/generated/uj_technical_slam_01_centerline.csv'))
        self.declare_parameter('results_file', str(Path.home() / 'f1tenth_sim/results/localization/amcl_autonomous_drive.csv'))
        self.declare_parameter('lookahead_distance', 0.8)
        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('speed_cap_mps', 0.35)
        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('max_pose_age_s', 0.65)
        self.declare_parameter('max_scan_age_s', 0.5)
        self.declare_parameter('max_odom_age_s', 0.5)
        self.declare_parameter('max_path_cte_m', 0.33)
        self.declare_parameter('max_position_std_m', 0.70)
        self.declare_parameter('max_heading_std_deg', 30.0)
        self.declare_parameter('front_stop_distance_m', 0.4)
        self.declare_parameter('front_half_angle_deg', 17.0)
        self.declare_parameter('auto_stop_laps', 1)
        self.declare_parameter('start_x', -5.8592)
        self.declare_parameter('start_y', -2.1720)
        self.declare_parameter('start_yaw_rad', 2.20775)
        self.declare_parameter('start_position_tolerance_m', 0.7)
        self.declare_parameter('start_heading_tolerance_deg', 45.0)
        self.declare_parameter('max_amcl_jump_m', 0.75)
        p = lambda k: self.get_parameter(k).value
        self.lookahead = float(p('lookahead_distance'))
        self.wheelbase = float(p('wheelbase'))
        self.speed_cap = float(p('speed_cap_mps'))
        self.max_steer = float(p('max_steering_angle'))
        self.max_pose_age = float(p('max_pose_age_s'))
        self.max_scan_age = float(p('max_scan_age_s'))
        self.max_odom_age = float(p('max_odom_age_s'))
        self.max_cte = float(p('max_path_cte_m'))
        self.max_pos_std = float(p('max_position_std_m'))
        self.max_yaw_std = math.radians(float(p('max_heading_std_deg')))
        self.front_stop = float(p('front_stop_distance_m'))
        self.front_fov = math.radians(float(p('front_half_angle_deg')))
        self.auto_laps = int(p('auto_stop_laps'))
        self.start_x, self.start_y = float(p('start_x')), float(p('start_y'))
        self.start_yaw = float(p('start_yaw_rad'))
        self.start_dist = float(p('start_position_tolerance_m'))
        self.start_yaw_tol = math.radians(float(p('start_heading_tolerance_deg')))
        self.max_jump = float(p('max_amcl_jump_m'))
        self.path = self.read_path(str(p('waypoints_file')))
        if not 0 < self.speed_cap <= 1.2:
            raise ValueError('Simulation speed cap must be >0 and <=1.2 m/s')
        if self.lookahead < 0.25:
            raise ValueError('Lookahead must be >=0.25m')
        if self.count_publishers('/drive') > 0:
            raise RuntimeError('Another /drive publisher exists. STOP the old racing controller FIRST.')

        self.enabled = False
        self.latched_stop = ''
        self.amcl_pose = None
        self.amcl_seen = None
        self.amcl_stamp = None
        self.last_amcl_position = None
        self.scan_seen = None
        self.scan_stamp = None
        self.scan_min_front = float('inf')
        self.odom_seen = None
        self.collision = False
        self.last_no_motion_request = 0.0
        self.no_motion_pending = None
        self.previous_index = None
        self.progress_points = 0
        self.laps_completed = 0
        self.started = False
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 10)
        self.create_subscription(LaserScan, '/fusion/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Bool, '/ego_racecar/collision', self.on_collision, 10)
        self.create_subscription(Bool, '/amcl_drive/enable', self.on_enable, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, 10)
        self.nomotion_client = self.create_client(Empty, '/request_nomotion_update')
        self.path_csv = Path(str(p('results_file'))).expanduser()
        self.path_csv.parent.mkdir(parents=True, exist_ok=True)
        self.csv_fp = self.path_csv.open('w', newline='')
        self.csv = csv.writer(self.csv_fp)
        self.csv.writerow(['wall_time_s', 'state', 'reason', 'estimated_x_map',
                           'estimated_y_map', 'estimated_heading_deg', 'estimated_cte_m',
                           'waypoint_index', 'commanded_speed_mps', 'steering_deg',
                           'amcl_age_s', 'scan_age_s', 'front_range_m', 'lap'])
        self.last_print = 0.0
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(
            f'SIM CLOSED-LOOP controller READY, but DISABLED. '
            f'Uses slam_map->fusion_base TF, /amcl_pose, /fusion/scan, /odometry/filtered. '
            f'/drive publisher ACTIVE at zero speed. '
            f'Speed cap={self.speed_cap:.2f}m/s, lookahead={self.lookahead:.2f}m. '
            f'CSV: {self.path_csv}')
        self.get_logger().warn('This is simulation-only code. DO NOT connect /drive to physical actuators.')

    @staticmethod
    def read_path(path):
        pts = []
        with open(Path(path).expanduser()) as f:
            for line in f:
                if line.strip() and not line.lstrip().startswith('#'):
                    fields = line.strip().split(';')
                    pts.append((float(fields[0]), float(fields[1]), float(fields[2])))
        if len(pts) < 20 or any(pts[i+1][0] <= pts[i][0] for i in range(len(pts)-1)):
            raise ValueError('Expected >=20 increasing arc-length SLAM-map waypoints')
        return pts

    def on_enable(self, msg):
        if msg.data:
            if self.latched_stop:
                self.get_logger().error(f'REFUSED enable: latched fault={self.latched_stop}. Reset entire sim to clear.')
                return
            self.enabled = True
            self.get_logger().warn('ENABLE received. Car moves ONLY when all safety guards pass.')
        else:
            self.enabled = False
            self.get_logger().warn('DISABLED: commanding zero speed')

    def on_amcl(self, msg):
        if msg.header.frame_id != 'slam_map':
            self.latched_stop = f'AMCL wrong frame {msg.header.frame_id}'
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        stamp = stamp_s(msg.header.stamp)
        if self.last_amcl_position is not None and self.started:
            last_stamp, last_x, last_y = self.last_amcl_position
            dt = stamp - last_stamp
            if 0 < dt < 0.5 and math.hypot(x-last_x, y-last_y) > self.max_jump:
                self.latched_stop = f'AMCL pose jump >{self.max_jump:.2f}m'
                self.get_logger().error(self.latched_stop)
        self.last_amcl_position = (stamp, x, y)
        self.amcl_pose = msg.pose
        self.amcl_seen = time.monotonic()
        self.amcl_stamp = stamp

    def on_scan(self, msg):
        self.scan_seen = time.monotonic()
        self.scan_stamp = stamp_s(msg.header.stamp)
        minimum = float('inf')
        for i, range_m in enumerate(msg.ranges):
            if not math.isfinite(range_m) or not msg.range_min <= range_m <= msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) <= self.front_fov:
                minimum = min(minimum, range_m)
        self.scan_min_front = minimum

    def on_odom(self, msg):
        self.odom_seen = time.monotonic()

    def on_collision(self, msg):
        if msg.data:
            self.collision = True
            self.latched_stop = 'simulated collision'
            self.get_logger().error('COLLISION LATCHED: stopping')

    def request_nomotion_if_needed(self, now_mono):
        if self.started and self.enabled:
            return
        if now_mono - self.last_no_motion_request < 1.5:
            return
        if not self.nomotion_client.service_is_ready():
            return
        if self.no_motion_pending is not None and not self.no_motion_pending.done():
            return
        self.no_motion_pending = self.nomotion_client.call_async(Empty.Request())
        self.last_no_motion_request = now_mono

    def nearest(self, x, y):
        return min(range(len(self.path)), key=lambda i:
                   (self.path[i][1]-x)**2 + (self.path[i][2]-y)**2)

    def target(self, i):
        required = self.path[i][0] + self.lookahead
        for j in range(i, len(self.path)):
            if self.path[j][0] >= required:
                return j
        remaining = required - self.path[-1][0]
        for j in range(len(self.path)):
            if self.path[j][0] >= remaining:
                return j
        return 0

    def path_cte(self, x, y):
        pts = self.path
        return min(dist_to_segment(x, y, pts[i][1], pts[i][2],
                                   pts[(i+1)%len(pts)][1], pts[(i+1)%len(pts)][2])
                   for i in range(len(pts)))

    def steer(self, x, y, yaw, target_idx):
        tx, ty = self.path[target_idx][1], self.path[target_idx][2]
        dx, dy = tx-x, ty-y
        r2 = dx*dx+dy*dy
        if r2 < 1e-6:
            return 0.0
        lateral = -math.sin(yaw)*dx + math.cos(yaw)*dy
        return max(-self.max_steer, min(self.max_steer,
                                       math.atan(self.wheelbase*2*lateral/r2)))

    def stop(self):
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.drive.speed = 0.0
        cmd.drive.steering_angle = 0.0
        self.drive_pub.publish(cmd)

    def tick(self):
        mono = time.monotonic()
        self.request_nomotion_if_needed(mono)
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        state = 'STOP'
        reason = 'disabled'
        px = py = yaw = float('nan')
        cte = float('nan')
        nearest_idx = -1
        speed = steering = 0.0
        pose_age = ros_now-self.amcl_stamp if self.amcl_stamp is not None else float('inf')
        scan_age = ros_now-self.scan_stamp if self.scan_stamp is not None else float('inf')

        if self.latched_stop:
            reason = self.latched_stop
        elif self.count_publishers('/drive') > 1:
            self.latched_stop = 'more than one /drive publisher'
            reason = self.latched_stop
        elif not self.enabled:
            reason = 'disabled'
        elif self.amcl_pose is None:
            reason = 'waiting for AMCL pose'
        elif not (-0.3 <= pose_age <= self.max_pose_age) or mono-self.amcl_seen > self.max_pose_age:
            reason = 'AMCL pose stale'
        elif self.scan_seen is None or not (-0.2 <= scan_age <= self.max_scan_age) or mono-self.scan_seen > self.max_scan_age:
            reason = 'LiDAR scan stale'
        elif self.odom_seen is None or mono-self.odom_seen > self.max_odom_age:
            reason = 'EKF odometry stale'
        elif self.scan_min_front < self.front_stop:
            reason = f'front obstacle {self.scan_min_front:.2f}m'
        else:
            pose = self.amcl_pose
            cov = pose.covariance
            std_pos = math.sqrt(max(0., cov[0], cov[7]))
            std_yaw = math.sqrt(max(0., cov[35]))
            if not all(math.isfinite(z) for z in (std_pos, std_yaw)):
                reason = 'invalid AMCL covariance'
            elif std_pos > self.max_pos_std or std_yaw > self.max_yaw_std:
                reason = 'AMCL uncertainty too high'
            else:
                try:
                    tf = self.tf_buffer.lookup_transform('slam_map', 'fusion_base', Time())
                    trans = tf.transform.translation
                    px, py, yaw = trans.x, trans.y, get_yaw(tf.transform.rotation)
                    tf_age = ros_now - stamp_s(tf.header.stamp)
                    if not (-0.4 <= tf_age <= self.max_odom_age):
                        reason = f'TF stale ({tf_age:.2f}s)'
                    elif not all(math.isfinite(z) for z in (px, py, yaw)):
                        reason = 'invalid TF pose'
                    else:
                        nearest_idx = self.nearest(px, py)
                        cte = self.path_cte(px, py)
                        if not self.started:
                            from_start = math.hypot(px-self.start_x, py-self.start_y)
                            yaw_err = abs(angle_diff(yaw, self.start_yaw))
                            if from_start > self.start_dist or yaw_err > self.start_yaw_tol:
                                reason = f'NOT at expected start: offset={from_start:.2f}m yaw={math.degrees(yaw_err):.0f}deg'
                            else:
                                self.started = True
                                self.previous_index = nearest_idx
                                self.get_logger().warn('ARMED: closed-loop AMCL/EKF steering engaged (sim only).')
                        if self.started:
                            if cte > self.max_cte:
                                reason = f'estimated cross-track error {cte:.2f}m'
                            else:
                                speed = self.speed_cap
                                steering = self.steer(px, py, yaw, self.target(nearest_idx))
                                state, reason = 'DRIVE', 'OK'
                                if self.previous_index is not None:
                                    inc = (nearest_idx-self.previous_index) % len(self.path)
                                    if 0 < inc <= 12:
                                        self.progress_points += inc
                                        if self.progress_points >= len(self.path):
                                            self.progress_points -= len(self.path)
                                            self.laps_completed += 1
                                            self.get_logger().warn(f'ESTIMATED LAP {self.laps_completed} COMPLETE')
                                            if self.auto_laps > 0 and self.laps_completed >= self.auto_laps:
                                                self.latched_stop = f'completed {self.auto_laps} estimated lap(s)'
                                                reason = self.latched_stop
                                                state, speed, steering = 'STOP', 0.0, 0.0
                                self.previous_index = nearest_idx
                except TransformException as exc:
                    reason = f'waiting for TF: {str(exc)[:70]}'

        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.drive.speed = speed
        cmd.drive.steering_angle = steering
        self.drive_pub.publish(cmd)
        self.csv.writerow([f'{ros_now:.3f}', state, reason,
                           f'{px:.4f}', f'{py:.4f}', f'{math.degrees(yaw):.3f}',
                           f'{cte:.4f}', nearest_idx, f'{speed:.3f}',
                           f'{math.degrees(steering):.3f}', f'{pose_age:.3f}',
                           f'{scan_age:.3f}', f'{self.scan_min_front:.3f}', self.laps_completed])
        if mono-self.last_print >= 1.0:
            self.csv_fp.flush()
            self.get_logger().info(f'{state}: {reason} | speed={speed:.2f} m/s '
                f'steer={math.degrees(steering):+.1f}deg CTE={cte:.3f}m '
                f'AMCLage={pose_age:.2f}s front={self.scan_min_front:.2f}m '
                f'lap={self.laps_completed} progress={self.progress_points}/{len(self.path)}')
            self.last_print = mono

    def close(self):
        try:
            for _ in range(8):
                self.stop()
                time.sleep(0.025)
        except Exception:
            pass
        self.csv_fp.flush()
        self.csv_fp.close()
        print(f'Autonomous command log: {self.path_csv}', flush=True)


def main():
    rclpy.init()
    node = AutonomousDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
