#!/usr/bin/env python3
"""Read-only AMCL Pure Pursuit shadow experiment for F1TENTH ROS 2 Humble.

Subscribes to /amcl_pose (slam_map), computes Pure Pursuit on the unregistered
SLAM-map centerline, and publishes ONLY /drive_shadow. The simulator does NOT
consume /drive_shadow. It also observes /drive for diagnostic comparison only;
/drive never influences the computed command.
"""
import csv
import math
import os
import statistics
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def segment_distance(px, py, ax, ay, bx, by):
    vx, vy = bx - ax, by - ay
    denom = vx * vx + vy * vy
    if denom <= 1e-12:
        return math.hypot(px - ax, py - ay)
    u = max(0.0, min(1.0, ((px - ax)*vx + (py - ay)*vy) / denom))
    return math.hypot(px - ax - u*vx, py - ay - u*vy)


class AMCLShadowPurePursuit(Node):
    def __init__(self):
        super().__init__('amcl_shadow_pure_pursuit')
        default_path = os.path.expanduser('~/f1tenth_sim/reference/generated/uj_technical_slam_01_centerline.csv')
        default_profile = os.path.expanduser('~/f1tenth_sim/reference/generated/uj_technical_slam_01_speed_profile.csv')
        default_csv = os.path.expanduser('~/f1tenth_sim/results/localization/amcl_shadow.csv')
        self.declare_parameter('waypoints_file', default_path)
        self.declare_parameter('speed_profile_file', default_profile)
        self.declare_parameter('results_file', default_csv)
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('speed_scale', 1.0)
        self.declare_parameter('speed_cap_mps', 1.2)
        self.declare_parameter('max_pose_age_s', 0.4)
        self.declare_parameter('max_position_std_m', 0.6)
        self.declare_parameter('max_heading_std_deg', 30.0)
        self.declare_parameter('max_path_error_m', 0.5)
        self.declare_parameter('expected_frame', 'slam_map')
        get = lambda p: self.get_parameter(p).value
        self.lookahead = float(get('lookahead_distance'))
        self.wheelbase = float(get('wheelbase'))
        self.max_steering = float(get('max_steering_angle'))
        self.speed_scale = float(get('speed_scale'))
        self.speed_cap = float(get('speed_cap_mps'))
        self.max_age = float(get('max_pose_age_s'))
        self.max_pos_std = float(get('max_position_std_m'))
        self.max_yaw_std = math.radians(float(get('max_heading_std_deg')))
        self.max_path_error = float(get('max_path_error_m'))
        self.expected_frame = str(get('expected_frame'))
        self.waypoints = self.load_waypoints(os.path.expanduser(get('waypoints_file')))
        self.speeds = self.load_speeds(os.path.expanduser(get('speed_profile_file')))
        if len(self.waypoints) != len(self.speeds):
            raise RuntimeError('Waypoint and speed profile lengths differ')
        self.pose = None
        self.pose_received_mono = None
        self.pose_stamp_s = None
        self.oracle = None
        self.oracle_received_mono = 0.0
        self.output_path = Path(os.path.expanduser(get('results_file')))
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = self.output_path.open('w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['ros_time_s','pose_age_s','amcl_x','amcl_y','amcl_yaw_deg',
                              'path_error_m','nearest_index','target_index',
                              'amcl_speed_mps','amcl_steering_deg','oracle_speed_mps',
                              'oracle_steering_deg','steering_difference_deg','state'])
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive_shadow', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, 10)
        self.create_subscription(AckermannDriveStamped, '/drive', self.on_oracle, 10)
        self.create_timer(0.05, self.on_timer)
        self.samples = 0
        self.invalid_samples = 0
        self.steering_diff = []
        self.last_log = time.monotonic()
        self.get_logger().info(f'SHADOW ONLY: publishes /drive_shadow, never /drive; CSV: {self.output_path}')
        self.get_logger().info(f'Loaded {len(self.waypoints)} SLAM-map waypoints, 20Hz, lookahead={self.lookahead:.2f} m')

    @staticmethod
    def load_waypoints(path):
        rows = []
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                vals = [float(x) for x in line.split(';')]
                if len(vals) < 3:
                    raise ValueError('Waypoint format requires s;x;y;...')
                rows.append((vals[0], vals[1], vals[2]))
        if len(rows) < 3:
            raise ValueError('Too few waypoints')
        if any(rows[i][0] > rows[i+1][0] for i in range(len(rows)-1)):
            raise ValueError('Waypoint arc-length must increase')
        return rows

    @staticmethod
    def load_speeds(path):
        with open(path, newline='') as f:
            return [float(row['planned_speed_mps']) for row in csv.DictReader(f)]

    def on_pose(self, msg):
        if msg.header.frame_id != self.expected_frame:
            self.get_logger().error(f'Wrong AMCL frame {msg.header.frame_id}, expected {self.expected_frame}')
            return
        self.pose = msg.pose
        self.pose_received_mono = time.monotonic()
        self.pose_stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_oracle(self, msg):
        # Diagnostics only. This message is NEVER used in steering calculations.
        self.oracle = msg.drive
        self.oracle_received_mono = time.monotonic()

    def nearest(self, x, y):
        return min(range(len(self.waypoints)), key=lambda i:
                   (self.waypoints[i][1]-x)**2 + (self.waypoints[i][2]-y)**2)

    def target(self, i):
        required = self.waypoints[i][0] + self.lookahead
        for j in range(i, len(self.waypoints)):
            if self.waypoints[j][0] >= required:
                return j
        remaining = required - self.waypoints[-1][0]
        for j in range(len(self.waypoints)):
            if self.waypoints[j][0] >= remaining:
                return j
        return 0

    def path_error(self, x, y):
        pts = self.waypoints
        return min(segment_distance(x, y, pts[i][1], pts[i][2],
                                    pts[(i+1) % len(pts)][1], pts[(i+1) % len(pts)][2])
                   for i in range(len(pts)))

    def compute(self, x, y, yaw, target_index):
        _, tx, ty = self.waypoints[target_index]
        dx, dy = tx - x, ty - y
        ly = -math.sin(yaw) * dx + math.cos(yaw) * dy
        r2 = dx * dx + dy * dy
        if r2 < 1e-6:
            return 0.0
        curvature = 2.0 * ly / r2
        return max(-self.max_steering,
                   min(self.max_steering, math.atan(self.wheelbase * curvature)))

    def on_timer(self):
        if self.pose is None:
            return
        now_mono = time.monotonic()
        stamp_now = self.get_clock().now().nanoseconds * 1e-9
        stamp_age = stamp_now - self.pose_stamp_s
        arrival_age = now_mono - self.pose_received_mono
        p = self.pose.pose.position
        yaw = yaw_of(self.pose.pose.orientation)
        nearest = self.nearest(p.x, p.y)
        target = self.target(nearest)
        cte = self.path_error(p.x, p.y)
        covariance = self.pose.covariance
        std_pos = math.sqrt(max(0., covariance[0], covariance[7]))
        std_yaw = math.sqrt(max(0., covariance[35]))
        valid = (math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(yaw) and
                 -0.15 <= stamp_age <= self.max_age and arrival_age <= self.max_age and
                 std_pos <= self.max_pos_std and std_yaw <= self.max_yaw_std and
                 cte <= self.max_path_error)
        speed = min(self.speed_cap, max(0., self.speeds[nearest] * self.speed_scale)) if valid else 0.0
        steering = self.compute(p.x, p.y, yaw, target) if valid else 0.0
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.expected_frame
        cmd.drive.speed = speed
        cmd.drive.steering_angle = steering
        self.publisher.publish(cmd)  # NEVER /drive
        oracle_ok = self.oracle is not None and (now_mono - self.oracle_received_mono) < 0.2
        ospeed = float(self.oracle.speed) if oracle_ok else float('nan')
        odeg = math.degrees(self.oracle.steering_angle) if oracle_ok else float('nan')
        diff = math.degrees(steering) - odeg if oracle_ok and valid else float('nan')
        if math.isfinite(diff):
            self.steering_diff.append(abs(diff))
        self.samples += 1
        if not valid:
            self.invalid_samples += 1
        self.writer.writerow([f'{stamp_now:.3f}', f'{stamp_age:.3f}', f'{p.x:.4f}',
                              f'{p.y:.4f}', f'{math.degrees(yaw):.3f}', f'{cte:.4f}',
                              nearest, target, f'{speed:.3f}', f'{math.degrees(steering):.3f}',
                              f'{ospeed:.3f}', f'{odeg:.3f}', f'{diff:.3f}',
                              'VALID' if valid else 'BLOCKED'])
        if now_mono - self.last_log > 5.0:
            self.csv_file.flush()
            mad = statistics.mean(self.steering_diff) if self.steering_diff else float('nan')
            self.get_logger().info(
                f'shadow speed={speed:.2f}m/s steer={math.degrees(steering):+.1f}deg '
                f'pathCTE={cte:.3f}m age={stamp_age:.2f}s '
                f'mean |steer-Oracle|={mad:.2f}deg blocked={self.invalid_samples}/{self.samples}')
            self.last_log = now_mono

    def close(self):
        self.csv_file.flush()
        self.csv_file.close()
        if self.steering_diff:
            a = sorted(self.steering_diff)
            p95 = a[min(len(a)-1, int(0.95 * (len(a)-1)))]
            self.get_logger().info(f'SHADOW SUMMARY: mean abs steer error={statistics.mean(a):.2f}deg, '
                                   f'p95={p95:.2f}deg, max={max(a):.2f}deg, '
                                   f'blocked={self.invalid_samples}/{self.samples}')
        self.get_logger().info(f'CSV saved: {self.output_path}')


def main():
    rclpy.init()
    node = AMCLShadowPurePursuit()
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
