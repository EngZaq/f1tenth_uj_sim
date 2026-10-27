#!/usr/bin/env python3
"""F1TENTH V3.2: conservative LiDAR arc selector. Physical platform is shadow-only.

This is an experimental LOCAL planner, not a certified safety system.
Scans are assumed to use ROS conventions, with laser yaw aligned to base_link.
Unknown/no-return space is NOT considered verified free space.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def scan_points(scan, max_distance=None):
    ranges = np.asarray(scan.ranges, dtype=float)
    angles = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
    max_valid = scan.range_max if max_distance is None else min(scan.range_max, max_distance)
    usable = np.isfinite(ranges) & (ranges > max(0.05, scan.range_min)) & (ranges <= max_valid)
    # Positive infinity means no hit, NOT a confirmed free cell out to 100m.
    # Retain it separately for visibility/coverage accounting.
    return angles, ranges, usable


def assess_scan(scan):
    a, r, valid = scan_points(scan)
    f = np.abs(a) <= math.radians(20)
    if np.count_nonzero(f) < 5:
        return False, 'no forward coverage'
    # A sparse forward window is too uncertain to approve motion.
    if np.mean(valid[f]) < 0.50:
        return False, 'insufficient finite forward returns'
    if np.percentile(r[f & valid], 15) < 0.55:
        return False, 'front obstacle'
    return True, 'ok'


def arc_pose(s, delta, wheelbase):
    kappa = math.tan(delta) / wheelbase
    if abs(kappa) < 1e-7:
        return s, 0.0, 0.0
    th = s * kappa
    return math.sin(th) / kappa, (1 - math.cos(th)) / kappa, th


def arc_clearance(scan, delta, wheelbase=0.33, length=0.34, width=0.20,
                  lidar_from_rear=0.25, margin=0.10, horizon=1.0):
    """Check finite laser returns against swept rectangular car footprint.

    Coordinates: base_link proxy at vehicle rear edge; laser at x=lidar_from_rear.
    Treats every finite return as an obstacle point; does not prove free space
    in unobserved regions. Unknown forward coverage is screened elsewhere.
    """
    # Retain all actual finite detections for scan-quality accounting.
    # Nearby obstacle point filtering happens separately below.
    a, r, valid = scan_points(scan)
    if not np.any(valid):
        return False, 0.0
    # Convert obstacles from laser-centered frame to rear-edge body frame.
    ox = lidar_from_rear + r[valid] * np.cos(a[valid])
    oy = r[valid] * np.sin(a[valid])
    # Only consider nearby obstacle points; bbox for swept path.
    near = (ox >= -0.8) & (ox <= horizon + length + 1.0) & (abs(oy) <= 2.0)
    ox, oy = ox[near], oy[near]
    if ox.size == 0:
        return False, 0.0  # No measured evidence of geometry.
    # A footprint-sized box around each sampled vehicle pose. x from rear to nose.
    half_w = width / 2 + margin
    front = length + margin
    rear = -margin
    worst = float('inf')
    for s in np.linspace(0, horizon, max(9, int(horizon / 0.05) + 1)):
        x, y, heading = arc_pose(float(s), delta, wheelbase)
        dx, dy = ox - x, oy - y
        local_x = np.cos(heading) * dx + np.sin(heading) * dy
        local_y = -np.sin(heading) * dx + np.cos(heading) * dy
        # Distance to expanded rectangle (0 if within).
        ex = np.maximum(np.maximum(rear - local_x, local_x - front), 0.0)
        ey = np.maximum(abs(local_y) - half_w, 0.0)
        distances = np.hypot(ex, ey)
        closest = float(np.min(distances))
        worst = min(worst, closest)
        if closest <= 0.001:
            return False, 0.0
    return True, worst


def direction_evidence(scan, heading, half_angle=math.radians(9)):
    """Finite-range evidence along a proposed heading (laser is forward-aligned).

    Positive infinity is *unknown*, not evidence of a free corridor.
    """
    angles, ranges, valid = scan_points(scan)
    delta = np.arctan2(np.sin(angles - heading), np.cos(angles - heading))
    selected = np.abs(delta) <= half_angle
    if np.count_nonzero(selected) < 5:
        return 0.0, 0.0
    confidence = float(np.mean(valid[selected]))
    measured = ranges[selected & valid]
    if measured.size < 4:
        return 0.0, confidence
    return float(np.percentile(measured, 20)), confidence


def choose_arc(scan, previous=0.0, *, wheelbase=0.33, length=0.34,
               width=0.20, lidar_from_rear=0.25, max_steer=0.40,
               margin=0.10, horizon=0.80):
    good, reason = assess_scan(scan)
    if not good:
        return 0.0, 0.0, reason

    # A steering command is a *curved path*, not a direct heading command.
    # Approximate arc tangent at preview distance to compare scan evidence.
    choices = np.linspace(-max_steer, max_steer, 33)
    candidates = []
    blocked = 0
    unsupported = 0
    for steering in choices:
        steering = float(steering)
        safe, clearance = arc_clearance(scan, steering, wheelbase, length,
                                        width, lidar_from_rear, margin, horizon)
        if not safe:
            blocked += 1
            continue
        _, _, heading = arc_pose(horizon, steering, wheelbase)
        # Use a preview at half the arc's final yaw, not the whole yaw change.
        distance, confidence = direction_evidence(scan, 0.5 * heading)
        if confidence < 0.55 or distance <= 0.60:
            unsupported += 1
            continue
        # Forward free-space evidence drives the turn decision; straightness
        # and continuity are secondary preferences, not the entire objective.
        score = (1.3 * min(distance, 2.5) / 2.5
                 + 0.35 * min(clearance, 0.5) / 0.5
                 - 0.18 * abs(steering / max_steer)
                 - 0.22 * abs((steering - previous) / max_steer))
        candidates.append((score, steering, distance))
    if not candidates:
        return 0.0, 0.0, f'no arc blocked={blocked} unsupported={unsupported}'
    _, steer, distance = max(candidates)
    steer = clamp(steer, previous - 0.045, previous + 0.045)
    speed = 0.15 * max(0.65, 1.0 - abs(steer) / max_steer * 0.35)
    if distance < 1.0:
        speed = min(speed, 0.10)
    return steer, speed, 'ok'


class ExplorationV3(Node):
    def __init__(self):
        super().__init__('uj_exploration_v3_2_debug')
        self.declare_parameter('platform', 'physical')
        self.declare_parameter('enable_drive', False)
        self.declare_parameter('proposal_topic', '/exploration/v3_2_debug/drive')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('max_speed', 0.15)
        self.declare_parameter('scan_timeout', 0.25)
        self.platform = str(self.get_parameter('platform').value)
        if self.platform not in ('physical', 'sim'):
            raise ValueError('platform must be physical or sim')
        if self.platform == 'physical' and self.get_parameter('enable_drive').value:
            raise ValueError('Physical actuation is deliberately disabled in V3')
        self.proposed = self.create_publisher(AckermannDriveStamped, self.get_parameter('proposal_topic').value, 10)
        self.drive = (self.create_publisher(AckermannDriveStamped, '/drive', 10)
                      if self.platform == 'sim' and self.get_parameter('enable_drive').value else None)
        self.scan = None
        self.seen = None
        self.collision = False
        self.steer = 0.0  # last accepted steering, kept through safety stops
        self.good_scan_streak = 0
        self.last_log_time = 0
        self.last_status = None
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value, self.on_scan, 10)
        if self.platform == 'sim':
            self.create_subscription(Bool, '/ego_racecar/collision', self.on_collision, 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f'V3.2 debug running: platform={self.platform}, actuation={self.drive is not None}')

    def on_scan(self, msg):
        self.scan = msg
        self.seen = self.get_clock().now()

    def on_collision(self, msg):
        self.collision = bool(msg.data)

    def tick(self):
        now = self.get_clock().now()
        speed, target, reason = 0.0, 0.0, 'waiting for scan'
        if self.collision:
            reason = 'collision'
        elif self.scan is not None and self.seen is not None:
            age = (now - self.seen).nanoseconds / 1e9
            if 0 <= age <= float(self.get_parameter('scan_timeout').value):
                target, speed, reason = choose_arc(self.scan, self.steer)
                speed = min(speed, float(self.get_parameter('max_speed').value))
            else:
                reason = 'stale scan'
        if reason == 'ok':
            self.good_scan_streak += 1
            self.steer = target
            # Two consecutive accepted control cycles to re-enable motion.
            if self.good_scan_streak < 2:
                speed = 0.0
                reason = 'waiting for stable clearance'
        else:
            self.good_scan_streak = 0
            # Fail-stop immediately; do not erase the previous steering preference.
            target = 0.0
            speed = 0.0
        cmd = AckermannDriveStamped()
        cmd.header.stamp = now.to_msg()
        cmd.drive.steering_angle = float(target)
        cmd.drive.speed = float(speed)
        self.proposed.publish(cmd)
        if self.drive is not None:
            self.drive.publish(cmd)
        if reason != self.last_status and (now.nanoseconds - self.last_log_time) > 500_000_000:
            self.last_log_time = now.nanoseconds
            self.get_logger().info(f'status={reason} steer={target:+.3f} speed={speed:.2f} mode={"SIM DRIVE" if self.drive else "SHADOW"}')
            self.last_status = reason


def main():
    rclpy.init()
    node = ExplorationV3()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # Stopping a node does not constitute a physical emergency stop.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

