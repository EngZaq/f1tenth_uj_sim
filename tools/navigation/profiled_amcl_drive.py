#!/usr/bin/env python3
"""Experimental ROS/Gym saved-map racing controller, NOT for hardware.

Uses AMCL/EKF map-frame pose, a full speed-profile route and forward predicted
vehicle footprints. Every fault latches a zero-speed hold. No simulator truth
enters this controller; the synthetic sensor source remains sim-only.

WARNING: this in-process guard is not independent redundancy. Production needs
a separate command arbiter, heartbeat braking, physical emergency stop and
hard actuator limits.
"""
import csv
import math
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener, TransformException

from racing_policy import ProfiledRoute, normalized_angle, ramp_velocity, route_progress_delta
from confidence_policy import ConfidenceGate, uncertainty_speed_cap
from pose_uncertainty import planar_covariance, project_uncertainty
from trajectory_safety import command_swept_collision, stopping_horizon


def stamp_s(stamp):
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def yaw_of(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


class ProfiledAMCLDrive(Node):
    def __init__(self):
        super().__init__('profiled_amcl_drive')
        defaults = {
            'waypoints_file': '',
            'results_file': '/tmp/f1tenth_profiled.csv',
            'speed_cap_mps': 0.40,
            'lookahead_base_m': 0.50,
            'lookahead_speed_gain_s': 0.30,
            'max_steering_angle': 0.4189,
            'max_path_cte_m': 0.09,
            'max_pose_age_s': 0.65,
            'max_scan_age_s': 0.45,
            'max_odom_age_s': 0.45,
            'max_across_track_std_m': 0.08,
            'max_along_track_std_m': 0.12,
            'max_major_position_std_m': 0.16,
            'max_heading_std_deg': 10.0,
            'max_amcl_jump_m': 0.45,
            'swept_margin_m': 0.07,
            'start_x': 0.0,
            'start_y': 0.0,
            'start_yaw_rad': 0.0,
            'start_position_tolerance_m': 0.45,
            'start_heading_tolerance_deg': 25.0,
            'auto_stop_laps': 1,
            'drive_topic': '/drive',
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = lambda k: self.get_parameter(k).value
        self.route = ProfiledRoute(str(p('waypoints_file')))
        self.speed_limit = float(p('speed_cap_mps'))
        self.cte_limit = float(p('max_path_cte_m'))
        self.max_steer = float(p('max_steering_angle'))
        self.pose_limit = float(p('max_pose_age_s'))
        self.scan_limit = float(p('max_scan_age_s'))
        self.odom_limit = float(p('max_odom_age_s'))
        self.across_std_limit = float(p('max_across_track_std_m'))
        self.along_std_limit = float(p('max_along_track_std_m'))
        self.major_std_limit = float(p('max_major_position_std_m'))
        self.yaw_std_limit = math.radians(float(p('max_heading_std_deg')))
        self.confidence = ConfidenceGate(soft_pos=self.across_std_limit,
            soft_along=self.along_std_limit,hard_pos=self.major_std_limit,
            soft_yaw_deg=float(p('max_heading_std_deg')))
        self.jump_limit = float(p('max_amcl_jump_m'))
        self.margin = float(p('swept_margin_m'))
        self.aim_base = float(p('lookahead_base_m'))
        self.aim_gain = float(p('lookahead_speed_gain_s'))
        self.start = (float(p('start_x')), float(p('start_y')),
                      float(p('start_yaw_rad')))
        self.start_tolerance = float(p('start_position_tolerance_m'))
        self.start_yaw_tolerance = math.radians(float(p('start_heading_tolerance_deg')))
        self.auto_laps = int(p('auto_stop_laps'))
        self.drive_topic = str(p('drive_topic'))
        if self.drive_topic not in ('/drive','/mission/drive'):
            raise ValueError('Unsupported simulator drive topic')
        if not 0.05 < self.speed_limit <= 0.8:
            raise ValueError('Simulation-only speed cap must be in (0.05, 0.8]m/s')
        if not 0.04 <= self.margin <= 0.2:
            raise ValueError('Invalid predicted footprint collision margin')
        if not (0<self.across_std_limit<=.08 and .08<self.along_std_limit<self.major_std_limit<=.16):
            raise ValueError('Invalid route-relative confidence bounds')
        if self.count_publishers(self.drive_topic):
            raise RuntimeError('STOP all existing /drive publishers before starting this controller')

        self.enabled = False
        self.started = False
        self.latch = ''
        self.pose = None
        self.pose_stamp = None
        self.pose_seen = None
        self.last_amcl_position = None
        self.scan = None
        self.scan_stamp = None
        self.scan_seen = None
        self.scans = deque(maxlen=8)
        self.odom_seen = None
        self.measured_speed = 0.
        self.measured_yaw_rate = 0.
        self.no_motion_pending = None
        self.no_motion_at = 0.
        self.no_motion_count = 0
        self.index = None
        self.progress = 0
        self.travel = 0.
        self.last_xy = None
        self.laps = 0
        self.last_tick = time.monotonic()
        self.last_output_speed = 0.
        self.last_steering = 0.
        self.last_print = 0.

        self.pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, 10)
        self.create_subscription(LaserScan, '/fusion/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, 10)
        self.create_subscription(Bool, '/ego_racecar/collision', self.on_collision, 10)
        self.create_subscription(Bool, '/amcl_drive/enable', self.on_enable, 10)
        self.no_motion_client = self.create_client(Empty, '/request_nomotion_update')

        csv_path = Path(str(p('results_file'))).expanduser()
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.fp = csv_path.open('w', newline='')
        self.writer = csv.writer(self.fp)
        self.writer.writerow(['ros_time', 'state', 'reason', 'index',
                              'progress', 'lap', 'x', 'y', 'yaw_deg',
                              'cte_m', 'plan_speed_mps', 'cmd_speed_mps',
                              'steering_deg', 'amcl_age_s', 'scan_age_s',
                              'odom_age_s', 'predicted_body_gap_m',
                              'valid_scan_hits', 'no_motion_count',
                              'amcl_pos_std_m', 'amcl_yaw_std_deg', 'confidence_state',
                              'amcl_across_track_std_m', 'amcl_along_track_std_m',
                              'measured_speed_mps','amcl_stamp_s','amcl_major_std_m',
                              'uncertainty_speed_cap_mps','stopping_horizon_m'])
        self.timer = self.create_timer(.05, self.tick)
        self.get_logger().warn(
            f'PROFILED CONTROLLER DISABLED; route={p("waypoints_file")}, log={csv_path}, cap={self.speed_limit}m/s. '
            'Simulation ONLY. Explicit /amcl_drive/enable required.')

    def on_enable(self, msg):
        if msg.data and self.latch:
            self.get_logger().error(f'Cannot enable after safety hold: {self.latch}')
            return
        self.enabled = bool(msg.data)
        self.get_logger().warn('ARMED request accepted' if self.enabled else 'DISABLED')

    def on_pose(self, msg):
        if msg.header.frame_id != 'slam_map':
            self.hold('unexpected AMCL map frame')
            return
        t = stamp_s(msg.header.stamp)
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if not (math.isfinite(x) and math.isfinite(y)):
            self.hold('invalid AMCL position')
            return
        if self.last_amcl_position and self.started:
            last_t, old_x, old_y = self.last_amcl_position
            if 0 < t - last_t < 0.6 and math.hypot(x-old_x, y-old_y) > self.jump_limit:
                self.hold('AMCL pose jump')
        self.last_amcl_position = (t, x, y)
        self.pose = msg.pose
        self.pose_stamp = t
        self.pose_seen = time.monotonic()

    def on_scan(self, msg):
        if msg.header.frame_id != 'fusion_laser':
            self.hold('unexpected LiDAR frame')
            return
        self.scan = msg
        self.scan_stamp = stamp_s(msg.header.stamp)
        self.scan_seen = time.monotonic()
        self.scans.append(msg)

    def on_odom(self, msg):
        self.odom_seen = time.monotonic()
        self.measured_speed = float(msg.twist.twist.linear.x)
        self.measured_yaw_rate = float(msg.twist.twist.angular.z)

    def local_scan(self,ros_now):
        # A newly arrived scan may precede its EKF transform by one callback.
        # Use the newest transformable scan within the same freshness limit.
        for scan in reversed(self.scans):
            scan_age=ros_now-stamp_s(scan.header.stamp)
            if not 0<=scan_age<=self.scan_limit:
                continue
            source_time=Time.from_msg(scan.header.stamp)
            if not self.tf_buffer.can_transform_full(
                    'fusion_base',Time(),'fusion_laser',source_time,'fusion_odom'):
                continue
            transform=self.tf_buffer.lookup_transform_full(
                'fusion_base',Time(),'fusion_laser',source_time,'fusion_odom')
            if not -.1<=ros_now-stamp_s(transform.header.stamp)<=self.odom_limit:
                continue
            t=transform.transform
            return scan,(t.translation.x,t.translation.y,yaw_of(t.rotation)),scan_age
        raise ValueError('no fresh scan with local odometry transform')

    def on_collision(self, msg):
        if msg.data:
            self.hold('simulated collision')

    def hold(self, reason):
        if not self.latch:
            self.latch = reason
            self.get_logger().error(f'SAFETY HOLD: {reason}. Requires mission restart.')
        self.enabled = False

    def nomotion_update(self, monotonic_now, pose_age):
        if self.scan_seen is None:
            return
        # Proactive refresh at slow speeds; a stationary update should not
        # depend on the controller being disabled.
        if (self.enabled and self.confidence.paused_since is None and
                math.isfinite(pose_age) and pose_age < 0.20):
            return
        if monotonic_now - self.no_motion_at < 0.27:
            return
        if not self.no_motion_client.service_is_ready():
            return
        if self.no_motion_pending is not None and not self.no_motion_pending.done():
            return
        self.no_motion_pending = self.no_motion_client.call_async(Empty.Request())
        self.no_motion_at = monotonic_now
        self.no_motion_count += 1

    def send(self, speed=0., steering=0.):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)
        self.pub.publish(msg)

    def tick(self):
        mono = time.monotonic()
        dt = max(.0, min(mono - self.last_tick, .2))
        self.last_tick = mono
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        age = lambda stamp: ros_now - stamp if stamp is not None else float('inf')
        pose_age, scan_age = age(self.pose_stamp), age(self.scan_stamp)
        odom_age = mono-self.odom_seen if self.odom_seen is not None else float('inf')
        self.nomotion_update(mono, pose_age)
        state = 'STOP'
        reason = 'disabled'
        x = y = yaw = cte = plan = gap = float('nan')
        pos_std = yaw_std = float('nan')
        confidence_state = 'NOT_EVALUATED'
        across_std = along_std = float('nan')
        major_std = float('nan')
        confidence_cap = float('nan')
        horizon = float('nan')
        index = self.index if self.index is not None else -1
        valid_hits = 0
        speed = steer = 0.
        issue = ''
        if self.latch:
            reason = 'SAFETY HOLD: ' + self.latch
        elif self.count_publishers(self.drive_topic) > 1:
            issue = 'competing /drive publisher'
        elif not self.enabled:
            reason = 'disabled'
        elif self.pose is None:
            issue = 'missing AMCL pose'
        elif self.pose_seen is None or not (-.3 <= pose_age <= self.pose_limit) or mono-self.pose_seen > self.pose_limit:
            issue = 'AMCL pose stale'
        elif self.scan_seen is None or not (-.3 <= scan_age <= self.scan_limit) or mono-self.scan_seen > self.scan_limit:
            issue = 'LiDAR scan stale'
        elif not 0 <= odom_age <= self.odom_limit:
            issue = 'EKF odometry stale'
        else:
            cov = self.pose.covariance
            try:
                covariance=planar_covariance(cov)
            except ValueError as exc:
                issue = str(exc)
            else:
                pos_std = math.sqrt(max(cov[0], cov[7]))
                # Distinguish corridor-normal uncertainty from ambiguity along
                # the corridor. Collision prediction uses local motion below.
                px0 = self.pose.pose.position.x
                py0 = self.pose.pose.position.y
                diagnostic_idx = int(np.argmin(
                    np.sum((self.route.xy-np.array([px0,py0]))**2,axis=1)))
                path_yaw = float(self.route.data[diagnostic_idx,3])
                across_std,along_std,major_std,yaw_std=project_uncertainty(covariance,path_yaw)
                confidence_state, confidence_detail = self.confidence.check(
                    mono,self.pose_stamp,across_std,yaw_std,self.measured_speed,
                    along_std=along_std,major_std=major_std)
                if confidence_state == 'FAULT':
                    issue = confidence_detail
                elif confidence_state == 'PAUSE':
                    reason = 'LOCALIZATION PAUSE: ' + confidence_detail
                else:
                    try:
                        tf = self.tf_buffer.lookup_transform('slam_map', 'fusion_base', Time())
                        tf_age = age(stamp_s(tf.header.stamp))
                        if not (-.35 <= tf_age <= .45):
                            raise ValueError('stale map->base TF')
                        x = tf.transform.translation.x
                        y = tf.transform.translation.y
                        yaw = yaw_of(tf.transform.rotation)
                        index = self.route.nearest(x, y, self.index if self.started else None)
                        cte = self.route.error(x, y, index)
                        if not all(map(math.isfinite, [x, y, yaw, cte])):
                            raise ValueError('nonfinite estimated pose')
                        if not self.started:
                            if math.hypot(x-self.start[0], y-self.start[1]) > self.start_tolerance:
                                raise ValueError('outside scan-seeded start region')
                            if abs(normalized_angle(yaw-self.start[2])) > self.start_yaw_tolerance:
                                raise ValueError('start heading disagreement')
                        if cte > self.cte_limit:
                            raise ValueError(f'estimated CTE {cte:.3f}m exceeds {self.cte_limit:.3f}m')
                        confidence_cap=uncertainty_speed_cap(self.speed_limit,along_std,pause_limit=self.along_std_limit)
                        plan = min(self.speed_limit,self.route.speed_at(index),confidence_cap)
                        lookahead = max(.45, min(.80, self.aim_base + self.aim_gain * plan))
                        steer = self.route.aim(x, y, yaw, index, lookahead, self.max_steer)
                        # Compensate scan age through local odometry, then roll
                        # out the actual body-frame steering command. Global
                        # map/AMCL translation error cancels from this geometry.
                        if not all(map(math.isfinite,(self.measured_speed,self.measured_yaw_rate))):
                            raise ValueError('nonfinite measured motion')
                        scan,scan_to_body,scan_age=self.local_scan(ros_now)
                        guard_velocity=max(self.measured_speed,self.last_output_speed,plan)
                        reaction=max(.35,max(0.,scan_age)+.25)
                        horizon=stopping_horizon(guard_velocity,reaction)
                        initial_steer=self.last_steering
                        if abs(self.measured_speed)>=.08:
                            initial_steer=float(np.clip(math.atan(
                                .3302*self.measured_yaw_rate/self.measured_speed),
                                -self.max_steer,self.max_steer))
                        blocked, gap, valid_hits = command_swept_collision(
                            scan.ranges,scan.angle_min,scan.angle_increment,
                            scan.range_min,scan.range_max,steer,guard_velocity,
                            initial_steering=initial_steer,scan_to_body=scan_to_body,
                            reaction=reaction,margin=self.margin)
                        if blocked:
                            raise ValueError(f'predicted swept-path obstruction {gap:.3f}m, hits={valid_hits}, stopping_horizon={horizon:.3f}m')
                        if self.started and self.index is not None:
                            self.progress += route_progress_delta(self.index,index,self.route.n)
                        if self.last_xy is not None:
                            step = math.hypot(x-self.last_xy[0], y-self.last_xy[1])
                            if step > .2:
                                raise ValueError('estimated pose discontinuity')
                            self.travel += step
                        self.last_xy = (x, y)
                        self.index = index
                        self.started = True
                        if (self.progress >= self.route.n and
                                self.travel >= .75 * self.route.length and
                                math.hypot(x-self.start[0], y-self.start[1]) < .75):
                            self.progress -= self.route.n
                            self.laps += 1
                            self.get_logger().warn(f'ESTIMATED LAP {self.laps} COMPLETE')
                            if self.auto_laps > 0 and self.laps >= self.auto_laps:
                                self.hold(f'completed {self.auto_laps} estimated laps')
                        if not self.latch:
                            speed = ramp_velocity(self.last_output_speed, plan, dt)
                            state, reason = 'DRIVE', 'OK'
                    except (TransformException, ValueError) as exc:
                        issue = str(exc)
        if issue:
            self.hold(issue)
            reason = 'SAFETY HOLD: '+self.latch
        if self.latch:
            state, speed, steer = 'STOP', 0., 0.
        self.send(speed, steer)
        self.last_output_speed = speed
        self.last_steering = steer
        self.writer.writerow([
            f'{ros_now:.3f}', state, reason, index, self.progress, self.laps,
            f'{x:.4f}', f'{y:.4f}', f'{math.degrees(yaw):.3f}',
            f'{cte:.4f}', f'{plan:.3f}', f'{speed:.3f}',
            f'{math.degrees(steer):.3f}', f'{pose_age:.3f}',
            f'{scan_age:.3f}', f'{odom_age:.3f}', f'{gap:.3f}',
            valid_hits, self.no_motion_count,
            f'{pos_std:.4f}', f'{math.degrees(yaw_std):.3f}', confidence_state,
            f'{across_std:.4f}', f'{along_std:.4f}',
            f'{self.measured_speed:.6f}',
            f'{self.pose_stamp:.9f}' if self.pose_stamp is not None else 'nan',
            f'{major_std:.4f}',f'{confidence_cap:.4f}',f'{horizon:.4f}'])
        if mono-self.last_print > 1.0:
            self.fp.flush()
            self.get_logger().info(
                f'{state}: {reason} | v={speed:.2f}m/s planned={plan:.2f} '
                f'CTE={cte:.3f}m body_gap={gap:.3f}m '
                f'pos_std={pos_std:.3f}m lateral_std={across_std:.3f}m '
                f'longitudinal_std={along_std:.3f}m yaw_std={math.degrees(yaw_std):.1f}deg '
                f'AMCL={pose_age:.2f}s progress={self.progress}/{self.route.n} '
                f'lap={self.laps}')
            self.last_print = mono

    def close(self):
        # A launcher SIGINT can invalidate ROS before rclpy.spin() returns.
        # The launcher also emits an independent zero-speed command.
        try:
            if self.context.ok():
                for _ in range(10):
                    if not self.context.ok():
                        break
                    try:
                        self.send()
                    except Exception as exc:
                        self.get_logger().warn(f'Zero-speed publish during shutdown failed: {exc}')
                        break
                    time.sleep(.025)
        finally:
            if not self.fp.closed:
                self.fp.flush()
                self.fp.close()


def main():
    rclpy.init()
    node = ProfiledAMCLDrive()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
