#!/usr/bin/env python3
"""Detect a loop using EKF odometry, then capture SLAM-frame final pose.
Simulation milestone only. Never subscribes to simulator truth.
"""
import json
import math
import os
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from map_coverage import corridor_coverage


def yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))


def diff(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


class FirstLap(Node):
    def __init__(self):
        super().__init__('mission_first_lap')
        self.declare_parameter('output', '/tmp/f1tenth_mission_lap.json')
        self.declare_parameter('min_distance_m', 22.0)
        self.declare_parameter('min_time_s', 40.0)
        self.declare_parameter('leave_distance_m', 2.5)
        self.declare_parameter('return_distance_m', 0.85)
        self.declare_parameter('heading_tolerance_deg', 48.0)
        self.file = Path(str(self.get_parameter('output').value)).expanduser()
        self.start = None
        self.prev = None
        self.start_wall = None
        self.distance = 0.0
        self.left_start = False
        self.complete = False
        self.first_return_distance = None
        self.coverage = {'closed':False}
        self.map_seen = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Bool, '/mission/first_lap_complete', qos)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, 10)
        self.create_subscription(OccupancyGrid, '/mission/slam_map', self.on_map, qos)
        self.last_info = 0.0
        self.get_logger().info('First-lap watcher uses EKF odom and SLAM TF, never simulator truth.')

    def on_map(self, msg):
        if msg.header.frame_id != 'slam_map':
            return
        try:
            self.coverage=corridor_coverage(msg.data,msg.info.width,msg.info.height,msg.info.resolution)
            self.map_seen=time.monotonic()
        except ValueError as exc:
            self.coverage={'closed':False}
            self.get_logger().error('Coverage check rejected map: '+str(exc))

    def on_odom(self, msg):
        if self.complete:
            return
        p = msg.pose.pose
        x, y, h = p.position.x, p.position.y, yaw(p.orientation)
        now = time.monotonic()
        if self.start is None:
            self.start = (x, y, h)
            self.prev = (x, y)
            self.start_wall = now
            self.get_logger().info('EKF first-lap origin captured')
            return
        delta = math.hypot(x - self.prev[0], y - self.prev[1])
        # Reject impossible EKF discontinuity (sensor reset), not treat as travel.
        if delta > 0.70:
            self.get_logger().error('EKF discontinuity >0.70m; lap tracker refuses to continue')
            return
        self.distance += delta
        self.prev = (x, y)
        radius = math.hypot(x-self.start[0], y-self.start[1])
        heading_error = abs(math.degrees(diff(h,self.start[2])))
        elapsed = now-self.start_wall
        if radius > float(self.get_parameter('leave_distance_m').value):
            self.left_start = True
        if now-self.last_info > 6.0:
            self.get_logger().info(f'Explored={self.distance:.1f}m away={radius:.2f}m '
                                   f'heading_delta={heading_error:.1f}deg elapsed={elapsed:.0f}s '
                                   f'observed_corridor_closed={self.coverage["closed"]}')
            self.last_info = now
        if not (self.left_start and
                self.distance >= float(self.get_parameter('min_distance_m').value) and
                elapsed >= float(self.get_parameter('min_time_s').value) and
                radius <= float(self.get_parameter('return_distance_m').value) and
                heading_error <= float(self.get_parameter('heading_tolerance_deg').value)):
            return
        if self.first_return_distance is None:
            self.first_return_distance = self.distance
        if (not self.coverage['closed'] or self.map_seen is None or now-self.map_seen>8.):
            # An odometry return is only a hypothesis of a complete map. Keep
            # exploring until the observed 0.30 m corridor actually closes.
            return
        try:
            tf = self.tf_buffer.lookup_transform('slam_map','fusion_base',Time())
        except TransformException as exc:
            self.get_logger().warn(f'Loop found but SLAM pose unavailable: {str(exc)[:100]}')
            return
        m = tf.transform
        record = {'finish_x':float(m.translation.x), 'finish_y':float(m.translation.y),
                  'finish_yaw':yaw(m.rotation), 'frame_id':'slam_map',
                  'distance_odom_m':round(self.distance,3),
                  'estimated_lap_length_m':round(self.first_return_distance,3),
                  'duration_s':round(elapsed,2),
                  'return_distance_odom_m':round(radius,3),
                  'return_yaw_error_deg':round(heading_error,2),
                  'observed_corridor':self.coverage,
                  'source':'EKF return plus observed closed corridor; current pose from SLAM TF'}
        self.file.parent.mkdir(parents=True,exist_ok=True)
        temp=self.file.with_suffix('.tmp')
        temp.write_text(json.dumps(record, indent=2))
        os.replace(temp,self.file)
        self.complete=True
        self.pub.publish(Bool(data=True))
        self.get_logger().warn(f'FIRST LAP DETECTED. STOP EXPLORATION. {self.distance:.2f}m'
                               f' Pose in slam_map=({record["finish_x"]:.2f},{record["finish_y"]:.2f})')


def main():
    rclpy.init()
    node=FirstLap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__=='__main__': main()
