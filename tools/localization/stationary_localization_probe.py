#!/usr/bin/env python3
"""SIM-ONLY stationary diagnostics; no drive, TF or pose publications.

Simulator truth is recorded for evaluation only. It never sets the AMCL pose,
filters measurements, or changes the no-motion request schedule.
"""
import argparse
import json
import math
import time
from pathlib import Path


def stamp_s(message):
    t = message.header.stamp
    return t.sec+t.nanosec*1e-9


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_safe(item) for key,item in value.items()}
    if isinstance(value, list):
        return [json_safe(item) for item in value]
    return value


class Moments:
    def __init__(self):
        self.n = 0
        self.mean = self.m2 = 0.0
        self.minimum = self.maximum = None

    def add(self, value):
        value = float(value)
        if not math.isfinite(value):
            return
        self.n += 1
        delta = value-self.mean
        self.mean += delta/self.n
        self.m2 += delta*(value-self.mean)
        self.minimum = value if self.minimum is None else min(self.minimum, value)
        self.maximum = value if self.maximum is None else max(self.maximum, value)

    def result(self):
        return {'count': self.n, 'mean': self.mean if self.n else None,
                'sample_std': math.sqrt(max(0.0, self.m2/(self.n-1))) if self.n > 1 else None,
                'min': self.minimum, 'max': self.maximum}


class ProbeData:
    def __init__(self):
        self.signals = {}
        self.streams = {}
        self.scans = {}
        self.frames = {}
        self.poses = {}

    def sample(self, name, value):
        self.signals.setdefault(name, Moments()).add(value)

    def arrival(self, name, message):
        t = stamp_s(message)
        stream = self.streams.setdefault(name, {'count': 0, 'first_stamp_s': t,
                                               'last_stamp_s': t,
                                               'nonincreasing_stamps': 0})
        if stream['count']:
            dt = t-stream['last_stamp_s']
            if dt > 0:
                self.sample(name+'_interval_s', dt)
            else:
                stream['nonincreasing_stamps'] += 1
        stream['last_stamp_s'] = t
        stream['count'] += 1

    def scan(self, name, message):
        self.arrival(name, message)
        self.frames[name] = message.header.frame_id
        # Count every arrival but copy the full ranges at <=1 Hz. The probe
        # must not consume a large fraction of the simulator's CPU budget.
        if name in self.scans and 0 <= stamp_s(message)-self.scans[name]['last']['stamp_s'] < 1.0:
            return
        record = {'stamp_s': stamp_s(message), 'frame_id': message.header.frame_id,
                  'angle_min': float(message.angle_min),
                  'angle_max': float(message.angle_max),
                  'angle_increment': float(message.angle_increment),
                  'range_min': float(message.range_min), 'range_max': float(message.range_max),
                  'time_increment': float(message.time_increment),
                  'scan_time': float(message.scan_time),
                  'ranges': [float(v) if math.isfinite(v) else None for v in message.ranges]}
        item = self.scans.setdefault(name, {'first': record, 'last': record})
        item['last'] = record

    def odom(self, name, message):
        self.arrival(name, message)
        if name == 'truth':
            self.frames['truth_body'] = message.child_frame_id
        twist = message.twist.twist
        for suffix, value in (('vx_mps', twist.linear.x), ('vy_mps', twist.linear.y),
                              ('wz_rad_s', twist.angular.z)):
            self.sample(name+'_'+suffix, value)
        p = message.pose.pose
        record = {'stamp_s': stamp_s(message), 'frame_id': message.header.frame_id,
                  'child_frame_id': message.child_frame_id,
                  'x': float(p.position.x), 'y': float(p.position.y),
                  'quaternion': {k: float(getattr(p.orientation, k)) for k in ('x','y','z','w')}}
        item = self.poses.setdefault(name, {'first': record, 'last': record})
        item['last'] = record
        self.sample(name+'_displacement_from_first_m',
                    math.hypot(record['x']-item['first']['x'], record['y']-item['first']['y']))

    def imu(self, message):
        self.arrival('imu_raw', message)
        self.sample('imu_raw_wz_rad_s', message.angular_velocity.z)
        self.sample('imu_reported_wz_variance', message.angular_velocity_covariance[8])

    def amcl(self, message):
        self.arrival('amcl', message)

    def result(self):
        return {'signals': {k: v.result() for k,v in self.signals.items()},
                'streams': self.streams, 'frames': self.frames,
                'scans': self.scans, 'poses': self.poses}


class RefreshSchedule:
    """One outstanding service request; monotonic cadence and response timeout."""
    def __init__(self, period=0.30, timeout=2.0):
        if not all(math.isfinite(v) and v > 0 for v in (period, timeout)):
            raise ValueError('Refresh period and timeout must be finite and positive')
        self.period, self.timeout = period, timeout
        self.pending = None
        self.sent_at = None
        self.sent = self.completed = 0

    def tick(self, now, ready, send):
        if self.pending is not None:
            if self.pending.done():
                self.pending.result()  # Propagate service exceptions.
                self.completed += 1
                self.pending = None
            elif now-self.sent_at >= self.timeout:
                raise RuntimeError('AMCL no-motion service response timed out')
            else:
                return
        if ready and (self.sent_at is None or now-self.sent_at >= self.period):
            self.pending = send()
            self.sent_at = now
            self.sent += 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--duration', type=float, default=45.0)
    parser.add_argument('--refresh-period', type=float, default=0.30)
    args = parser.parse_args()
    try:
        schedule = RefreshSchedule(args.refresh_period)
        if not math.isfinite(args.duration) or args.duration <= 0:
            raise ValueError('Duration must be finite and positive')
    except ValueError as exc:
        parser.error(str(exc))

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, LaserScan
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from std_srvs.srv import Empty
    from tf2_ros import Buffer, TransformListener, TransformException

    rclpy.init(args=[])
    node = Node('stationary_localization_probe')
    data = ProbeData()
    client = node.create_client(Empty, '/request_nomotion_update')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    # Keep subscription objects for the complete observation period.
    subscriptions = [
        node.create_subscription(LaserScan, '/scan', lambda m: data.scan('raw_scan',m), qos_profile_sensor_data),
        node.create_subscription(LaserScan, '/fusion/scan', lambda m: data.scan('fusion_scan',m), qos_profile_sensor_data),
        node.create_subscription(Odometry, '/ego_racecar/odom', lambda m: data.odom('truth',m), 30),
        node.create_subscription(Odometry, '/odometry/filtered', lambda m: data.odom('ekf',m), 30),
        node.create_subscription(Odometry, '/sim/vesc/wheel_odom', lambda m: data.odom('wheel',m), 30),
        node.create_subscription(Imu, '/sim/imu/data_raw', data.imu, 30),
        node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', data.amcl, 30)]
    transforms, tf_errors = {}, {}
    started = time.monotonic()
    status, failure = 'incomplete', None
    try:
        while rclpy.ok() and time.monotonic()-started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            schedule.tick(now, client.service_is_ready(), lambda: client.call_async(Empty.Request()))
            if now-started > 5 and schedule.sent == 0:
                raise RuntimeError('AMCL no-motion service unavailable')
            pairs = [('fusion_base','fusion_laser')]
            if 'truth_body' in data.frames and 'raw_scan' in data.frames:
                pairs.append((data.frames['truth_body'], data.frames['raw_scan']))
            for parent, child in pairs:
                key = parent+' -> '+child
                if key in transforms:
                    continue
                try:
                    tf = buffer.lookup_transform(parent, child, Time())
                    transforms[key] = {'translation': {k:float(getattr(tf.transform.translation,k)) for k in ('x','y','z')},
                                       'quaternion': {k:float(getattr(tf.transform.rotation,k)) for k in ('x','y','z','w')}}
                    tf_errors.pop(key, None)
                except TransformException as exc:
                    tf_errors[key] = str(exc)
        if not rclpy.ok():
            raise RuntimeError('ROS context stopped before observation completed')
        status = 'observation completed; no accuracy pass/fail assigned'
    except BaseException as exc:
        failure = str(exc)
        raise
    finally:
        record = data.result()
        record.update({'status': status, 'failure': failure,
                       'duration_s': time.monotonic()-started,
                       'requested_refresh_period_s': args.refresh_period,
                       'refresh_requests': schedule.sent, 'refresh_completed': schedule.completed,
                       'transforms': transforms, 'tf_errors': tf_errors,
                       'scan_snapshot_max_hz': 1.0,
                       'scan_range_encoding': 'null means nonfinite original range; no invented hit'})
        try:
            import xml.etree.ElementTree as ET
            from ament_index_python.packages import get_package_share_directory
            package_xml = Path(get_package_share_directory('nav2_amcl'))/'package.xml'
            record['nav2_amcl_package_version'] = ET.parse(package_xml).getroot().findtext('version')
        except Exception as exc:
            record['nav2_amcl_version_read_error'] = str(exc)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(json_safe(record), indent=2, allow_nan=False)+'\n')
        print(f'Stationary probe: {schedule.sent} refresh requests; '
              f'{data.streams.get("amcl",{}).get("count",0)} AMCL poses. Output: {args.output}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
