#!/usr/bin/env python3
"""SIM-ONLY single /drive owner with heartbeat, scan and collision stops.

This is not an independent obstacle planner or a physical emergency stop.
Controllers publish to /mission/drive; this process publishes /drive at 20 Hz.
"""
import argparse
import json
import math
import time
from pathlib import Path


class DrivePolicy:
    def __init__(self, command_timeout=.25, scan_timeout=.45):
        self.command_timeout, self.scan_timeout = command_timeout, scan_timeout
        self.command = (0., 0.)
        self.command_seen = self.scan_seen = None
        self.fault = ''

    def accept(self, now, speed, steering, message_age=0.):
        if not all(math.isfinite(v) for v in (speed, steering, message_age)):
            self.fault = 'nonfinite drive command'
        elif not 0 <= speed <= .8 or abs(steering) > .419:
            self.fault = 'drive command exceeds simulation limits'
        elif speed > 0 and not -.05 <= message_age <= self.command_timeout:
            self.command_seen = None
        else:
            self.command, self.command_seen = (speed, steering), now

    def output(self, now, competing_publishers=False):
        if competing_publishers:
            self.fault = 'competing /drive publisher'
        if self.scan_seen is not None and now-self.scan_seen > self.scan_timeout:
            self.fault = 'LiDAR scan heartbeat lost'
        if self.fault:
            return 0., 0., self.fault
        if self.scan_seen is None:
            return 0., 0., 'waiting for scan'
        if self.command_seen is None or not 0 <= now-self.command_seen <= self.command_timeout:
            return 0., 0., 'command heartbeat absent'
        return self.command[0], self.command[1], 'forwarding'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fault-file', type=Path, required=True)
    args = parser.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from ackermann_msgs.msg import AckermannDriveStamped
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool
    rclpy.init(args=[])
    node = Node('simulation_drive_watchdog')
    policy = DrivePolicy()
    if node.count_publishers('/drive'):
        raise RuntimeError('Existing /drive publisher; watchdog refuses ownership')
    pub = node.create_publisher(AckermannDriveStamped, '/drive', 10)
    fault_written = False
    def command(message):
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        now = node.get_clock().now().nanoseconds*1e-9
        speed = float(message.drive.speed)
        age = now-stamp if stamp else (float('inf') if speed else 0.)
        policy.accept(time.monotonic(), speed,float(message.drive.steering_angle),age)
    def scan(message):
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        age = node.get_clock().now().nanoseconds*1e-9-stamp
        if -.05 <= age <= policy.scan_timeout:
            policy.scan_seen = time.monotonic()
    def collision(message):
        if message.data:
            policy.fault = 'simulator collision'
    def tick():
        nonlocal fault_written
        speed, steering, reason = policy.output(time.monotonic(), node.count_publishers('/drive') > 1)
        msg = AckermannDriveStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.drive.speed, msg.drive.steering_angle = speed, steering
        pub.publish(msg)
        if policy.fault and not fault_written:
            args.fault_file.write_text(json.dumps({'fault':reason,'time_s':time.time()})+'\n')
            fault_written = True
            node.get_logger().error('WATCHDOG STOP: '+reason)
    node.create_subscription(AckermannDriveStamped, '/mission/drive', command, 10)
    node.create_subscription(LaserScan, '/scan', scan, qos_profile_sensor_data)
    node.create_subscription(Bool, '/ego_racecar/collision', collision, 10)
    node.create_timer(.05, tick)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            policy.command_seen = None
            tick()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
