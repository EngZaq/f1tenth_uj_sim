#!/usr/bin/env python3
"""Reuse the repository's existing LiDAR free-gap explorer to drive SIM only.
Automatically stop when mission_first_lap reports a closed lap.
"""
import time
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "exploration"))
import rclpy
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.node import Node
from exploration_controller import ExplorationController


class MissionExplorer(ExplorationController):
    def __init__(self):
        super().__init__()
        if not bool(self.get_parameter('enable_drive').value):
            raise ValueError('Mission explorer must be explicitly enabled in SIM by launcher')
        self.finished=False
        self.latched_collision=False
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool,'/mission/first_lap_complete',self.on_complete,qos)
        self.get_logger().warn('SIM ONLY: mapping first lap by LiDAR, actual /drive publisher.')

    def collision_cb(self, msg):
        if msg.data:
            self.latched_collision=True
        self.collision=self.latched_collision

    def on_complete(self,msg):
        if msg.data and not self.finished:
            self.finished=True
            self.get_logger().warn('FIRST LAP COMPLETE: Explorer STOPPED, maintaining 20Hz zero commands')

    def tick(self):
        if self.finished or self.latched_collision:
            msg=AckermannDriveStamped()
            msg.header.stamp=self.get_clock().now().to_msg()
            msg.drive.speed=0.0
            msg.drive.steering_angle=0.0
            self.proposed.publish(msg)
            self.drive.publish(msg)
        else:
            super().tick()


def main():
    rclpy.init()
    n=MissionExplorer()
    try: rclpy.spin(n)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException): pass
    finally:
        for _ in range(8):
            msg=AckermannDriveStamped()
            msg.drive.speed=0.0
            n.drive.publish(msg)
            time.sleep(.02)
        n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__=='__main__': main()
