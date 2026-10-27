import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from exploration_controller_v3_2 import scan_points, arc_pose


def diagnose(scan):
    angles, ranges, valid = scan_points(scan)

    # Existing V3.2 geometry, deliberately unchanged for diagnosis.
    wheelbase = 0.33
    length = 0.34
    width = 0.20
    lidar_x = 0.25
    margin = 0.10
    horizon = 0.80

    ox = lidar_x + ranges[valid] * np.cos(angles[valid])
    oy = ranges[valid] * np.sin(angles[valid])

    near = (
        (ox >= -0.8)
        & (ox <= horizon + length + 1.0)
        & (np.abs(oy) <= 2.0)
    )
    ox, oy = ox[near], oy[near]

    rear = -margin
    front = length + margin
    half_w = width / 2 + margin

    print(f"Nearby obstacle points: {len(ox)}")
    print("Steering | First collision distance | Obstacle point")

    blocked = 0

    for steer in np.linspace(-0.40, 0.40, 33):
        hit = None

        for s in np.linspace(0, horizon, 33):
            x, y, heading = arc_pose(float(s), float(steer), wheelbase)

            dx = ox - x
            dy = oy - y

            lx = np.cos(heading) * dx + np.sin(heading) * dy
            ly = -np.sin(heading) * dx + np.cos(heading) * dy

            inside = (
                (lx >= rear)
                & (lx <= front)
                & (np.abs(ly) <= half_w)
            )

            if np.any(inside):
                i = int(np.flatnonzero(inside)[0])
                hit = (s, ox[i], oy[i])
                break

        if hit:
            blocked += 1
            print(
                f"{steer:+.3f} rad | {hit[0]:.3f} m | "
                f"({hit[1]:+.3f}, {hit[2]:+.3f})"
            )
        else:
            print(f"{steer:+.3f} rad | CLEAR | --")

    print(f"\nBlocked arcs: {blocked}/33")


class ScanDiagnostic(Node):
    def __init__(self):
        super().__init__("arc_collision_diagnostic")
        self.done = False
        self.sub = self.create_subscription(
            LaserScan, "/scan", self.callback, 10
        )

    def callback(self, msg):
        diagnose(msg)
        self.done = True


rclpy.init()
node = ScanDiagnostic()

try:
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.5)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
