
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class CheckFootprint(Node):
    def __init__(self):
        super().__init__('check_footprint')
        self.create_subscription(LaserScan, '/scan', self.callback, 10)

    def callback(self, msg):
        r = np.asarray(msg.ranges, dtype=float)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment

        valid = np.isfinite(r) & (r > max(0.05, msg.range_min)) & (r <= msg.range_max)

        # Current V3.2 assumptions:
        lidar_x = 0.25
        length = 0.34
        half_width = 0.20  # 0.10 half vehicle width + 0.10 margin
        rear = -0.10
        front = length + 0.10

        x = lidar_x + r[valid] * np.cos(a[valid])
        y = r[valid] * np.sin(a[valid])

        inside = (x >= rear) & (x <= front) & (np.abs(y) <= half_width)

        print("Finite returns:", len(x))
        print("Points inside initial expanded footprint:", np.count_nonzero(inside))

        for xx, yy in zip(x[inside][:10], y[inside][:10]):
            print(f"  obstacle: x={xx:.3f}, y={yy:.3f}")

        rclpy.try_shutdown()

rclpy.init()
node = CheckFootprint()
try:
    rclpy.spin(node)
except Exception:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
