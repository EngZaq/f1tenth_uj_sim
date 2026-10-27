
#!/usr/bin/env python3
import math
import sys
import numpy as np
import rosbag2_py

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

bag_path = sys.argv[1]

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
    rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
)

sectors = {
    'front': (-20, 20),
    'front_left': (20, 80),
    'front_right': (-80, -20),
}

counts = {name: [] for name in sectors}
steering = []
speeds = []
scan_count = 0

while reader.has_next():
    topic, data, timestamp = reader.read_next()

    if topic == '/exploration/drive':
        msg = deserialize_message(data, AckermannDriveStamped)
        steering.append(msg.drive.steering_angle)
        speeds.append(msg.drive.speed)

    elif topic == '/scan':
        msg = deserialize_message(data, LaserScan)
        scan_count += 1

        angles = np.degrees(
            msg.angle_min +
            np.arange(len(msg.ranges)) * msg.angle_increment
        )
        ranges = np.asarray(msg.ranges, dtype=float)

        valid = (
            np.isfinite(ranges)
            & (ranges > 0.05)
            & (ranges <= msg.range_max)
        )

        for name, (lo, hi) in sectors.items():
            selected = (angles >= lo) & (angles <= hi)
            counts[name].append(
                float(np.mean(valid[selected])) if np.any(selected) else 0.0
            )

print(f'Scans analyzed: {scan_count}')
print('\nValid beam percentage by sector:')

for name, values in counts.items():
    print(f'  {name}: {100 * np.mean(values):.1f}%')

if steering:
    s = np.asarray(steering)
    v = np.asarray(speeds)
    print('\nV2 proposed commands:')
    print(f'  Samples: {len(s)}')
    print(f'  Mean absolute steering: {np.mean(np.abs(s)):.3f} rad')
    print(f'  Steering saturation (|δ| >= 0.39): {100*np.mean(np.abs(s)>=0.39):.1f}%')
    print(f'  Zero-speed commands: {100*np.mean(v == 0):.1f}%')
