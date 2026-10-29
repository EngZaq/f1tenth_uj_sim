#!/usr/bin/env python3
"""Pure geometry: LiDAR hits tested against future Ackermann vehicle footprints.
This is a supporting collision guard, not a stand-alone emergency-stop system.
"""
import math
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sim_geometry import LASER


def swept_path_collision(ranges, angle_min, angle_increment, range_min, range_max,
                         route, pose, nearest_idx, velocity,
                         wheelbase=0.3302, laser_x=LASER['x_m'],
                         front=0.46145, rear=-0.11855, halfwidth=0.155,
                         margin=0.07, deceleration=0.7, reaction=0.35,
                         laser_y=LASER['y_m'], laser_yaw=LASER['yaw_rad']):
    """route Nx4 columns s,x,y,yaw; pose=(x,y,yaw) in route map frame.

    Returns (blocked, minimum_obstacle_to_body_distance_m, valid_hit_count).
    Uses measured endpoints only; requires independent scan freshness and
    vehicle emergency braking. Map and localization errors need separate margin.
    """
    route = np.asarray(route)
    ranges = np.asarray(ranges, dtype=float)
    if ranges.ndim != 1 or ranges.size < 50 or not 0 <= nearest_idx < len(route):
        raise ValueError('Invalid LiDAR/path inputs')
    angles = angle_min + np.arange(ranges.size) * angle_increment
    valid = (np.isfinite(ranges) &
             (ranges >= max(0.08, range_min)) &
             (ranges <= min(range_max - 0.02, 12.0)))
    if int(valid.sum()) < 35:
        return True, 0.0, int(valid.sum())
    radii, bearings = ranges[valid], angles[valid]
    hits = np.column_stack((laser_x + radii * np.cos(bearings+laser_yaw),
                            laser_y + radii * np.sin(bearings+laser_yaw)))
    speed = max(0.0, float(velocity))
    horizon = max(0.55, speed * reaction +
                  speed * speed / (2 * max(deceleration, 0.1)) + 0.55)

    base_s = float(route[nearest_idx, 0])
    closed_length = float(route[-1, 0]) + 0.1
    x0, y0, yaw0 = pose
    c0, s0 = math.cos(yaw0), math.sin(yaw0)
    footprints = [(0.0, 0.0, 0.0)]
    for offset in range(min(len(route), 100)):
        i = (nearest_idx + offset) % len(route)
        ds = (float(route[i, 0]) - base_s) % max(closed_length, 0.1)
        if offset > 0 and ds > horizon:
            break
        dx, dy = route[i, 1] - x0, route[i, 2] - y0
        heading = math.atan2(math.sin(route[i, 3] - yaw0),
                             math.cos(route[i, 3] - yaw0))
        footprints.append((c0 * dx + s0 * dy,
                           -s0 * dx + c0 * dy, heading))

    nearest = float('inf')
    for x, y, heading in footprints:
        dx, dy = hits[:, 0] - x, hits[:, 1] - y
        c, s = math.cos(heading), math.sin(heading)
        lon, lat = c * dx + s * dy, -s * dx + c * dy
        edge_x = np.maximum(np.maximum(rear - lon, lon - front), 0.0)
        edge_y = np.maximum(np.abs(lat) - halfwidth, 0.0)
        gap = float(np.min(np.hypot(edge_x, edge_y)))
        nearest = min(nearest, gap)
        if gap < margin:
            return True, nearest, int(valid.sum())
    return False, nearest, int(valid.sum())
