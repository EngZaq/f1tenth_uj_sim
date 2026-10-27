#!/usr/bin/env python3
"""Extract a closed racing centerline from a ROS SLAM occupancy map."""

import argparse
import csv
import math
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy import ndimage
from scipy.interpolate import CubicSpline
from skimage.morphology import skeletonize


def extract(map_yaml, output_prefix, step, max_speed):
    metadata = yaml.safe_load(Path(map_yaml).read_text())

    image_path = Path(metadata['image'])
    if not image_path.is_absolute():
        image_path = Path(map_yaml).parent / image_path

    pixels = np.asarray(Image.open(image_path).convert('L'))
    height, width = pixels.shape

    # White = observed free; gray = unknown; black = occupied.
    free = pixels >= 250

    labels, count = ndimage.label(
        free, structure=np.ones((3, 3))
    )
    if count == 0:
        raise ValueError("No observed free space")

    counts = np.bincount(labels.ravel())
    counts[0] = 0
    region = labels == np.argmax(counts)

    skeleton = skeletonize(region)
    coords = [tuple(p) for p in np.argwhere(skeleton)]
    lookup = set(coords)

    neighbors = {}
    for row, col in coords:
        neighbors[(row, col)] = [
            (row + dr, col + dc)
            for dr in (-1, 0, 1)
            for dc in (-1, 0, 1)
            if (dr or dc)
            and (row + dr, col + dc) in lookup
        ]

    # This map must contain exactly one simple closed loop.
    if len(coords) < 40 or any(
        len(v) != 2 for v in neighbors.values()
    ):
        raise ValueError(
            "Skeleton is not a simple loop; inspect map"
        )

    start = min(coords)
    route = [start]
    previous, current = None, start

    while True:
        nxt = [
            p for p in neighbors[current]
            if p != previous
        ][0]

        if nxt == start:
            break

        if nxt in route:
            raise ValueError("Unexpected secondary loop")

        route.append(nxt)
        previous, current = current, nxt

    if len(route) != len(coords):
        raise ValueError("Disconnected centerline")

    resolution = float(metadata['resolution'])
    ox, oy, angle = map(float, metadata['origin'])

    rows = np.array([p[0] for p in route], dtype=float)
    cols = np.array([p[1] for p in route], dtype=float)

    u = (cols + 0.5) * resolution
    v = (height - rows - 0.5) * resolution

    x = ox + np.cos(angle) * u - np.sin(angle) * v
    y = oy + np.sin(angle) * u + np.cos(angle) * v

    # Smooth pixel stair-stepping without opening the loop.
    x = ndimage.gaussian_filter1d(x, 3.0, mode='wrap')
    y = ndimage.gaussian_filter1d(y, 3.0, mode='wrap')

    points = np.column_stack([x, y])
    closed = np.vstack([points, points[0]])

    ds = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    cumulative = np.r_[0.0, np.cumsum(ds)]
    length = float(cumulative[-1])

    n = int(round(length / step))
    if n < 40:
        raise ValueError("Insufficient path samples")

    s = np.linspace(0, length, n, endpoint=False)

    fx = CubicSpline(
        cumulative, closed[:, 0], bc_type='periodic'
    )
    fy = CubicSpline(
        cumulative, closed[:, 1], bc_type='periodic'
    )

    x, y = fx(s), fy(s)
    dx, dy = fx(s, 1), fy(s, 1)
    ddx, ddy = fx(s, 2), fy(s, 2)

    yaw = np.arctan2(dy, dx)
    kappa = (dx * ddy - dy * ddx) / np.maximum(
        (dx * dx + dy * dy) ** 1.5, 1e-9
    )

    # Validate against occupied AND unknown space.
    clearance_map = (
        ndimage.distance_transform_edt(region) * resolution
    )

    mx = np.cos(angle) * (x - ox) + np.sin(angle) * (y - oy)
    my = -np.sin(angle) * (x - ox) + np.cos(angle) * (y - oy)

    pc = np.floor(mx / resolution).astype(int)
    pr = height - 1 - np.floor(my / resolution).astype(int)

    if np.any((pc < 0) | (pc >= width) |
              (pr < 0) | (pr >= height)):
        raise ValueError("Path exits the map")

    if not np.all(region[pr, pc]):
        raise ValueError("Path enters unobserved/occupied space")

    clearance = clearance_map[pr, pc]

    if clearance.min() < 0.30:
        raise ValueError(
            f"Insufficient centerline clearance: "
            f"{clearance.min():.3f} m"
        )

    xy = np.column_stack([x, y])
    segments = np.linalg.norm(
        np.roll(xy, -1, axis=0) - xy, axis=1
    )

    # Conservative simulation-only speed profile.
    lateral_accel = 0.9
    acceleration = 0.5
    braking = 0.8

    raw_speed = np.minimum(
        max_speed,
        np.sqrt(
            lateral_accel /
            np.maximum(np.abs(kappa), 1e-3)
        )
    )

    speed = raw_speed.copy()

    for _ in range(30):
        for i in range(n - 1, -1, -1):
            j = (i + 1) % n
            speed[i] = min(
                speed[i],
                math.sqrt(
                    speed[j] ** 2 + 2 * braking * segments[i]
                )
            )

        for i in range(n):
            j = (i + 1) % n
            speed[j] = min(
                speed[j],
                math.sqrt(
                    speed[i] ** 2 + 2 * acceleration * segments[i]
                )
            )

    ax = (
        np.roll(speed, -1) ** 2 - speed ** 2
    ) / (2 * segments + 1e-9)

    prefix = Path(output_prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)

    path_csv = prefix.with_name(prefix.name + '_centerline.csv')
    speed_csv = prefix.with_name(prefix.name + '_speed_profile.csv')
    preview_file = prefix.with_name(prefix.name + '_preview.png')

    # Existing Pure Pursuit: seven semicolon-separated columns.
    with path_csv.open('w') as f:
        f.write(
            '# Coordinates generated in slam_map frame\n'
        )
        f.write(
            '# s_m;x_m;y_m;psi_rad;kappa_1pm;vx_mps;ax_mps2\n'
        )

        for values in zip(s, x, y, yaw, kappa, speed, ax):
            f.write(
                ';'.join(f'{float(z):.6f}' for z in values)
                + '\n'
            )

    # Existing racing controller's speed-profile format.
    with speed_csv.open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'index', 's_m', 'x_m', 'y_m',
            'curvature_1pm', 'raw_speed_mps',
            'planned_speed_mps'
        ])

        for i in range(n):
            writer.writerow([
                i, f'{s[i]:.4f}',
                f'{x[i]:.4f}', f'{y[i]:.4f}',
                f'{kappa[i]:.5f}',
                f'{raw_speed[i]:.3f}',
                f'{speed[i]:.3f}'
            ])

    preview = Image.fromarray(pixels).convert('RGB')
    preview = preview.resize(
        (width * 4, height * 4),
        Image.Resampling.NEAREST
    )

    path_pixels = list(zip(
        mx / resolution * 4,
        (height - my / resolution) * 4
    ))

    draw = ImageDraw.Draw(preview)
    draw.line(
        path_pixels + [path_pixels[0]],
        fill=(0, 170, 250),
        width=3
    )
    preview.save(preview_file)

    print(f"Closed skeleton: {len(route)} pixels")
    print(f"Smoothed path: {n} waypoints, {length:.2f} m")
    print(f"Minimum clearance: {clearance.min():.2f} m")
    print(f"Median clearance: {np.median(clearance):.2f} m")
    print(f"Speed range: {speed.min():.2f}–{speed.max():.2f} m/s")
    print("Saved:")
    print(path_csv)
    print(speed_csv)
    print(preview_file)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('map_yaml')
    parser.add_argument('--out', required=True)
    parser.add_argument('--step', type=float, default=0.10)
    parser.add_argument('--max-speed', type=float, default=1.2)

    args = parser.parse_args()
    extract(args.map_yaml, args.out, args.step, args.max_speed)
