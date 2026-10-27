#!/usr/bin/env python3
"""Register a SLAM centerline to the simulator map. Simulation only."""

from pathlib import Path
import math
import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt, map_coordinates
from scipy.optimize import differential_evolution

ws = Path.home() / 'f1tenth_sim'

config = yaml.safe_load(
    (ws / 'src/f1tenth_gym_ros/config/sim.yaml').read_text()
)
p = config['bridge']['ros__parameters']

if p['map_path'] != 'maps/uj_technical_loop_v1':
    raise SystemExit('STOP: load the technical circuit first')

track = ws / 'src/f1tenth_gym_ros' / p['map_path']
metadata = yaml.safe_load(track.with_suffix('.yaml').read_text())

image_path = Path(metadata['image'])
if not image_path.is_absolute():
    image_path = track.parent / image_path

image = np.asarray(Image.open(image_path).convert('L'))
resolution = float(metadata['resolution'])
ox, oy, map_yaw = map(float, metadata['origin'])
height, width = image.shape

clearance_grid = (
    distance_transform_edt(image >= 250) * resolution
)

prefix = ws / 'reference/generated/uj_technical_slam_01'
source = prefix.with_name(prefix.name + '_centerline.csv')
output = prefix.with_name(prefix.name + '_registered.csv')

data = np.loadtxt(source, delimiter=';', comments='#')

if data.ndim != 2 or data.shape[1] != 7:
    raise SystemExit('STOP: invalid centerline CSV')

sx = float(p['sx'])
sy = float(p['sy'])
starting_yaw = float(p['stheta'])

anchor = int(np.argmin(
    (data[:, 1] - sx)**2 + (data[:, 2] - sy)**2
))

start_error = math.hypot(
    data[anchor, 1] - sx,
    data[anchor, 2] - sy
)

if start_error > 0.30:
    raise SystemExit('STOP: starting position mismatch')

relative = data[:, 1:3] - data[anchor, 1:3]

initial_rotation = (
    (starting_yaw - data[anchor, 3] + math.pi)
    % (2 * math.pi) - math.pi
)

def transform(params):
    dx, dy, theta = params
    c, s = math.cos(theta), math.sin(theta)

    return np.column_stack([
        sx + dx + c*relative[:, 0] - s*relative[:, 1],
        sy + dy + s*relative[:, 0] + c*relative[:, 1]
    ])

def clearance(points):
    x = points[:, 0] - ox
    y = points[:, 1] - oy

    c, s = math.cos(map_yaw), math.sin(map_yaw)

    u = c*x + s*y
    v = -s*x + c*y

    cols = u / resolution - 0.5
    rows = height - v / resolution - 0.5

    return map_coordinates(
        clearance_grid,
        [rows, cols],
        order=1,
        mode='constant',
        cval=0
    )

def objective(params):
    distances = clearance(transform(params))
    dx, dy, theta = params

    return (
        np.maximum(0.45 - distances, 0.0).dot(
            np.maximum(0.45 - distances, 0.0)
        )
        + 2.0*(dx*dx + dy*dy)
        + (theta - initial_rotation)**2
    )

solution = differential_evolution(
    objective,
    bounds=[
        (-0.25, 0.25),
        (-0.25, 0.25),
        (initial_rotation - 0.15, initial_rotation + 0.15)
    ],
    seed=5,
    maxiter=120,
    tol=1e-6,
    polish=True
)

path = transform(solution.x)

# Check intermediate points, not just saved waypoints.
next_points = np.roll(path, -1, axis=0)

dense = np.concatenate([
    (1.0-t)*path + t*next_points
    for t in np.linspace(0, 1, 6, endpoint=False)
])

minimum = float(clearance(dense).min())

print(f'Start error: {start_error:.3f} m')
print(f'Minimum path clearance: {minimum:.3f} m')
print(
    f'Rotation correction: '
    f'{math.degrees(solution.x[2]):+.2f} deg'
)

if minimum < 0.39:
    raise SystemExit(
        'STOP: insufficient clearance. Racing is not authorized.'
    )

registered = data.copy()
registered[:, 1:3] = path
registered[:, 3] = (
    (data[:, 3] + solution.x[2] + math.pi)
    % (2*math.pi) - math.pi
)

with output.open('w') as file:
    file.write('# Simulation-only registered SLAM path\n')
    file.write(
        '# s_m;x_m;y_m;psi_rad;kappa_1pm;vx_mps;ax_mps2\n'
    )
    for row in registered:
        file.write(
            ';'.join(f'{value:.6f}' for value in row) + '\n'
        )

print('VALIDATION PASSED')
print(f'Saved: {output}')
