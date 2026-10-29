"""Shared simulation laser calibration; never a simulator pose source."""
import math
from pathlib import Path
import yaml

CONFIG = Path(__file__).resolve().parents[1]/'config/sim_laser_geometry.yaml'


def load_laser_geometry(path=CONFIG):
    data = yaml.safe_load(Path(path).read_text())
    for key in ('x_m', 'y_m', 'yaw_rad'):
        data[key] = float(data[key])
        if not math.isfinite(data[key]):
            raise ValueError('Laser geometry must be finite')
    for key in ('reference_body_frame', 'reference_scan_frame'):
        if not isinstance(data[key], str) or not data[key]:
            raise ValueError('Missing reference laser frame')
    return data


LASER = load_laser_geometry()


def validate_reference_transform(transform):
    q = transform.rotation
    heading = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    values = (transform.translation.x,transform.translation.y,q.x,q.y,q.z,q.w)
    if not all(math.isfinite(v) for v in values):
        raise RuntimeError('Nonfinite simulator laser transform')
    if abs(q.x)>1e-4 or abs(q.y)>1e-4:
        raise RuntimeError('Nonplanar laser mounting is unsupported by the 2-D relay')
    if (abs(transform.translation.x-LASER['x_m']) > 1e-4 or
            abs(transform.translation.y-LASER['y_m']) > 1e-4 or
            abs(math.atan2(math.sin(heading-LASER['yaw_rad']),
                           math.cos(heading-LASER['yaw_rad']))) > 1e-4):
        raise RuntimeError('Simulator laser TF disagrees with config/sim_laser_geometry.yaml')


def laser_points(ranges, bearings, scale=1.0):
    # NumPy arrays are supported without importing NumPy in this module.
    import numpy as np
    return np.column_stack((LASER['x_m']+scale*ranges*np.cos(bearings+LASER['yaw_rad']),
                            LASER['y_m']+scale*ranges*np.sin(bearings+LASER['yaw_rad'])))
