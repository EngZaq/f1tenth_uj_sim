"""Shared simulation laser calibration; never a simulator pose source."""
import math
from pathlib import Path
import yaml

CONFIG = Path(__file__).resolve().parents[1]/'config/sim_laser_geometry.yaml'


def load_laser_geometry(path=CONFIG):
    data = yaml.safe_load(Path(path).read_text())
    for key in ('x_m', 'y_m', 'yaw_rad','rear_axle_to_reference_body_x_m',
                'rear_axle_to_body_center_x_m','body_length_m','body_width_m'):
        data[key] = float(data[key])
        if not math.isfinite(data[key]):
            raise ValueError('Laser geometry must be finite')
    for key in ('reference_body_frame', 'reference_scan_frame'):
        if not isinstance(data[key], str) or not data[key]:
            raise ValueError('Missing reference laser frame')
    if data.get('fusion_body_reference')!='rear_axle':
        raise ValueError('This kinematic stack requires a rear-axle fusion frame')
    return data


LASER = load_laser_geometry()
REAR_TO_COG=LASER['rear_axle_to_reference_body_x_m']
FUSION_LASER=(REAR_TO_COG+LASER['x_m'],LASER['y_m'],LASER['yaw_rad'])
BODY_FRONT=LASER['rear_axle_to_body_center_x_m']+LASER['body_length_m']/2
BODY_REAR=LASER['rear_axle_to_body_center_x_m']-LASER['body_length_m']/2
BODY_HALFWIDTH=LASER['body_width_m']/2


def rear_axle_pose(cog_pose):
    """Pinned ST simulator state (CoG) -> kinematic rear axle; evaluation only."""
    x,y,yaw=cog_pose
    return x-REAR_TO_COG*math.cos(yaw),y-REAR_TO_COG*math.sin(yaw),yaw


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
    return np.column_stack((FUSION_LASER[0]+scale*ranges*np.cos(bearings+FUSION_LASER[2]),
                            FUSION_LASER[1]+scale*ranges*np.sin(bearings+FUSION_LASER[2])))
