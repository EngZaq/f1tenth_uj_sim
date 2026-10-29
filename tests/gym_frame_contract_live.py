"""Check actual pinned Gym dynamics, collision and laser reference points.

Requires the real Gym package, not ROS or a simulator ground-truth controller.
"""
import math
import sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from sim_geometry import LASER, FUSION_LASER, REAR_TO_COG, rear_axle_pose, BODY_FRONT, BODY_REAR
from f1tenth_gym.envs.dynamic_models import DynamicModel,get_f1tenth_vehicle_parameters
from f1tenth_gym.envs.dynamic_models.single_track import vehicle_dynamics_st
from f1tenth_gym.envs.simulator import F110Simulator as Simulator

p=get_f1tenth_vehicle_parameters()
assert abs(p.lr-REAR_TO_COG)<1e-9
assert abs(p.lf+p.lr-.3302)<1e-9
offset=Simulator._compute_collision_body_offset(p,DynamicModel.ST)
np.testing.assert_allclose(np.array(offset)+[p.lr,0.],
                           [LASER['rear_axle_to_body_center_x_m'],0.],atol=1e-9)
np.testing.assert_allclose([BODY_FRONT,BODY_REAR],
    [p.lr+offset[0]+p.length/2,p.lr+offset[0]-p.length/2],atol=1e-9)
fake=SimpleNamespace(config=SimpleNamespace(lidar_config=SimpleNamespace(
    base_link_to_lidar_tf=(LASER['x_m'],LASER['y_m'],LASER['yaw_rad']))))
for heading in (0.,.8,-1.7):
    cog=np.array([1.,2.,heading]);rear=rear_axle_pose(cog)
    actual_laser=Simulator._lidar_pose_from_base(fake,cog)
    fused_laser=np.array([rear[0]+FUSION_LASER[0]*math.cos(heading),
                          rear[1]+FUSION_LASER[0]*math.sin(heading),heading])
    np.testing.assert_allclose(actual_laser,fused_laser,atol=1e-9)
    for delta in (-.4,0.,.4):
        beta=math.atan(p.lr/(p.lr+p.lf)*math.tan(delta))
        state=np.array([1.,2.,delta,.3,heading,0.,beta])
        derivative=vehicle_dynamics_st(state,np.zeros(2),p.to_array(DynamicModel.ST))
        rear_velocity=derivative[:2]+p.lr*derivative[4]*np.array([math.sin(heading),-math.cos(heading)])
        forward=.3*math.cos(beta)
        np.testing.assert_allclose(rear_velocity,
            forward*np.array([math.cos(heading),math.sin(heading)]),atol=1e-7)
print('PASS: actual Gym CoG, rear-axle kinematics, collision body and scan origin agree')
