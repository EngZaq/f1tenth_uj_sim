#!/usr/bin/env python3
"""SIM-ONLY startup scan-to-map seed in an already saved SLAM frame.

Does NOT consume simulator truth or publish motion commands. The old lap-finish
pose is merely a search REGION; it is never assumed to be the current pose.
"""
import argparse
import sys
import json
import math
from pathlib import Path
import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt, map_coordinates
from scipy.optimize import minimize
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sim_geometry import laser_points


def wrap(a):
    return (a + np.pi) % (2*np.pi) - np.pi


def load_map(yaml_file):
    meta=yaml.safe_load(Path(yaml_file).read_text())
    pic=Path(meta['image'])
    if not pic.is_absolute(): pic=Path(yaml_file).parent/pic
    pixels=np.array(Image.open(pic).convert('L'))
    occ=pixels < 80
    unknown=(pixels >= 80) & (pixels < 245)
    free=pixels >= 245
    res=float(meta['resolution']); ox,oy,ang=map(float,meta['origin'])
    return pixels,occ,unknown,free,(res,ox,oy,ang)


def pixels_at(x,y,metadata,shape):
    res,ox,oy,ang=metadata
    c,s=math.cos(ang),math.sin(ang)
    u=c*(x-ox)+s*(y-oy)
    v=-s*(x-ox)+c*(y-oy)
    return shape[0]-v/res-0.5,u/res-0.5


def scan_points(scan, subsample=4):
    a=scan.angle_min+np.arange(len(scan.ranges))*scan.angle_increment
    r=np.array(scan.ranges,dtype=float)
    valid=np.isfinite(r) & (r>=max(0.12,scan.range_min)) & (r<min(7.0,scan.range_max-0.05))
    aa=a[valid][::subsample]; rr=r[valid][::subsample]
    if len(aa)<40:
        raise ValueError(f'Not enough measured LiDAR hits: {len(aa)}')
    return laser_points(rr, aa), laser_points(rr, aa, scale=0.75)


def path_points(path_file):
    arr=np.loadtxt(str(path_file),delimiter=';',comments='#')
    return arr


def fit_seed(scan, map_yaml, path_file, lap_pose):
    pixels,occ,unknown,free,meta=load_map(map_yaml)
    shape=pixels.shape
    # Distance to occupied cells, truncated at 0.8 m.
    distance=distance_transform_edt(~occ)*meta[0]
    lidar,inside=scan_points(scan)
    trajectory=path_points(path_file)
    finish=np.array([lap_pose['finish_x'],lap_pose['finish_y']]); fh=lap_pose['finish_yaw']
    near=np.where(np.hypot(trajectory[:,1]-finish[0],trajectory[:,2]-finish[1])<1.45)[0]
    if len(near)<4:
        raise ValueError('Saved finish is not near recovered path; cannot seed')

    def transformed(local,p):
        c,s=math.cos(p[2]),math.sin(p[2])
        return p[0]+c*local[:,0]-s*local[:,1],p[1]+s*local[:,0]+c*local[:,1]

    def score(p):
        if abs(wrap(p[2]-fh)) > 0.9: return 10.0
        if math.hypot(p[0]-finish[0],p[1]-finish[1])>1.6: return 10.0
        wx,wy=transformed(lidar,p)
        rr,cc=pixels_at(wx,wy,meta,shape)
        d=map_coordinates(distance,[rr,cc],order=1,mode='constant',cval=1.0)
        free_at=map_coordinates(free.astype(float),[rr,cc],order=0,mode='constant',cval=0.)
        unknown_at=map_coordinates(unknown.astype(float),[rr,cc],order=0,mode='constant',cval=1.)
        # Map artifacts exist; robust truncation is intentional. Hits must be near walls.
        endpoint=float(np.mean(np.minimum(d,0.6)**2))
        # A LiDAR beam cannot pass through a known wall before its endpoint.
        ix,iy=transformed(inside,p)
        ir,ic=pixels_at(ix,iy,meta,shape)
        inside_occ=map_coordinates(occ.astype(float),[ir,ic],order=0,mode='constant',cval=1.)
        return endpoint + 0.035*np.mean(inside_occ) + 0.007*np.mean(unknown_at) + 0.0005*np.mean(~np.isfinite(d))

    candidates=[]
    for idx in near[::max(1,len(near)//25)]:
        row=trajectory[idx]
        if abs(wrap(row[3]-fh)) > 0.65: continue
        for rot in (-0.17,0.0,0.17):
            cand=np.array([row[1],row[2],wrap(row[3]+rot)])
            candidates.append((score(cand),cand))
    if not candidates:
        raise ValueError('No path-aligned pose candidates close to prior finish')
    candidates.sort(key=lambda item:item[0])
    refined=[]
    for _,candidate in candidates[:8]:
        result=minimize(score,candidate,method='Powell',bounds=[
            (finish[0]-1.6,finish[0]+1.6),
            (finish[1]-1.6,finish[1]+1.6),
            (fh-0.9,fh+0.9)],options={'maxiter':90,'xtol':1e-3,'ftol':1e-4})
        p=result.x; val=score(p)
        route_dist=np.min(np.hypot(trajectory[:,1]-p[0],trajectory[:,2]-p[1]))
        refined.append((val,route_dist,p))
    refined.sort(key=lambda t:t[0])
    best,route_error,pose=refined[0]
    if best>0.070:
        raise ValueError(f'Poor scan-to-map fit ({best:.4f}); refuse guess. View map/scans.')
    if route_error>0.30:
        raise ValueError(f'Seed is {route_error:.2f}m from path; refuse driving')
    if abs(wrap(pose[2]-fh))>0.85:
        raise ValueError('Seed yaw differs too much from previous-lap finish')
    return {'x':float(pose[0]),'y':float(pose[1]),'yaw':float(wrap(pose[2])),
            'match_score':float(best),'route_distance_m':float(route_error),
            'hint_x':float(finish[0]),'hint_y':float(finish[1]),'hint_yaw':float(fh),
            'candidates_tested':len(candidates)}


def main():
    ap=argparse.ArgumentParser(description='Live stationary scan match for saved-map AMCL initialization. No actuation.')
    ap.add_argument('--map',required=True);ap.add_argument('--path',required=True)
    ap.add_argument('--lap-pose',required=True);ap.add_argument('--output',required=True)
    ap.add_argument('--timeout',type=float,default=15.0)
    args=ap.parse_args()
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from rclpy.qos import qos_profile_sensor_data
    import time
    rclpy.init()
    node=Node('saved_map_scan_seed')
    received=[]
    node.create_subscription(LaserScan,'/fusion/scan',lambda msg:received.append(msg),qos_profile_sensor_data)
    until=time.monotonic()+args.timeout
    while len(received)<3 and time.monotonic()<until: rclpy.spin_once(node,timeout_sec=0.3)
    node.destroy_node();rclpy.shutdown()
    if not received: raise SystemExit('No /fusion/scan received')
    lap=json.loads(Path(args.lap_pose).read_text())
    seed=fit_seed(received[-1],args.map,args.path,lap)
    Path(args.output).write_text(json.dumps(seed,indent=2)+'\n')
    print('LIVE SCAN MATCH (simulation only):',json.dumps(seed,indent=2))
    print('This is a pose HYPOTHESIS; check AMCL pose before enabling driving.')


if __name__=='__main__': main()
