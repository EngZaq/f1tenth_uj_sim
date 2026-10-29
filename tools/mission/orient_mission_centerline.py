#!/usr/bin/env python3
"""Orient a SLAM centerline to the mapping-lap finish heading; no oracle coordinates.
Use sampled geometry; recalculates arc length and tangent headings after reversing.
"""
import argparse
import json
import math
from pathlib import Path
import numpy as np


def orient(path,pose,out):
    raw=np.loadtxt(path,comments='#',delimiter=';')
    if raw.ndim != 2 or raw.shape[1] < 7 or len(raw)<40:
        raise ValueError('Unexpected generated centerline (need 7 columns, >=40 points)')
    x,y,h=float(pose['finish_x']),float(pose['finish_y']),float(pose['finish_yaw'])
    n=len(raw)
    start=int(np.argmin((raw[:,1]-x)**2+(raw[:,2]-y)**2))
    if math.hypot(raw[start,1]-x,raw[start,2]-y)>0.60:
        raise ValueError('Finish pose is >0.60 m from extracted centerline; do not race')
    # Determine direction using tangent of the nearest segment.
    nxt=raw[(start+1)%n,1:3]-raw[(start-1)%n,1:3]
    heading=np.array([math.cos(h),math.sin(h)])
    forward=float(nxt@heading)>=0.0
    indices=(start+np.arange(n))%n if forward else (start-np.arange(n))%n
    arr=raw[indices,:7].copy()
    xy=arr[:,1:3]
    segs=np.linalg.norm(np.roll(xy,-1,axis=0)-xy,axis=1)
    if min(segs)<=1e-5: raise ValueError('Duplicate points in extracted centerline')
    arr[:,0]=np.r_[0.0,np.cumsum(segs[:-1])]
    tangent=np.roll(xy,-1,axis=0)-np.roll(xy,1,axis=0)
    arr[:,3]=np.arctan2(tangent[:,1],tangent[:,0])
    # Curvature from periodic geometry; sign follows new travel direction.
    xp=(np.roll(xy,-1,axis=0)-np.roll(xy,1,axis=0))/2.0
    xpp=np.roll(xy,-1,axis=0)-2*xy+np.roll(xy,1,axis=0)
    arr[:,4]=(xp[:,0]*xpp[:,1]-xp[:,1]*xpp[:,0])/np.maximum((xp[:,0]**2+xp[:,1]**2)**1.5,1e-9)
    arr[:,6]=0.0  # no acceleration feed-forward used by mission controller
    out=Path(out)
    out.parent.mkdir(parents=True,exist_ok=True)
    with out.open('w') as f:
        f.write('# s_m;x_m;y_m;psi_rad;kappa_1pm;vx_mps;ax_mps2 (slam_map frame)\n')
        for row in arr:
            f.write(';'.join(f'{float(v):.6f}' for v in row)+'\n')
    yaw_err=math.degrees(math.atan2(math.sin(arr[0,3]-h),math.cos(arr[0,3]-h)))
    if abs(yaw_err)>55:
        out.unlink(missing_ok=True)
        raise ValueError(f'Path tangent differs from robot heading by {yaw_err:.1f}deg')
    print(f'Oriented {n} points; direction={"forward" if forward else "reversed"}; '
          f'nearest={start}; starting-heading error={yaw_err:.1f}deg; saved {out}')

if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('path')
    p.add_argument('pose_json')
    p.add_argument('output')
    a=p.parse_args()
    orient(a.path,json.loads(Path(a.pose_json).read_text()),a.output)
