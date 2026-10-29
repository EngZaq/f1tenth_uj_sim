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
from navigation.pose_uncertainty import planar_covariance


def stopping_horizon(velocity, reaction=.35, deceleration=.7, buffer=.10):
    if (not all(math.isfinite(v) for v in (velocity,reaction,deceleration,buffer)) or
            reaction<0 or deceleration<=0 or buffer<.10):
        raise ValueError('Invalid stopping-horizon inputs')
    speed=max(0.,float(velocity))
    return speed*reaction+speed*speed/(2*deceleration)+buffer


def command_swept_collision(ranges, angle_min, angle_increment, range_min, range_max,
                            steering, velocity, initial_steering=None,
                            scan_to_body=(LASER['x_m'],LASER['y_m'],LASER['yaw_rad']),
                            wheelbase=.3302,front=.46145,rear=-.11855,halfwidth=.155,
                            margin=.07,deceleration=.7,reaction=.35,
                            speed_std=.01,yaw_rate_std=.01):
    """Check a commanded Ackermann stopping tube against local scan endpoints.

    scan_to_body maps laser-at-acquisition into body-at-rollout-start, normally
    obtained through local odometry. No map pose or global covariance enters.
    The tube allows any steering transition between measured and requested
    curvature, and a 3-sigma local speed/yaw-rate error over the stopping time.
    These are simulation assumptions, not an independently certified brake.
    Returns (blocked, minimum_tube_adjusted_gap_m, valid_hit_count).
    """
    initial_steering=steering if initial_steering is None else initial_steering
    parameters=(angle_min,angle_increment,range_min,range_max,steering,velocity,
                initial_steering,wheelbase,front,rear,halfwidth,margin,
                speed_std,yaw_rate_std,*scan_to_body)
    if (not all(math.isfinite(v) for v in parameters) or angle_increment<=0 or
            range_min<0 or range_max<=range_min or velocity<0 or wheelbase<=0 or
            rear>=front or halfwidth<=0 or margin<=0 or speed_std<0 or
            yaw_rate_std<0 or max(abs(steering),abs(initial_steering))>.419):
        raise ValueError('Invalid local swept-tube inputs')
    horizon=stopping_horizon(velocity,reaction,deceleration)
    ranges=np.asarray(ranges,dtype=float)
    if ranges.ndim!=1 or ranges.size<50:
        raise ValueError('Invalid LiDAR input shape')
    angles=angle_min+np.arange(ranges.size)*angle_increment
    valid=(np.isfinite(ranges)&(ranges>=max(.08,range_min))&
           (ranges<=min(range_max-.02,12.)))
    count=int(valid.sum())
    if count<35:
        return True,0.,count
    tx,ty,rotation=scan_to_body
    bearings=angles[valid]+rotation
    hits=np.column_stack((tx+ranges[valid]*np.cos(bearings),
                          ty+ranges[valid]*np.sin(bearings)))
    curvature=math.tan(steering)/wheelbase
    curvature_change=abs(curvature-math.tan(initial_steering)/wheelbase)
    corner_radius=max(math.hypot(front,halfwidth),math.hypot(rear,halfwidth))
    samples=max(1,math.ceil(horizon/.02))
    spacing=horizon/samples
    nearest=float('inf')
    for distance in np.linspace(0.,horizon,samples+1):
        heading=curvature*distance
        if abs(curvature)<1e-9:
            x,y=distance,0.
        else:
            x,y=math.sin(heading)/curvature,(1-math.cos(heading))/curvature
        c,s=math.cos(heading),math.sin(heading)
        dx,dy=hits[:,0]-x,hits[:,1]-y
        lon,lat=c*dx+s*dy,-s*dx+c*dy
        edge_x=np.maximum(np.maximum(rear-lon,lon-front),0.)
        edge_y=np.maximum(np.abs(lat)-halfwidth,0.)
        # If curvature remains between the initial and requested values,
        # |heading error|<=distance*delta_k and |centre error|<=s²*delta_k/2.
        transition=(.5*distance**2+corner_radius*distance)*curvature_change
        elapsed=min(distance/max(velocity,.05),reaction+velocity/deceleration)
        local_noise=3*elapsed*(speed_std+yaw_rate_std*(corner_radius+.5*distance))
        # Cover poses between adjacent samples using a rigid-body motion bound.
        sampling=.5*spacing*(1+corner_radius*abs(curvature))
        gap=float(np.min(np.hypot(edge_x,edge_y)))-transition-local_noise-sampling
        nearest=min(nearest,gap)
    return nearest<margin,nearest,count


def swept_path_collision(ranges, angle_min, angle_increment, range_min, range_max,
                         route, pose, nearest_idx, velocity,
                         wheelbase=0.3302, laser_x=LASER['x_m'],
                         front=0.46145, rear=-0.11855, halfwidth=0.155,
                         margin=0.07, deceleration=0.7, reaction=0.35,
                         laser_y=LASER['y_m'], laser_yaw=LASER['yaw_rad'],
                         pose_covariance=None, uncertainty_sigma=3.):
    """Offline reference-route analysis; runtime uses command_swept_collision.

    route Nx4 columns s,x,y,yaw; pose=(x,y,yaw) in route map frame.

    Returns (blocked, minimum_uncertainty_adjusted_body_gap_m, valid_hit_count).
    Uses measured endpoints only; requires independent scan freshness and
    vehicle emergency braking. Map and localization errors need separate margin.
    """
    route = np.asarray(route)
    covariance=None if pose_covariance is None else planar_covariance(pose_covariance)
    if (not all(math.isfinite(v) for v in
                (angle_min,angle_increment,range_min,range_max,velocity)) or
            angle_increment<=0 or range_min<0 or range_max<=range_min):
        raise ValueError('Invalid LiDAR geometry or speed')
    if not math.isfinite(uncertainty_sigma) or uncertainty_sigma<=0:
        raise ValueError('Invalid pose uncertainty multiplier')
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
    # The rectangle already includes the full front/rear overhang. Check the
    # distance travelled during reaction and braking plus a sampling buffer;
    # do not count another vehicle-length-scale extension as stopping distance.
    horizon = stopping_horizon(speed,reaction,deceleration)

    base_s = float(route[nearest_idx, 0])
    closed_length = float(route[-1,0])+float(np.linalg.norm(route[-1,1:3]-route[0,1:3]))
    x0, y0, yaw0 = pose
    c0, s0 = math.cos(yaw0), math.sin(yaw0)
    footprints = [(0.0, 0.0, 0.0)]
    for offset in range(min(len(route), 100)):
        i = (nearest_idx + offset) % len(route)
        ds = (float(route[i, 0]) - base_s) % max(closed_length, 0.1)
        dx, dy = route[i, 1] - x0, route[i, 2] - y0
        heading = math.atan2(math.sin(route[i, 3] - yaw0),
                             math.cos(route[i, 3] - yaw0))
        footprints.append((c0 * dx + s0 * dy,
                           -s0 * dx + c0 * dy, heading))
        # Include the first waypoint beyond the horizon to cover the whole
        # stopping interval despite discrete route sampling.
        if offset>0 and ds>=horizon:
            break

    nearest = float('inf')
    for footprint_index,(x, y, heading) in enumerate(footprints):
        dx, dy = hits[:, 0] - x, hits[:, 1] - y
        c, s = math.cos(heading), math.sin(heading)
        lon, lat = c * dx + s * dy, -s * dx + c * dy
        edge_x = np.maximum(np.maximum(rear - lon, lon - front), 0.0)
        edge_y = np.maximum(np.abs(lat) - halfwidth, 0.0)
        distances=np.hypot(edge_x,edge_y)
        if covariance is not None and footprint_index>0:
            # The current body and scan share a frame, so global pose error
            # cancels for footprint 0. Future route rectangles are fixed in
            # the map. Propagate scan endpoint placement uncertainty onto the
            # rectangle's separating plane, including position/yaw coupling.
            dl=lon-np.clip(lon,rear,front)
            dt=lat-np.clip(lat,-halfwidth,halfwidth)
            norm=np.maximum(distances,1e-12)
            nl,nt=dl/norm,dt/norm
            ch,sh=math.cos(yaw0+heading),math.sin(yaw0+heading)
            nx,ny=ch*nl-sh*nt,sh*nl+ch*nt
            j_yaw=nx*(-s0*hits[:,0]-c0*hits[:,1])+ny*(c0*hits[:,0]-s0*hits[:,1])
            jacobian=np.column_stack((nx,ny,j_yaw))
            variance=np.einsum('ij,jk,ik->i',jacobian,covariance,jacobian)
            bound=uncertainty_sigma*np.sqrt(np.maximum(variance,0.))
            # Bound the omitted second-order rotation term over the same
            # yaw interval, rather than relying on linearization alone.
            yaw_interval=uncertainty_sigma*math.sqrt(max(0.,float(covariance[2,2])))
            bound+=.5*np.linalg.norm(hits,axis=1)*yaw_interval**2
            distances=distances-bound
        gap = float(np.min(distances))
        nearest = min(nearest, gap)
        if gap < margin:
            return True, nearest, int(valid.sum())
    return False, nearest, int(valid.sum())
