#!/usr/bin/env python3
"""Extract a map-derived closed centerline from an observed track corridor.

Unlike the old strict extractor, tolerates spurs and junctions in the skeleton.
Requires a genuine observed free-space loop around a substantial enclosed hole.
Does NOT turn unknown cells into free space or infer an unseen map section.

Output names/formats intentionally match extract_slam_centerline.py.
"""
import argparse
import csv
import math
from collections import deque
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy import ndimage as ndi
from scipy.interpolate import CubicSpline
from skimage.morphology import skeletonize


def raster_mask(path):
    meta = yaml.safe_load(Path(path).read_text())
    source = Path(meta['image'])
    if not source.is_absolute():
        source = Path(path).parent / source
    gray = np.asarray(Image.open(source).convert('L'))
    resolution = float(meta['resolution'])
    ox, oy, angle = map(float, meta['origin'])
    # nav2_map_saver PGM: 254=known free, 205=unknown, 0=occupied.
    if gray.ndim != 2 or resolution <= 0:
        raise ValueError('Invalid occupancy map')
    if int(meta.get('negate', 0)) != 0:
        raise ValueError('Unsupported negated occupancy image')
    free = gray >= 250
    labels, num = ndi.label(free, np.ones((3, 3), bool))
    if not num:
        raise ValueError('Map contains no observed free-space region')
    sizes = np.bincount(labels.ravel()); sizes[0] = 0
    region = labels == np.argmax(sizes)
    return meta, gray, region, resolution, ox, oy, angle


def adjacency_from_pixels(pixels):
    nodes = set(map(tuple, pixels))
    adj = {}
    for r, c in nodes:
        nb = set()
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                j = (r + dr, c + dc)
                if j not in nodes:
                    continue
                # Suppress shortcut diagonals that produce spurious triangles.
                # The full pixel connectivity remains via the cardinal neighbor.
                if dr and dc and ((r + dr, c) in nodes or
                                  (r, c + dc) in nodes):
                    continue
                nb.add(j)
        adj[(r,c)] = nb
    return adj


def prune_spurs(adjacency):
    adj = {k: set(v) for k,v in adjacency.items()}
    leaves = deque(k for k,v in adj.items() if len(v) < 2)
    while leaves:
        node = leaves.popleft()
        if node not in adj or len(adj[node]) >= 2:
            continue
        for neighbor in list(adj[node]):
            adj[neighbor].discard(node)
            if len(adj[neighbor]) < 2:
                leaves.append(neighbor)
        del adj[node]
    return adj


def face_cycles(adj):
    """Walk embedded graph faces, including the external boundary.

    8-neighbor pixel diagonals that cross cardinal routes were removed first.
    Faces with repeated vertices (bridges) are discarded.
    """
    order = {}
    for u, nb in adj.items():
        order[u] = sorted(nb,
                          key=lambda v: math.atan2(v[0]-u[0], v[1]-u[1]))
    seen = set()
    faces = []
    for u in adj:
        for v in adj[u]:
            if (u,v) in seen:
                continue
            start = (u,v)
            edge = start
            route = []
            while edge not in seen:
                seen.add(edge)
                a, b = edge
                route.append(a)
                nbr = order.get(b, [])
                if not nbr or a not in nbr:
                    break
                idx = nbr.index(a)
                # Consistent turn rule visits the face to one side of each edge.
                next_b = nbr[(idx - 1) % len(nbr)]
                edge = (b,next_b)
            if edge == start and len(route) >= 20 and len(set(route)) == len(route):
                area = 0.5*sum(
                    a[1]*b[0] - b[1]*a[0]
                    for a,b in zip(route, route[1:]+route[:1])
                )
                faces.append((abs(area),route))
    return sorted(faces, reverse=True, key=lambda p:p[0])


def inside_polygon(point, route):
    # Ray casting with image coordinate x=column, y=row.
    y, x = point
    inside=False
    for a,b in zip(route, route[1:]+route[:1]):
        ya,xa=a; yb,xb=b
        if ((ya>y)!=(yb>y)):
            xx = xa+(xb-xa)*(y-ya)/(yb-ya)
            if xx>x:
                inside = not inside
    return inside


def choose_cycle(region, skeleton, resolution, expected_length):
    holes = ndi.binary_fill_holes(region) & ~region
    labels, num = ndi.label(holes, structure=np.ones((3,3)))
    candidates=[]
    for k in range(1,num+1):
        area = int(np.sum(labels==k))
        if area * resolution**2 < 0.5:
            continue
        center = ndi.center_of_mass(labels==k)
        candidates.append((area,center))
    if not candidates:
        raise ValueError('No substantial enclosed interior in observed free space. '
                         'Map is probably open/incomplete: cannot safely construct a loop.')
    candidates.sort(reverse=True,key=lambda x:x[0])
    center = candidates[0][1]
    adj = prune_spurs(adjacency_from_pixels(np.argwhere(skeleton)))
    faces=face_cycles(adj)
    expected_pixels = expected_length/resolution if expected_length else None
    valid=[]
    for area, cycle in faces:
        if not inside_polygon(center,cycle):
            continue
        lengths = sum(math.hypot(b[0]-a[0],b[1]-a[1])
                      for a,b in zip(cycle,cycle[1:]+cycle[:1]))
        length = lengths*resolution
        if length < 8:
            continue
        if expected_length and not (0.55*expected_length <= length <= 1.55*expected_length):
            continue
        valid.append((area,length,cycle))
    if not valid:
        raise ValueError(f'Could not find closed centerline around enclosed interior. '
                         f'Skeleton points={int(skeleton.sum())}, 2-core={len(adj)}, '
                         f'candidate faces={len(faces)}; map may have holes or crossings.')
    # Largest enclosed face corresponds to boundary cycle of the main track skeleton.
    valid.sort(reverse=True,key=lambda x:x[0])
    area,length,route=valid[0]
    print(f'Skeleton pixels: {int(skeleton.sum())}; after pruning spurs: {len(adj)}; '
          f'candidate cycles: {len(faces)}; selected length={length:.2f}m')
    return route


def extract(map_yaml, output_prefix, step, max_speed, expected_length):
    meta,pixels,region,res,ox,oy,angle=raster_mask(map_yaml)
    sk=skeletonize(region)
    route=choose_cycle(region,sk,res,expected_length)
    height,width=pixels.shape
    rc=np.asarray(route,float)
    # Note route includes the same starting point ONLY implicitly.
    # Convert pixels to world coordinates, respecting ROS map origin rotation.
    col,row=rc[:,1],rc[:,0]
    u=(col+0.5)*res
    v=(height-row-0.5)*res
    x=ox+math.cos(angle)*u-math.sin(angle)*v
    y=oy+math.sin(angle)*u+math.cos(angle)*v
    x=ndi.gaussian_filter1d(x,2.0,mode='wrap')
    y=ndi.gaussian_filter1d(y,2.0,mode='wrap')
    closed=np.vstack([np.column_stack((x,y)),[x[0],y[0]]])
    ds=np.linalg.norm(np.diff(closed,axis=0),axis=1)
    if min(ds)<1e-5:
        raise ValueError('Repeated points in extracted skeleton')
    cumulative=np.r_[0.,np.cumsum(ds)]
    length=float(cumulative[-1])
    count=max(40,int(round(length/step)))
    s=np.linspace(0,length,count,endpoint=False)
    fx=CubicSpline(cumulative,closed[:,0],bc_type='periodic')
    fy=CubicSpline(cumulative,closed[:,1],bc_type='periodic')
    xs,ys=fx(s),fy(s)
    dx,dy=fx(s,1),fy(s,1)
    ddx,ddy=fx(s,2),fy(s,2)
    headings=np.arctan2(dy,dx)
    kappa=(dx*ddy-dy*ddx)/np.maximum((dx*dx+dy*dy)**1.5,1e-9)
    # Safety check ALL interpolated path segments at <=2.5cm intervals.
    xy=np.column_stack((xs,ys))
    fine=[]
    for a,b in zip(xy,np.roll(xy,-1,axis=0)):
        n=max(2,int(math.ceil(np.linalg.norm(b-a)/0.025)))
        fine.extend(a+(b-a)*(j/n) for j in range(n))
    dense=np.asarray(fine)
    def coord(p):
        rx=p[:,0]-ox; ry=p[:,1]-oy
        uu= math.cos(angle)*rx+math.sin(angle)*ry
        vv=-math.sin(angle)*rx+math.cos(angle)*ry
        return (height-1-np.floor(vv/res).astype(int),
                np.floor(uu/res).astype(int))
    rr,cc=coord(dense)
    valid=(rr>=0)&(rr<height)&(cc>=0)&(cc<width)
    if not np.all(valid):
        raise ValueError('Extracted centerline exits occupancy map')
    if not np.all(region[rr,cc]):
        raise ValueError('Extracted centerline intersects occupied or unknown map cells')
    clear=ndi.distance_transform_edt(region)*res
    min_clear=float(clear[rr,cc].min())
    if min_clear<0.30:
        raise ValueError(f'Insufficient centerline clearance {min_clear:.3f}m (<0.30m)')
    if expected_length and not (0.55*expected_length <= length <= 1.55*expected_length):
        raise ValueError(f'Path length {length:.1f}m does not match traversed '
                         f'{expected_length:.1f}m; review map')
    segments=np.linalg.norm(np.roll(xy,-1,axis=0)-xy,axis=1)
    if min(segments)<1e-5:
        raise ValueError('Invalid resampled path spacing')
    accel,brake,latacc=0.5,0.8,0.9
    raw_speed=np.minimum(max_speed,np.sqrt(latacc/np.maximum(np.abs(kappa),1e-3)))
    speed=raw_speed.copy()
    for _ in range(30):
        for i in range(count-1,-1,-1):
            j=(i+1)%count
            speed[i]=min(speed[i],math.sqrt(speed[j]**2+2*brake*segments[i]))
        for i in range(count):
            j=(i+1)%count
            speed[j]=min(speed[j],math.sqrt(speed[i]**2+2*accel*segments[i]))
    ax=(np.roll(speed,-1)**2-speed**2)/(2*segments+1e-9)
    prefix=Path(output_prefix)
    prefix.parent.mkdir(parents=True,exist_ok=True)
    pathfile=prefix.with_name(prefix.name+'_centerline.csv')
    speedfile=prefix.with_name(prefix.name+'_speed_profile.csv')
    preview=prefix.with_name(prefix.name+'_preview.png')
    with pathfile.open('w') as f:
        f.write('# Coordinates generated in slam_map frame\n')
        f.write('# s_m;x_m;y_m;psi_rad;kappa_1pm;vx_mps;ax_mps2\n')
        for z in zip(s,xs,ys,headings,kappa,speed,ax):
            f.write(';'.join(f'{float(q):.6f}' for q in z)+'\n')
    with speedfile.open('w',newline='') as f:
        writer=csv.writer(f)
        writer.writerow(['index','s_m','x_m','y_m','curvature_1pm',
                         'raw_speed_mps','planned_speed_mps'])
        for i in range(count):
            writer.writerow([i,f'{s[i]:.4f}',f'{xs[i]:.4f}',f'{ys[i]:.4f}',
                             f'{kappa[i]:.5f}',f'{raw_speed[i]:.3f}',f'{speed[i]:.3f}'])
    image=Image.fromarray(pixels).convert('RGB')
    image=image.resize((width*4,height*4),Image.Resampling.NEAREST)
    rr0,cc0=coord(xy)
    draw=ImageDraw.Draw(image)
    pix=list(zip((cc0+0.5)*4,(rr0+0.5)*4))
    draw.line(pix+[pix[0]],fill=(0,170,250),width=3)
    image.save(preview)
    print(f'VALIDATED: {count} points, path length {length:.2f}m, '
          f'minimum OBSERVED free-space clearance {min_clear:.3f}m')
    print('Saved:',pathfile,speedfile,preview,sep='\n  ')


def main():
    p=argparse.ArgumentParser()
    p.add_argument('map_yaml')
    p.add_argument('--out',required=True)
    p.add_argument('--step',type=float,default=0.10)
    p.add_argument('--max-speed',type=float,default=0.35)
    p.add_argument('--expected-length',type=float,default=0)
    a=p.parse_args()
    extract(a.map_yaml,a.out,a.step,a.max_speed,a.expected_length)

if __name__=='__main__':
    main()
