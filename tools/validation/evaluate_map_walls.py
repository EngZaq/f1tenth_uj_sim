#!/usr/bin/env python3
"""Offline SLAM map wall accuracy against the original simulator track (OFFLINE ONLY).

The reference track is used here for evaluation only; it never enters mapping or navigation.

Method (same idea as the team's no-loop baseline audit, wall RMSE 0.047 m):
  1. Observed walls = occupied cells of the saved SLAM map that share an edge with a
     known-free cell (4-neighbourhood), at the cell centres.
  2. Reference walls = the free/occupied pixel edges of the original track image
     (edge midpoints = the true wall surface, not half a pixel inside the wall).
  3. Rigid SE(2) registration (trimmed point-to-point ICP, coarse-to-fine rejection
     0.30 -> 0.03 m), started from the simulator start pose: the SLAM map frame starts
     at the car's rear axle at the start pose.
  4. Nearest-reference-wall distance of EVERY observed wall cell after registration:
     median, RMS, p95, max, % within 0.05 m and 0.10 m.
  5. Start/finish doubling (no loop closing: the start area is mapped twice, at the start
     and again at the end of the lap): wall cells within --start-radius of the start pose
     are explained by two rigid translations (two copies of the wall); the offset between
     them is reported when both copies hold >= 20 % of the cells and differ by >= 0.04 m.
     A translation along a straight wall is not observable; the fit is ridge-regularised.
  6. lap_pose.json (next to the map, if present): the mission's own return numbers.

Floor: wall cells are scored at their centres -> a perfect 0.025 m map scores ~0.007 m
(laser-hit cells straddle the surface) to 0.0125 m (cells fully behind the surface).

Usage:
  evaluate_map_walls.py results/missions/<run>/new_track.yaml
      [--reference maps/uj_technical_loop_v1.yaml] [--init X,Y,YAW_DEG]
      [--json out.json] [--npz out.npz] [--plot out.png]
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[2]
# Simulator start pose (f1tenth_gym_ros sim.yaml: sx -5.8, sy -2.1, stheta 2.1686 rad, CoG)
# moved to the rear axle (0.17145 m behind the CoG) = origin of the SLAM map frame.
DEFAULT_INIT = (-5.8 - 0.17145 * math.cos(2.1686), -2.1 - 0.17145 * math.sin(2.1686), math.degrees(2.1686))


def load_map(yaml_path):
    yaml_path = Path(yaml_path)
    meta = yaml.safe_load(yaml_path.read_text())
    img = np.array(Image.open(yaml_path.parent / meta['image']).convert('L')).astype(float)
    p = img / 255.0 if meta.get('negate', 0) else (255.0 - img) / 255.0
    occ = p > float(meta['occupied_thresh'])
    free = p < float(meta['free_thresh'])
    return occ, free, float(meta['resolution']), [float(v) for v in meta['origin'][:2]]


def cell_xy(rows, cols, shape, res, origin):
    return np.c_[origin[0] + (cols + 0.5) * res, origin[1] + (shape[0] - rows - 0.5) * res]


def observed_walls(occ, free, res, origin):
    # 4-neighbourhood: SLAM walls are ~1 cell thick and touch free space across an edge;
    # diagonal-only contacts would add cells lying deeper inside thick walls.
    near_free = ndimage.binary_dilation(free, structure=ndimage.generate_binary_structure(2, 1))
    r, c = np.nonzero(occ & near_free)
    return cell_xy(r, c, occ.shape, res, origin)


def reference_edges(occ, free, res, origin):
    pts = []
    H, W = occ.shape
    F = np.pad(free, 1, constant_values=False)
    for dr, dc in ((0, 1), (0, -1), (1, 0), (-1, 0)):
        nb_free = F[1 + dr:1 + dr + H, 1 + dc:1 + dc + W]       # free[r+dr, c+dc]
        r, c = np.nonzero(occ & nb_free)
        xy = cell_xy(r, c, occ.shape, res, origin)
        xy[:, 0] += 0.5 * res * dc
        xy[:, 1] -= 0.5 * res * dr
        pts.append(xy)
    return np.vstack(pts)


def se2(x, y, th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]]), np.array([x, y])


def icp(P, tree, Q, R, t, schedule=(0.30, 0.15, 0.08, 0.05, 0.03), iters=25):
    for gate in schedule:
        for _ in range(iters):
            X = P @ R.T + t
            d, j = tree.query(X)
            k = d < gate
            if k.sum() < 20:
                raise RuntimeError(f'ICP lost the walls at gate {gate} m (bad initial guess?)')
            A, B = X[k], Q[j[k]]
            ma, mb = A.mean(0), B.mean(0)
            U, _, Vt = np.linalg.svd((A - ma).T @ (B - mb))
            dR = (U @ Vt).T
            if np.linalg.det(dR) < 0:
                Vt[-1] *= -1
                dR = (U @ Vt).T
            dt = mb - dR @ ma
            R, t = dR @ R, dR @ t + dt
            if abs(math.atan2(dR[1, 0], dR[0, 0])) < 1e-6 and np.hypot(*dt) < 1e-6:
                break
    return R, t


def doubling(X, start_xy, tree, Q, radius):
    """Two-translation model of the start-area walls; returns dict (offset_m=None if not visible)."""
    k = np.hypot(*(X - start_xy).T) < radius
    Xs = X[k]
    out = {'start_region_cells': int(k.sum()), 'radius_m': radius, 'visible': False, 'offset_m': None}
    if len(Xs) < 40:
        return out
    _, j = tree.query(Xs)
    r = Xs - Q[j]
    # reference wall normal at the foot point: PCA of reference edge points within 0.075 m
    nrm = np.zeros_like(r)
    for i, q in enumerate(Q[j]):
        nb = Q[tree.query_ball_point(q, 0.075)]
        w, v = np.linalg.eigh(np.cov((nb - nb.mean(0)).T) if len(nb) > 2 else np.eye(2))
        nrm[i] = v[:, 0]
    N = nrm[:, :, None] * nrm[:, None, :]            # n n^T per cell

    def fit(sel):
        A = N[sel].sum(0) + 1e-3 * max(sel.sum(), 1) * np.eye(2)
        b = np.einsum('nij,nj->i', N[sel], r[sel])
        return np.linalg.solve(A, b)

    def err(d):
        return np.linalg.norm(r - np.einsum('nij,j->ni', N, d), axis=1)

    big = np.linalg.norm(r, axis=1) > 0.03
    if big.sum() < 10:
        return out
    dA, dB = np.zeros(2), fit(big)
    for _ in range(30):
        selB = err(dB) < err(dA)
        if selB.all() or not selB.any():
            break
        nA, nB = fit(~selB), fit(selB)
        if np.allclose(nA, dA, atol=1e-5) and np.allclose(nB, dB, atol=1e-5):
            break
        dA, dB = nA, nB
    selB = err(dB) < err(dA)
    fB = float(selB.mean())
    off = float(np.hypot(*(dB - dA)))
    out.update(copy_fractions=[round(1 - fB, 3), round(fB, 3)], copy_a_m=dA.round(4).tolist(), copy_b_m=dB.round(4).tolist(),
               offset_m=round(off, 4), residual_rms_after_two_copies_m=round(float(np.sqrt(np.mean(np.minimum(err(dA), err(dB)) ** 2))), 4),
               residual_rms_one_copy_m=round(float(np.sqrt(np.mean(np.linalg.norm(r, axis=1) ** 2))), 4))
    out['visible'] = bool(min(fB, 1 - fB) >= 0.20 and off >= 0.04)
    return out


def evaluate(map_yaml, reference_yaml, init, start_radius=2.0, start_in_map=(0.0, 0.0)):
    occ, free, res, origin = load_map(map_yaml)
    P = observed_walls(occ, free, res, origin)
    rocc, rfree, rres, rorigin = load_map(reference_yaml)
    Q = reference_edges(rocc, rfree, rres, rorigin)
    tree = cKDTree(Q)
    R0, t0 = se2(init[0], init[1], math.radians(init[2]))
    R, t = icp(P, tree, Q, R0, t0)
    X = P @ R.T + t
    d, _ = tree.query(X)
    th = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    result = {
        'map': str(map_yaml), 'reference': str(reference_yaml), 'wall_cells': int(len(P)),
        'registration': {'x': round(float(t[0]), 4), 'y': round(float(t[1]), 4), 'yaw_deg': round(th, 3),
                         'moved_from_initial_guess_m': round(float(np.hypot(*(t - t0))), 4),
                         'rotated_from_initial_guess_deg': round(th - init[2], 3)},
        'wall_error_m': {'median': round(float(np.median(d)), 4), 'rms': round(float(np.sqrt(np.mean(d ** 2))), 4),
                         'p95': round(float(np.percentile(d, 95)), 4), 'max': round(float(d.max()), 4),
                         'pct_within_0.05': round(float(100 * np.mean(d <= 0.05)), 2),
                         'pct_within_0.10': round(float(100 * np.mean(d <= 0.10)), 2)},
        # the SLAM map origin is the start pose (start_in_map = (0, 0) for mission maps)
        'start_finish_doubling': doubling(X, R @ np.asarray(start_in_map, float) + t, tree, Q, start_radius),
    }
    lap = Path(map_yaml).parent / 'lap_pose.json'
    if lap.is_file():
        L = json.loads(lap.read_text())
        result['lap_pose'] = {k: L.get(k) for k in ('return_distance_odom_m', 'return_yaw_error_deg', 'distance_odom_m',
                                                     'estimated_lap_length_m', 'duration_s')}
        if 'finish_x' in L:
            result['lap_pose']['slam_finish_distance_from_start_m'] = round(math.hypot(L['finish_x'], L['finish_y']), 4)
            result['lap_pose']['slam_finish_yaw_deg'] = round(math.degrees(L['finish_yaw']), 2)
    return result, dict(P=P, X=X, d=d, Q=Q, R=R, t=t)


def plot(res, arrays, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(9, 7))
    ax.scatter(*arrays['Q'].T, s=0.4, c='#b8b7b2', label='reference walls (original track)')
    sc = ax.scatter(*arrays['X'].T, s=1.5, c=arrays['d'], cmap='viridis', vmin=0, vmax=0.15, label='SLAM map walls (registered)')
    fig.colorbar(sc, ax=ax, label='distance to reference wall (m)', shrink=0.8)
    ax.plot(*arrays['t'], 'k*', ms=10, label='start (SLAM origin)')
    ax.set_aspect('equal')
    w = res['wall_error_m']
    ax.set_title(f'{Path(res["map"]).parent.name}\nwall RMS {w["rms"]:.4f} m · median {w["median"]:.4f} · p95 {w["p95"]:.4f} · max {w["max"]:.3f}',
                 loc='left', fontsize=10)
    lg = ax.legend(loc='lower left', fontsize=8, frameon=False)
    for h in list(getattr(lg, 'legend_handles', getattr(lg, 'legendHandles', [])))[:2]:
        h.set_sizes([20])
    fig.tight_layout()
    fig.savefig(out, dpi=130)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('map_yaml')
    ap.add_argument('--reference', default=str(ROOT / 'maps/uj_technical_loop_v1.yaml'))
    ap.add_argument('--init', default=','.join(f'{v:.4f}' for v in DEFAULT_INIT),
                    help='initial SLAM-map -> reference pose X,Y,YAW_DEG (default: sim start, rear axle)')
    ap.add_argument('--start-radius', type=float, default=2.0)
    ap.add_argument('--json')
    ap.add_argument('--npz')
    ap.add_argument('--plot')
    a = ap.parse_args()
    init = [float(v) for v in a.init.split(',')]
    res, arr = evaluate(a.map_yaml, a.reference, init, a.start_radius)
    text = json.dumps(res, indent=2)
    print(text)
    if a.json:
        Path(a.json).write_text(text + '\n')
    if a.npz:
        np.savez_compressed(a.npz, X=arr['X'], d=arr['d'], R=arr['R'], t=arr['t'])
    if a.plot:
        plot(res, arr, a.plot)


if __name__ == '__main__':
    try:
        main()
    except RuntimeError as exc:
        print('MAP EVALUATION FAILED: ' + str(exc), file=sys.stderr)
        raise SystemExit(1)
