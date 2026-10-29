#!/usr/bin/env python3
"""Offline F1TENTH AMCL diagnosis in the known simulator/map registration.

No ROS node, no navigation commands, and no modifications to user files.
The simulator-truth timestamp in the paired CSV has already been interpolated
by the paired monitor to AMCL's header stamp. A nonzero best shift is therefore
only a diagnostic: it does not establish transport latency by itself.
"""
import argparse
import csv
from pathlib import Path
import numpy as np


def load_centerline(path):
    out = []
    with path.open(newline='') as f:
        for row in csv.reader(f, delimiter=';'):
            if not row or row[0].strip().startswith('#'):
                continue
            try:
                vals = [float(x) for x in row[:3]]
            except (ValueError, TypeError):
                continue
            if len(vals) == 3:
                out.append(vals[1:3])
    if not out:
        raise ValueError(f'No valid [s,x,y] rows in {path}')
    return np.asarray(out)


def rigid(src, dst):
    A = src - src.mean(0)
    B = dst - dst.mean(0)
    U, _, Vt = np.linalg.svd(A.T @ B)
    D = np.diag([1.0, np.linalg.det(U @ Vt)])
    R = U @ D @ Vt
    shift = dst.mean(0) - src.mean(0) @ R
    return R, shift


def rms_vectors(x):
    return float(np.sqrt(np.mean(np.sum(x * x, axis=1))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', type=Path, default=Path.home() / 'f1tenth_sim')
    args = ap.parse_args()
    root = args.root
    src = load_centerline(root / 'reference/generated/uj_technical_slam_01_centerline.csv')
    dst = load_centerline(root / 'reference/generated/uj_technical_slam_01_registered.csv')
    if src.shape != dst.shape:
        raise ValueError('Original and registered paths have different shapes; no known correspondence.')
    R, offset = rigid(src, dst)
    fit_error = rms_vectors(src @ R + offset - dst)
    if fit_error > 0.01:
        raise ValueError(f'Unexpectedly high map-registration residual: {fit_error:.4f}m')

    csv_path = root / 'results/localization/fusion_amcl_paired.csv'
    with csv_path.open(newline='') as f:
        rows = list(csv.DictReader(f))
    required = ('elapsed_s', 'true_x', 'true_y', 'amcl_x', 'amcl_y')
    if not rows or any(k not in rows[0] for k in required):
        raise ValueError(f'Missing required data columns in {csv_path}')
    t = np.asarray([float(r['elapsed_s']) for r in rows])
    truth = np.asarray([[float(r['true_x']), float(r['true_y'])] for r in rows])
    amcl = np.asarray([[float(r['amcl_x']), float(r['amcl_y'])] for r in rows])
    if len(t) < 40:
        raise ValueError('Need at least 40 AMCL/truth pairs')
    inds = np.argsort(t)
    t, truth, amcl = t[inds], truth[inds], amcl[inds]
    if np.any(np.diff(t) <= 0):
        raise ValueError('Non-unique or non-increasing time values in CSV')
    aligned = amcl @ R + offset
    residual = aligned - truth

    # Local path tangent, signed along-track and cross-track residual.
    velocity = np.gradient(truth, t, axis=0)
    speed = np.linalg.norm(velocity, axis=1)
    good = speed > 0.25
    tangent = np.zeros_like(velocity)
    tangent[good] = velocity[good] / speed[good, None]
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    along = np.sum(residual * tangent, axis=1)
    cross = np.sum(residual * normal, axis=1)

    print('=== AMCL MAP-FRAME / TIMING DIAGNOSTIC ===')
    print(f'Recording: {csv_path}')
    print(f'Samples: {len(t)} | duration: {t[-1]-t[0]:.2f} s')
    print(f'Independent path registration error: {fit_error:.5f} m')
    print(f'Map->sim registration rotation: {np.degrees(np.arctan2(R[0,1], R[0,0])):+.3f} deg')
    print(f'Map->sim translation: {np.array2string(offset, precision=4)} m')
    print(f'Position RMSE (map registration, no extra fit): {rms_vectors(residual):.3f} m')
    print(f'Mean position residual X/Y: {np.array2string(residual.mean(0), precision=4)} m')
    if good.sum() >= 20:
        print(f'Mean signed along-track offset: {along[good].mean():+.3f} m')
        print(f'Along-track RMS: {np.sqrt(np.mean(along[good]**2)):.3f} m')
        print(f'Mean signed cross-track offset: {cross[good].mean():+.3f} m')
        print(f'Cross-track RMS: {np.sqrt(np.mean(cross[good]**2)):.3f} m')
        print(f'Median truth speed: {np.median(speed[good]):.3f} m/s')

    # All candidate shifts use exactly the same interior samples.
    # delta: compare AMCL pose stamped at t against truth(t + delta).
    # A delayed AMCL pose would generally give a NEGATIVE best delta.
    margin = 0.55
    mask = (t >= t[0] + margin) & (t <= t[-1] - margin)
    te = t[mask]
    Ae = aligned[mask]
    if len(te) < 20:
        raise ValueError('Not enough samples for shift sweep')
    lags = np.arange(-0.50, 0.5001, 0.01)
    direct = []
    translation_only = []
    for delta in lags:
        B = np.column_stack((np.interp(te+delta, t, truth[:,0]),
                             np.interp(te+delta, t, truth[:,1])))
        error = Ae - B
        direct.append(rms_vectors(error))
        translation_only.append(rms_vectors(error - error.mean(0)))
    direct = np.asarray(direct)
    translation_only = np.asarray(translation_only)
    idx = int(np.argmin(direct))
    idx_tr = int(np.argmin(translation_only))
    zero = int(np.argmin(np.abs(lags)))
    print('\n--- Fixed map->sim frame; NO additional fit ---')
    print(f'Zero-shift RMSE (common interior): {direct[zero]:.3f} m')
    print(f'Best time shift: {lags[idx]:+.2f} s; RMSE: {direct[idx]:.3f} m')
    print(f'Improvement from shift alone: {direct[zero]-direct[idx]:.3f} m')
    print('\n--- Exploratory extra constant X/Y fit (ground truth; offline ONLY) ---')
    print(f'Zero-shift RMSE after removing mean X/Y residual: {translation_only[zero]:.3f} m')
    print(f'Best time shift after removing mean residual: {lags[idx_tr]:+.2f} s; RMSE: {translation_only[idx_tr]:.3f} m')
    print('\nInterpretation:')
    print('* A shift close to zero suggests timestamp/lag is not the dominant error.')
    print('* A nonzero best shift suggests temporal misalignment OR pose filtering/dynamics; not proof of latency.')
    print('* Large cross-track RMS or residual after shift points to map/scan geometry or pose-estimation error.')
    print('* The full-run shift and X/Y fit use ground truth and are diagnostic only; never send them to navigation.')


if __name__ == '__main__':
    main()
