#!/usr/bin/env python3
"""MOATASEM (Test 7): curvature-limited speed profile for an existing route geometry. SIM ONLY.

Reads a route CSV (s;x;y;psi;kappa;vx;ax, slam_map frame), keeps x, y, and writes a copy with
  kappa  = smoothed curvature from the geometry (heading unwrapped along s, periodic Gaussian smoothing),
  vx     = min(v_max, sqrt(a_lat / |kappa|)), then a forward pass (accel) and a backward pass (brake)
           on the closed loop until it converges,
  ax     = (v_{i+1}^2 - v_i^2) / (2 ds),
  psi    = smoothed heading, s = arc length (unchanged geometry).
The grid-map route's own kappa column is noisy (it comes from a 0.025 m occupancy grid), so it is
recomputed: heading is smoothed with sigma_m (default 0.1 m: keeps the sim design centerline's peak curvature, 0.94 1/m) before differentiating, then kappa again.

Usage:
  moatasem_speed_profile.py <route.csv> --out <profile.csv> --a-lat 4.5 [--v-max 3.5 --accel 3.0
                            --brake 4.0 --sigma-m 0.1] [--plot <png>]
"""
import argparse
import math

import numpy as np
from scipy.ndimage import gaussian_filter1d


def build(route, a_lat, v_max=3.5, accel=3.0, brake=4.0, sigma_m=0.1):
    xy = route[:, 1:3]
    seg = np.linalg.norm(np.roll(xy, -1, axis=0) - xy, axis=1)          # i -> i+1, closed loop
    ds = float(np.mean(seg))
    d = np.roll(xy, -1, axis=0) - np.roll(xy, 1, axis=0)                 # central difference
    psi = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
    n = len(xy)
    # remove the 2*pi*k ramp before periodic smoothing, add it back afterwards
    turn = float(np.round((np.unwrap(np.r_[psi, psi[0]])[-1] - psi[0]) / (2 * np.pi)) * 2 * np.pi)
    s = np.r_[0, np.cumsum(seg[:-1])]
    L = float(np.sum(seg))
    ramp = turn * s / L
    sig = max(sigma_m / ds, 0.5)
    psi_s = gaussian_filter1d(psi - ramp, sig, mode='wrap') + ramp
    dpsi = (np.roll(psi_s, -1) - np.roll(psi_s, 1))
    dpsi[0] = psi_s[1] - (psi_s[-1] - turn)
    dpsi[-1] = (psi_s[0] + turn) - psi_s[-2]
    kappa = dpsi / (np.roll(seg, 1) + seg)
    kappa = gaussian_filter1d(kappa, max(0.5 * sig, 0.5), mode='wrap')
    v = np.minimum(v_max, np.sqrt(a_lat / np.maximum(np.abs(kappa), 1e-4)))
    for _ in range(200):
        old = v.copy()
        for i in range(n):                                   # forward: acceleration limit
            j = (i + 1) % n
            v[j] = min(v[j], math.sqrt(v[i] ** 2 + 2 * accel * seg[i]))
        for i in range(n - 1, -1, -1):                       # backward: braking limit
            j = (i + 1) % n
            v[i] = min(v[i], math.sqrt(v[j] ** 2 + 2 * brake * seg[i]))
        if np.max(np.abs(v - old)) < 1e-9:
            break
    ax = (np.roll(v, -1) ** 2 - v ** 2) / (2 * seg)
    out = np.column_stack((s, xy, (psi_s + np.pi) % (2 * np.pi) - np.pi, kappa, v, ax))
    lap_time = float(np.sum(seg / np.maximum(0.5 * (v + np.roll(v, -1)), 1e-3)))
    return out, dict(lap_time_s=lap_time, v_min=float(v.min()), v_max=float(v.max()),
                     v_mean=L / lap_time, length_m=L, kappa_abs_max=float(np.abs(kappa).max()))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('route'); p.add_argument('--out', required=True)
    p.add_argument('--a-lat', type=float, required=True); p.add_argument('--v-max', type=float, default=3.5)
    p.add_argument('--accel', type=float, default=3.0); p.add_argument('--brake', type=float, default=4.0)
    p.add_argument('--sigma-m', type=float, default=0.1); p.add_argument('--plot')
    a = p.parse_args()
    route = np.loadtxt(a.route, delimiter=';', comments='#')
    out, info = build(route, a.a_lat, a.v_max, a.accel, a.brake, a.sigma_m)
    with open(a.out, 'w') as f:
        f.write(f'# slam_map frame; MOATASEM speed profile (SIM ONLY) of {a.route}: a_lat {a.a_lat} m/s^2, '
                f'v_max {a.v_max}, accel {a.accel}, brake {a.brake}, kappa smoothing sigma {a.sigma_m} m\n')
        f.write('# s_m;x_m;y_m;psi_rad;kappa_1pm;vx_mps;ax_mps2\n')
        np.savetxt(f, out, delimiter=';', fmt='%.6f')
    print(f'{a.out}: ' + ' '.join(f'{k}={v:.3f}' for k, v in info.items()))
    if a.plot:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 3)); ax.plot(out[:, 0], out[:, 5]); ax.set_xlabel('s (m)'); ax.set_ylabel('v (m/s)')
        fig.tight_layout(); fig.savefig(a.plot, dpi=120)


if __name__ == '__main__':
    main()
