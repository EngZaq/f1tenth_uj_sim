#!/usr/bin/env python3
import csv
import numpy as np
from pathlib import Path

root = Path.home() / "f1tenth_sim"
generated = root / "reference/generated"

def load_path(name):
    rows = []
    with open(generated / name) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            try:
                values = [float(v) for v in line.split(";")]
                rows.append(values[:3])
            except ValueError:
                continue

    if not rows:
        raise ValueError(f"No numeric waypoints in {name}")

    return np.asarray(rows)

def rigid_fit(src, dst):
    a = src - src.mean(axis=0)
    b = dst - dst.mean(axis=0)
    U, _, Vt = np.linalg.svd(a.T @ b)
    D = np.diag([1.0, np.linalg.det(U @ Vt)])
    R = U @ D @ Vt
    offset = dst.mean(axis=0) - src.mean(axis=0) @ R
    return R, offset

def rmse(a, b):
    return np.sqrt(np.mean(np.sum((a-b)**2, axis=1)))

def nearest_distance(positions, path):
    d2 = np.sum(
        (positions[:, None, :] - path[None, :, :])**2,
        axis=2
    )
    return np.sqrt(np.min(d2, axis=1)), np.argmin(d2, axis=1)

raw = load_path("uj_technical_slam_01_centerline.csv")
reg = load_path("uj_technical_slam_01_registered.csv")

with open(root / "results/localization/fusion_amcl_paired.csv") as f:
    data = list(csv.DictReader(f))

truth = np.array([
    [float(r["true_x"]), float(r["true_y"])] for r in data
])

amcl = np.array([
    [float(r["amcl_x"]), float(r["amcl_y"])] for r in data
])

print("\n===== FRAME ALIGNMENT DIAGNOSTIC =====")
print(f"Original path points: {len(raw)}")
print(f"Registered path points: {len(reg)}")
print(f"Recorded poses: {len(data)}")

R_motion, t_motion = rigid_fit(amcl, truth)
motion_angle = np.degrees(np.arctan2(R_motion[0,1], R_motion[0,0]))

print("\n1. AMCL trajectory -> simulator trajectory")
print(f"Rotation: {motion_angle:+.3f} deg")
print(f"Translation: {np.round(t_motion, 4)} m")
print(f"Aligned RMSE: {rmse(amcl @ R_motion + t_motion, truth):.3f} m")

raw_xy = raw[:, 1:3]
reg_xy = reg[:, 1:3]

if len(raw) == len(reg):
    R_path, t_path = rigid_fit(raw_xy, reg_xy)
    path_angle = np.degrees(np.arctan2(R_path[0,1], R_path[0,0]))

    print("\n2. SLAM centerline -> registered centerline")
    print(f"Rotation: {path_angle:+.3f} deg")
    print(f"Translation: {np.round(t_path, 4)} m")
    print(f"Path registration residual: {rmse(raw_xy @ R_path + t_path, reg_xy):.3f} m")

    print("\n3. Difference between independently fitted transforms")
    angle_difference = (motion_angle - path_angle + 180) % 360 - 180
    print(f"Rotation disagreement: {angle_difference:+.3f} deg")
    print(f"Translation disagreement: {np.linalg.norm(t_motion - t_path):.3f} m")

    mapped_amcl = amcl @ R_path + t_path
    print(f"AMCL RMSE using path-derived transform: {rmse(mapped_amcl, truth):.3f} m")

    truth_cte, true_idx = nearest_distance(truth, reg_xy)
    amcl_cte, amcl_idx = nearest_distance(amcl, raw_xy)

    print("\n4. Path tracking in each native coordinate frame")
    print(f"Truth -> registered path, mean nearest distance: {truth_cte.mean():.3f} m")
    print(f"AMCL -> original SLAM path, mean nearest distance: {amcl_cte.mean():.3f} m")
    print(f"AMCL -> original SLAM path, 95th percentile: {np.percentile(amcl_cte,95):.3f} m")
    print(f"AMCL -> original SLAM path, max distance: {amcl_cte.max():.3f} m")

    s = raw[:, 0]
    length = s[-1] + np.linalg.norm(raw_xy[-1]-raw_xy[0])
    if np.all(np.diff(s) >= 0) and length > 0:
        ds = s[amcl_idx] - s[true_idx]
        ds = (ds + length/2) % length - length/2
        print(f"Mean absolute along-track offset: {np.mean(np.abs(ds)):.3f} m")
        print(f"95th percentile along-track offset: {np.percentile(np.abs(ds),95):.3f} m")
else:
    print("\nPaths have different point counts; cannot assume waypoint correspondence.")
    print("Skipping path registration and progress comparison.")

print("\nNOTE: These calculations use simulator truth only for offline evaluation.")
