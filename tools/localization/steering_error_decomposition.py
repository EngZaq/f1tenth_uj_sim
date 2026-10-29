#!/usr/bin/env python3
"""Diagnose Pure Pursuit steering error from AMCL position vs heading.

ROS 2 Humble. Read-only evaluator; NEVER publishes /drive.
Simulator odometry is used ONLY as evaluation ground truth.
Compares poses at matched ROS timestamps; all steering calculations use the
original path in slam_map and precisely the same lookahead algorithm.
"""
from __future__ import annotations

import csv
import math
import os
import statistics
from collections import deque
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


ROOT = Path.home() / "f1tenth_sim"
RAW_PATH = ROOT / "reference/generated/uj_technical_slam_01_centerline.csv"
REGISTERED_PATH = ROOT / "reference/generated/uj_technical_slam_01_registered.csv"
OUTPUT = ROOT / "results/localization/steering_error_decomposition.csv"
LOOKAHEADS = (0.5, 0.8, 1.0)
WHEELBASE = 0.3302
MAX_STEER = 0.4189
MAX_SYNC_DT = 0.06


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def read_path(path):
    data = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#"):
                fields = line.split(";")
                data.append(tuple(float(v) for v in fields[:3]))
    if len(data) < 3:
        raise ValueError(f"Not enough path points: {path}")
    return data


def path_registration(src, dst):
    """Find map -> sim rigid transform, avoiding numpy dependency."""
    if len(src) != len(dst):
        raise ValueError("Path point counts differ: cannot pair corresponding waypoints")
    mx = statistics.mean(p[1] for p in src)
    my = statistics.mean(p[2] for p in src)
    sx = statistics.mean(p[1] for p in dst)
    sy = statistics.mean(p[2] for p in dst)
    dot = 0.0
    cross = 0.0
    for a, b in zip(src, dst):
        ax, ay = a[1]-mx, a[2]-my
        bx, by = b[1]-sx, b[2]-sy
        dot += ax*bx + ay*by
        cross += ax*by - ay*bx
    theta = math.atan2(cross, dot)
    c, s = math.cos(theta), math.sin(theta)
    tx = sx - (c*mx-s*my)
    ty = sy - (s*mx+c*my)
    residual = math.sqrt(sum(
        (c*a[1]-s*a[2]+tx-b[1])**2 +
        (s*a[1]+c*a[2]+ty-b[2])**2
        for a, b in zip(src, dst)
    ) / len(src))
    if residual > 0.02:
        raise ValueError(f"Unexpected map-registration residual: {residual:.3f} m")
    return theta, tx, ty, residual


def to_map(sx, sy, syaw, reg):
    theta, tx, ty, _ = reg
    c, s = math.cos(theta), math.sin(theta)
    dx, dy = sx-tx, sy-ty
    return c*dx+s*dy, -s*dx+c*dy, wrap(syaw-theta)


def pure_pursuit(path, x, y, angle, lookahead):
    n = len(path)
    nearest = min(
        range(n),
        key=lambda i: (path[i][1]-x)**2 + (path[i][2]-y)**2
    )
    target_s = path[nearest][0] + lookahead
    target = next((j for j in range(nearest, n)
                   if path[j][0] >= target_s), None)
    if target is None:
        rem = target_s - path[-1][0]
        target = next((j for j in range(n) if path[j][0] >= rem), 0)
    dx, dy = path[target][1]-x, path[target][2]-y
    r2 = dx*dx+dy*dy
    steer = 0.0 if r2 < 1e-6 else math.atan(
        WHEELBASE*2.0*(-math.sin(angle)*dx + math.cos(angle)*dy)/r2)
    steer = max(-MAX_STEER, min(MAX_STEER, steer))
    return math.degrees(steer), nearest


def pct95(values):
    v = sorted(values)
    return v[min(len(v)-1, math.ceil(0.95*len(v))-1)]


class Diagnostic(Node):
    def __init__(self):
        super().__init__("steering_error_decomposition")
        self.path = read_path(RAW_PATH)
        registered = read_path(REGISTERED_PATH)
        self.reg = path_registration(self.path, registered)
        self.truth = deque(maxlen=1500)
        self.errors = {L: {"raw": [], "pos": [], "yaw": []} for L in LOOKAHEADS}
        self.pos_err = []
        self.yaw_err = []
        self.sync_dts = []
        self.started = False
        OUTPUT.parent.mkdir(parents=True, exist_ok=True)
        self.output_handle = open(OUTPUT, "w", newline="")
        self.writer = csv.writer(self.output_handle)
        self.writer.writerow([
            "stamp_s", "sync_dt_s", "truth_x_map", "truth_y_map",
            "amcl_x_map", "amcl_y_map", "position_error_m",
            "heading_error_deg", "lookahead_m", "truth_steer_deg",
            "amcl_steer_deg", "amcl_pos_truth_yaw_steer_deg",
            "truth_pos_amcl_yaw_steer_deg", "raw_abs_error_deg",
            "position_only_abs_error_deg", "heading_only_abs_error_deg",
            "truth_nearest_idx", "amcl_nearest_idx"
        ])
        self.create_subscription(Odometry, "/ego_racecar/odom", self.on_truth, 50)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl, 20
        )
        t, tx, ty, residual = self.reg
        print(f"Map->sim: rotation={math.degrees(t):+.4f}deg, "
              f"translation=({tx:+.4f},{ty:+.4f})m, "
              f"residual={residual:.6f}m", flush=True)
        print("Diagnostic only: reads simulator truth and AMCL. "
              "Publishes NO drive commands.", flush=True)
        print("Start the existing oracle racing controller, run one lap, "
              "then Ctrl+C here.", flush=True)

    def on_truth(self, msg):
        stamp = stamp_s(msg.header.stamp)
        pose = msg.pose.pose
        speed = abs(msg.twist.twist.linear.x)
        self.truth.append((
            stamp, pose.position.x, pose.position.y,
            yaw(pose.orientation), speed
        ))

    def on_amcl(self, msg):
        if msg.header.frame_id != "slam_map" or not self.truth:
            return
        t = stamp_s(msg.header.stamp)
        nearest = min(self.truth, key=lambda p: abs(p[0]-t))
        dt = abs(nearest[0] - t)
        if dt > MAX_SYNC_DT or nearest[4] < 0.25:
            return
        _, sx, sy, syaw, _ = nearest
        gx, gy, gh = to_map(sx, sy, syaw, self.reg)
        ap = msg.pose.pose
        ax, ay, ah = ap.position.x, ap.position.y, yaw(ap.orientation)
        pos_error = math.hypot(ax-gx, ay-gy)
        yaw_error = abs(math.degrees(wrap(ah-gh)))
        self.pos_err.append(pos_error)
        self.yaw_err.append(yaw_error)
        self.sync_dts.append(dt)
        for L in LOOKAHEADS:
            truth_steer, ti = pure_pursuit(self.path, gx, gy, gh, L)
            raw_steer, ai = pure_pursuit(self.path, ax, ay, ah, L)
            pos_steer, _ = pure_pursuit(self.path, ax, ay, gh, L)
            yaw_steer, _ = pure_pursuit(self.path, gx, gy, ah, L)
            eraw = abs(raw_steer-truth_steer)
            epos = abs(pos_steer-truth_steer)
            eyaw = abs(yaw_steer-truth_steer)
            self.errors[L]["raw"].append(eraw)
            self.errors[L]["pos"].append(epos)
            self.errors[L]["yaw"].append(eyaw)
            self.writer.writerow([
                f"{t:.6f}", f"{dt:.4f}", f"{gx:.4f}", f"{gy:.4f}",
                f"{ax:.4f}", f"{ay:.4f}", f"{pos_error:.4f}",
                f"{yaw_error:.3f}", f"{L:.2f}", f"{truth_steer:.3f}",
                f"{raw_steer:.3f}", f"{pos_steer:.3f}",
                f"{yaw_steer:.3f}", f"{eraw:.3f}", f"{epos:.3f}",
                f"{eyaw:.3f}", ti, ai
            ])
        if len(self.pos_err) % 50 == 0:
            self.output_handle.flush()
            n = len(self.pos_err)
            print(f"matched={n} pos={statistics.mean(self.pos_err):.3f}m "
                  f"yaw={statistics.mean(self.yaw_err):.2f}deg "
                  f"steering error L=0.5m:"
                  f"{statistics.mean(self.errors[0.5]['raw']):.2f}deg",
                  flush=True)

    def finish(self):
        self.output_handle.flush()
        self.output_handle.close()
        print("\n=== STEERING ERROR DECOMPOSITION ===")
        if not self.pos_err:
            print("No matched moving samples. Verify /amcl_pose and "
                  "/ego_racecar/odom, and timestamp synchronization.")
            return
        print(f"Matched moving samples: {len(self.pos_err)}")
        print(f"Mean time matching error: {statistics.mean(self.sync_dts)*1000:.1f} ms")
        print(f"Mean AMCL position error: {statistics.mean(self.pos_err):.3f} m")
        print(f"Mean AMCL heading error: {statistics.mean(self.yaw_err):.2f} deg")
        print("Absolute steering error against timestamp-matched true pose,"
              " SAME SLAM-map path:")
        for L in LOOKAHEADS:
            print(f"  Lookahead {L:.1f} m:")
            for label, key in (("AMCL position + AMCL yaw", "raw"),
                               ("AMCL position + TRUE yaw", "pos"),
                               ("TRUE position + AMCL yaw", "yaw")):
                v = self.errors[L][key]
                print(f"    {label:29s} "
                      f"mean={statistics.mean(v):5.2f} deg "
                      f"p95={pct95(v):5.2f} deg "
                      f"max={max(v):5.2f} deg")
        print(f"CSV: {OUTPUT}")


def main():
    rclpy.init()
    node = Diagnostic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        # ROS shutdown can race with a callback under certain rclpy builds.
        if rclpy.ok():
            raise
        print(f"ROS shutdown: {exc}")
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
