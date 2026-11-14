# Test 6: speed sweep, closed loop (AMCL + EKF C), F1TENTH simulator

Simulation only, 13–14 Nov 2026. Map `results/missions/autonomous_20261103_114924_818822`, Nav2 AMCL with the
**team laser model (A0, `config/fusion_amcl_moatasem.yaml`)**, EKF C (`config/fusion_ekf_moatasem.yaml`),
`config/fusion_sim_sensors_calibrated.yaml` (seed 42), driver `tools/navigation/profiled_amcl_drive_moatasem.py`
with the Test 4 limits. Truth was used only for grading, and as the stand-in localizer in the Step 3 runs.
Grading is the Test 4 method: rear axle (CoG − 0.17145 m along the heading), with the same fixed SE(2) (CAL_NT).

## Conclusion

- **Max speed validated: 0.8 m/s.** At 0.8 m/s (constant, corners included) the car completed **5 of 5 laps** with
  both 8.65 Hz and 30 Hz scans: true CTE mean 1.4–1.5 cm, max 4.1–4.4 cm, no collision, no guard stop.
  - One earlier 0.8 m/s run at 30 Hz stopped on lap 1 after an AMCL along-track jump (see below).
  - **0.9 m/s already fails**, before the end of lap 1.
- **What breaks first is the driver's collision guard, not localization.**
  - From 0.9 m/s up, every run ends in "no safe local speed: stopping tube obstructed" on a curve entry, with the
    car on the path (estimated CTE ≤ 1.2 cm, true CTE ≤ 4.4 cm).
  - **Driven by the true pose, the car stops at the same corners** (1.0 m/s: route index 120 truth vs 121 AMCL;
    1.5 m/s: stopped while still accelerating, in both). So this is a **controller limit**.
  - The guard assumes 0.35 s reaction and 0.7 m/s² braking. Its stopping tube is 0.76 m long at 0.8 m/s, 1.08 m at
    1.0 and 3.6 m at 2.0, and it follows the current steering arc. On this narrow track (min body clearance
    0.35 m) the tube reaches the wall on curve entries. The guard checks every speed proposal at the *measured*
    speed or above, so it cannot brake early: once the tube at the current speed touches a wall, it holds.
  - **2.0 and 3.0 m/s never started moving.** Twice "stale map->base TF", once "no fresh scan with local odometry
    transform". These are start-up checks of the driver, not localization errors (AMCL error 5–11 cm at standstill
    = the seed offset).
- **Localization is fine wherever the car could drive (≤ 1.0 m/s):**
  - AMCL error vs truth RMSE 5–6 cm, max 10–20 cm.
  - The pose the driver uses is **not late**: it is 1–5 cm *ahead* of truth on average.
  - EKF vx error ≤ 9 mm/s, EKF yaw-rate error 0.3–0.5 °/s in curves.
  - True rear-axle slip speed in curves is only 7–13 mm/s.
  - Speed and lateral slip are simply too low in this range to stress localization.
- **30 Hz scans did not help AMCL.**
  - RMSE about the same (5.5 vs 5.1 cm at 0.8 m/s), but max error higher (19.6–22.8 cm vs 12.3–16.0 cm).
  - Along-track jumps ≥ 0.15 m appeared in 3 of 9 runs at 30 Hz, and in none of 3 at 8.65 Hz. One of them stopped
    a run: a single 0.71 m map→odom update moved the driver's pose 0.21 m *away* from truth (0.027 → 0.206 m along
    the track). That is a real localization error.
  - AMCL CPU is 10 % vs 7 %.

### Which of my 7–10 Nov decisions still hold at speed

Speed could be tested only up to ~1.0 m/s sustained (1.3 m/s peak), so "at speed" means up to that.

| Decision | Holds? | Evidence (Test 6) |
|---|---|---|
| EKF gates off | yes, up to 1.0 m/s | EKF vx bias ≤ 9 mm/s, yaw-rate RMS ≤ 0.5 °/s in curves (1.75 °/s in one 1.0 m/s run at 8.65 Hz); no outliers to gate |
| R = measured | yes | the same: EKF velocity matches truth within sensor noise at every tested speed |
| no rf2o | yes | true lateral slip in curves 7–13 mm/s RMS: nothing for rf2o to correct in this speed range. Not tested where slip matters (> 1.5 m/s) |
| Q values (Q_vx 0.05) | yes | Q_vx 0.5 was **not better** (single 5-lap run: driver pose RMSE 5.4 vs 4.2 cm, AMCL 6.4 vs 5.5 cm, max step 12.9 vs 7.8 cm) |
| AMCL over PF | yes | AMCL localized well at every speed the driver allowed. PF not retested |
| Main scan rate 30 Hz (new) | **no gain** | AMCL max error and jumps worse at 30 Hz; RMSE unchanged; recommend keeping AMCL's input at ~8–10 Hz (or raising `update_min_d` / `resample_interval`) until tested on the car |

**Open:** localization above 1.3 m/s is untested. The driver's guard and start-up checks stop the car before
localization can be judged. Raising the speed needs controller work (guard braking model, earlier speed
reduction before curves, a speed-scaled lookahead above 1.5 m/s, start-up checks). That needs your decision.

## Setup changes made during the test (before the sweep)

### Why today's Test 4 setup froze (27–46 % of ticks vs 0.1 % in Test 4)
- **`/tf` load is the same as in Test 4** (1450 vs 1530 msg/s). The simulator publishes base_link + 4 wheel frames
  on a 4 ms timer.
- **A bare Python TF listener costs 93 % of a core** on this laptop just to take in that stream; the driver
  adds its guard work on top.
- **The machine was busier than during Test 4 (overnight):**
  - VS Code's C/C++ extension (`cpptools`) used 99.5 % of a core.
  - Foxglove was open in Chromium, fed by the simulator's `foxglove_bridge`.
  - The first runs were on **battery / "balanced"**.
  - After that: AC and `performance` profile (`powerprofilesctl set performance`, still set; `powerprofilesctl set balanced` reverts it).
  - The relay at 30 Hz was *not* the cause: a 0.8 m/s lap at 8.65 Hz also froze.

CPU per process during one 0.8 m/s lap, Test 4 setup unchanged, Foxglove closed, AC
(`runs/diag_v0.8_p01_fulltf/cpu_all.txt`, % of one core, machine total 631 % of 800 %):

| process | CPU % |
|---|---|
| driver (`profiled_amcl_drive_moatasem.py`) | 103.8 |
| scan relay (team, also has a full TF listener) | 100.6 |
| VS Code `cpptools` | 99.5 |
| gym_bridge | 95.3 |
| rosbag record | 36.9 |
| VS Code GPU process | 36.9 |
| sim_vesc_imu | 36.3 |
| drive_watchdog | 31.1 |
| AMCL | 16.3 |
| EKF | 12.5 |

Frozen-tick share (pose unchanged between two 50 ms ticks while moving):

| run | conditions | frozen |
|---|---|---|
| Test 4 T_1–T_5 | overnight, A3 | 0.0–0.1 % |
| smoke_v1.0_p01 | battery, Foxglove open, 8.65 Hz | 26.8 % |
| smoke2_v1.0 | AC + performance, Foxglove open, 30 Hz | 45.9 % |
| diag_v0.8_p01_fulltf | AC, Foxglove closed, Test 4 setup | 13.2 % (stopped at 6 m) |
| **val1** | **`/fusion/tf` remap, 8.65 Hz, 0.8 m/s** | **0.0 %, lap completed** |
| **val2 / val2b** | **`/fusion/tf` remap, 30 Hz, 0.8 m/s** | **0.0 % / 0.0 %** (val2: guard stop at 0.82 lap; val2b: lap completed) |

### Fix used for every Test 6 run: estimation TF on its own topic (launch remaps only)
- EKF and AMCL publish and listen on `/fusion/tf` / `/fusion/tf_static`.
  `fusion_base → fusion_laser` is published there by a static publisher, with the same value as the relay
  (`tools/sim_geometry.py`).
- The driver listens there (~70 msg/s, like the real car).
- Driver CPU dropped from 104 % to 40 %, AMCL from 16 to 7 %, EKF from 12 to 3 %.
- The TF audit passed in every run: one owner per edge, only `amcl` and `ekf_filter_node` on `/fusion/tf`.
- The team relay is unchanged; it still uses ~70–100 % of a core for its TF listener, which it only needs once at
  start-up (issue for the team).

### Route speed
The planner's profile (lateral accel 0.9 m/s², accel 0.5) caps corners at 0.89 m/s and peaks at 1.88 m/s for any
cap. So each speed uses a **constant-speed copy** of the same geometry (`results/test6_speed_sweep/routes/`).
The driver's own ramp (0.5 m/s² up) still applies.

## Step 0: speed limits

Watchdog change (team file, approved). The hard-coded 1.0 m/s ceiling is now the ROS parameter `max_speed`
(default 1.0). `--max-speed` (default 0.8) is unchanged, so team behaviour is identical. Team tests pass (3/3);
the default still refuses 1.5. Speed tests: `--max-speed 3.05 --ros-args -p max_speed:=3.05`.

```diff
diff --git a/tools/navigation/drive_watchdog.py b/tools/navigation/drive_watchdog.py
index 7de808d..c581c2e 100644
--- a/tools/navigation/drive_watchdog.py
+++ b/tools/navigation/drive_watchdog.py
@@ -12,8 +12,11 @@ from pathlib import Path
 
 
 class DrivePolicy:
-    def __init__(self, command_timeout=.25, scan_timeout=.45, max_speed=.8):
-        if not math.isfinite(max_speed) or not .05 <= max_speed <= 1.0:
+    def __init__(self, command_timeout=.25, scan_timeout=.45, max_speed=.8, speed_ceiling=1.0):
+        # speed_ceiling: upper bound for max_speed (ROS parameter `max_speed`, default 1.0).
+        if not math.isfinite(speed_ceiling) or speed_ceiling < .05:
+            raise ValueError("Invalid simulation speed ceiling")
+        if not math.isfinite(max_speed) or not .05 <= max_speed <= speed_ceiling:
             raise ValueError("Invalid simulation speed limit")
         self.max_speed = max_speed
         self.command_timeout, self.scan_timeout = command_timeout, scan_timeout
@@ -49,16 +52,21 @@ def main():
     parser = argparse.ArgumentParser(description=__doc__)
     parser.add_argument('--fault-file', type=Path, required=True)
     parser.add_argument("--max-speed",type=float,default=.8)
-    args = parser.parse_args()
+    import sys
     import rclpy
+    from rclpy.utilities import remove_ros_args
+    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
     from rclpy.node import Node
     from rclpy.qos import qos_profile_sensor_data
     from ackermann_msgs.msg import AckermannDriveStamped
     from sensor_msgs.msg import LaserScan
     from std_msgs.msg import Bool
-    rclpy.init(args=[])
+    rclpy.init(args=sys.argv)
     node = Node('simulation_drive_watchdog')
-    policy = DrivePolicy(max_speed=args.max_speed)
+    # Hard ceiling for --max-speed (sim only). Default 1.0 = previous hard-coded bound.
+    # Raise it explicitly for speed tests, e.g. --ros-args -p max_speed:=3.05 (a float).
+    speed_ceiling = float(node.declare_parameter('max_speed', 1.0).value)
+    policy = DrivePolicy(max_speed=args.max_speed, speed_ceiling=speed_ceiling)
     if node.count_publishers('/drive'):
         raise RuntimeError('Existing /drive publisher; watchdog refuses ownership')
     pub = node.create_publisher(AckermannDriveStamped, '/drive', 10)
```

**`full_autonomy.py --speed-cap`** (not changed):
- It is validated ≤ 0.8 and goes to the planner (`--max-speed`, the route profile) and to
  `run_profiled_saved_map.py` (its own ≤ 0.8 check, then the team driver with a hard-coded 0.8 bound).
- The watchdog gets `--max-speed max(0.8, explore_speed)`.
- A pass-through would need all three bounds raised, plus `--ros-args -p max_speed:=<cap>` on the watchdog.
  Even then, the planner's profile caps the real speed at ~0.9 m/s in corners.

Other limits that cap speed:

| where | limit |
|---|---|
| watchdog | `--max-speed` (command above it = latched fault); ceiling `max_speed` (was 1.0) |
| route profile (planner) | v ≤ √(0.9 / abs(κ)) (0.89 m/s at the tightest corner, R 0.88 m), accel 0.5, brake 0.8 m/s²; 3 Nov route: 0.61–0.80 m/s |
| driver | `max_allowed_speed_mps` ≤ 2.0 (my copy: raised to 5.0); ramp +0.5 / −0.8 m/s² (hard-coded); CTE hold 0.09 m; speed reduced from 2 cm CTE; pose step > 0.2 m per tick and AMCL jump > 0.45 m = hold |
| collision guard | stopping tube: reaction ≥ 0.35 s, braking 0.7 m/s², margin 0.07 m; length = v·0.35 + v²/1.4 |
| Gym vehicle (f1tenth) | v_max 20 m/s, a_max 9.51 m/s², steering ±0.419 rad, steering rate ±3.2 rad/s, μ 1.05 (single-track model): the tightest corner's friction limit is ≈ 3.0 m/s; `sim.yaml` has no speed limit |

## Step 1: how the driver sets speed and lookahead
- **Speed** = min(cap, lowest route vx over the next 6 points, AMCL along-track-σ cap (slows from 0.225 m, pause at 0.30 m)).
  - It is then reduced with the estimated CTE: full speed below 2 cm, 0.1 m/s at 6 cm.
  - The guard then picks the fastest clear proposal from {1, 0.85, 0.7, 0.55, 0.4, 0.25} × that, or 0.05.
  - Finally it is ramped at +0.5 / −0.8 m/s².
- **Lookahead** = clamp(0.30 + 0.20·v, 0.25, 0.60) m along the route; pure pursuit with wheelbase 0.3302 m.
  - **It scales with speed only up to 1.5 m/s.** Above that it stays at 0.60 m, so the preview time falls to 0.4 s
    at 1.5 m/s and 0.2 s at 3.0.

## Step 2: results per speed

One run per configuration, 5 laps requested. "driver pose" is the `slam_map → fusion_base` TF the driver read.
"Equivalent delay" = −(mean along-track error ÷ true speed). Negative means the estimate is *ahead*, not late.
"Curves" = true |yaw rate| > 0.5 rad/s; "–" = no such section reached.

### A. 0.8–1.0 m/s

| metric | 0.8 · 8.65 Hz | 0.8 · 30 Hz | 0.8 · 30 Hz (1st) | 0.9 · 30 Hz | 1.0 · 30 Hz | 1.0 · 8.65 Hz | 1.0 · TRUTH |
|---|---|---|---|---|---|---|---|
| end of run | **5 laps done** | **5 laps done** | pose step > 0.2 m | guard stop (tube obstructed) | guard stop (tube obstructed) | guard stop (tube obstructed) | guard stop (tube obstructed) |
| true laps / distance (m) | 5.00 / 176 | 5.00 / 176 | 0.56 / 20 | 0.86 / 30 | 0.32 / 11 | 0.31 / 11 | 0.35 / 12 |
| collisions | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| mean true speed (m/s) | 0.79 | 0.79 | 0.76 | 0.85 | 0.88 | 0.87 | 0.89 |
| true CTE mean / max (cm) | 1.4 / 4.1 | 1.5 / 4.4 | 1.8 / 3.5 | 1.6 / 3.9 | 2.2 / 3.5 | 2.3 / 4.3 | 0.4 / 1.4 |
| driver heading err vs truth mean / max (°) | 0.59 / 2.42 | 0.57 / 3.23 | 0.85 / 3.76 | 0.44 / 2.41 | 0.36 / 1.07 | 0.54 / 1.75 | 0.44 / 1.88 |
| driver pose err vs truth RMSE / max (cm) | 4.2 / 14.3 | 4.2 / 18.1 | 5.2 / 13.6 | 4.5 / 9.7 | 5.6 / 9.9 | 7.1 / 12.2 | 2.8 / 4.8 |
| AMCL pose err RMSE / max (cm) | 5.1 / 16.0 | 5.5 / 19.6 | 5.0 / 21.0 | 5.9 / 11.1 | 6.0 / 10.3 | 9.2 / 14.6 | – |
| AMCL σ / real error, position · yaw | 2.6 · 5.2 | 1.9 · 3.8 | 2.2 · 3.1 | 1.9 · 4.5 | 1.8 · 6.7 | 1.5 · 3.5 | – |
| driver pose along-track err mean (cm, + = ahead) | 1.1 | 1.5 | -1.8 | 2.4 | 2.4 | 4.8 | -2.8 |
| → equivalent delay (ms, + = late) | -13 | -19 | 22 | -29 | -31 | -56 | 29 |
| EKF vx − true vx, curves / straights (mm/s) | 2 / 2 | 0 / 2 | 2 / 2 | 5 / 3 | – / 0 | 9 / 4 | -0 / 1 |
| EKF yaw-rate error RMS in curves (°/s) | 0.33 | 0.51 | 0.26 | 0.49 | – | 1.75 | 0.33 |
| true rear-axle lateral (slip) speed RMS in curves (mm/s) | 7.1 | 7.1 | 7.3 | 9.1 | – | 10.0 | 12.7 |
| steering-rate std / max (°/s) | 10.3 / 98 | 10.9 / 68 | 11.3 / 52 | 11.5 / 71 | 8.1 / 26 | 7.7 / 26 | 7.4 / 46 |
| largest driver pose step (cm) | 8.0 | 7.8 | 11.4 | 7.5 | 8.0 | 8.4 | 7.6 |
| frozen pose ticks (%) | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 |
| AMCL pose rate / scan rate (Hz) | 8.4 / 8.4 | 12.0 / 30.0 | 11.5 / 30.0 | 13.2 / 30.0 | 13.3 / 30.0 | 7.9 / 8.3 | – / 30.0 |
| CPU % amcl / ekf / driver / relay | 7 / 3 / 39 / 68 | 10 / 4 / 42 / 70 | 10 / 4 / 44 / 75 | 9 / 3 / 41 / 70 | 10 / 4 / 47 / 89 | 7 / 3 / 44 / 79 | – / 4 / 45 / –  |

### B. 1.5–3.0 m/s

| metric | 1.5 · 30 Hz | 1.5 · TRUTH | 1.5 · TRUTH, scaled limits | 2.0 · 30 Hz | 2.0 · 30 Hz (rep) | 3.0 · 30 Hz |
|---|---|---|---|---|---|---|
| end of run | guard stop (tube obstructed) | pose step > 0.2 m | guard stop (tube obstructed) | no fresh scan+TF (start-up) | stale map->base TF (start-up) | stale map->base TF (start-up) |
| true laps / distance (m) | 0.06 / 2 | 0.07 / 3 | 0.08 / 3 | 0.00 / 0 | 0.00 / 0 | 0.00 / 0 |
| collisions | 0 | 0 | 0 | 0 | 0 | 0 |
| mean true speed (m/s) | 0.64 | 0.69 | 0.71 | 0.05 | 0.04 | 0.03 |
| true CTE mean / max (cm) | 0.6 / 1.4 | 0.6 / 1.4 | 0.7 / 1.4 | 1.2 / 1.4 | 1.2 / 1.4 | 1.3 / 1.4 |
| driver heading err vs truth mean / max (°) | 1.08 / 5.18 | 0.42 / 3.51 | 0.32 / 1.03 | 0.28 / 0.29 | 0.42 / 0.47 | 0.46 / 0.48 |
| driver pose err vs truth RMSE / max (cm) | 5.7 / 19.4 | 3.8 / 18.6 | 2.6 / 5.7 | 10.2 / 10.8 | 4.7 / 5.4 | 10.9 / 11.2 |
| AMCL pose err RMSE / max (cm) | 4.8 / 9.4 | – | – | 11.0 / 11.0 | 5.2 / 5.2 | 11.2 / 11.2 |
| AMCL σ / real error, position · yaw | 2.3 · 2.6 | – | – | 1.0 · 5.0 | 2.2 · 4.6 | 0.7 · 3.0 |
| driver pose along-track err mean (cm, + = ahead) | 0.7 | -3.3 | -2.7 | – | – | – |
| → equivalent delay (ms, + = late) | -20 | 35 | 30 | – | – | – |
| EKF vx − true vx, curves / straights (mm/s) | – / -3 | – / -1 | – / -1 | – / – | – / – | – / – |
| EKF yaw-rate error RMS in curves (°/s) | – | – | – | – | – | – |
| true rear-axle lateral (slip) speed RMS in curves (mm/s) | – | – | – | – | – | – |
| steering-rate std / max (°/s) | 11.1 / 46 | 5.8 / 29 | 6.5 / 30 | 11.2 / 36 | 8.5 / 20 | 9.1 / 26 |
| largest driver pose step (cm) | 19.4 | 10.4 | 9.6 | 0.1 | 0.1 | 0.0 |
| frozen pose ticks (%) | 4.7 | 2.1 | 0.0 | – | – | – |
| AMCL pose rate / scan rate (Hz) | 10.0 / 30.0 | – / 30.0 | – / 30.1 | 2.0 / 30.5 | 1.8 / 29.3 | 2.2 / 28.7 |
| CPU % amcl / ekf / driver / relay | 9 / 4 / 63 / 99 | – / 4 / 59 / –  | – / 4 / 57 / –  | 5 / 4 / 50 / 97 | 5 / 4 / 49 / 90 | 5 / 3 / 50 / 89 |

### C. one change at a time, 0.8 m/s, 30 Hz, 5 laps

| metric | baseline (A0, EKF C) | EKF Q_vx 0.05 → 0.5 | AMCL update_min_d 0.05 → 0.025 |
|---|---|---|---|
| end of run | **5 laps done** | **5 laps done** | **5 laps done** |
| true laps / distance (m) | 5.00 / 176 | 5.01 / 176 | 5.00 / 176 |
| collisions | 0 | 0 | 0 |
| mean true speed (m/s) | 0.79 | 0.79 | 0.79 |
| true CTE mean / max (cm) | 1.5 / 4.4 | 1.5 / 5.3 | 1.5 / 4.9 |
| driver heading err vs truth mean / max (°) | 0.57 / 3.23 | 0.61 / 3.14 | 0.70 / 3.58 |
| driver pose err vs truth RMSE / max (cm) | 4.2 / 18.1 | 5.4 / 21.0 | 6.5 / 22.0 |
| AMCL pose err RMSE / max (cm) | 5.5 / 19.6 | 6.4 / 22.1 | 7.5 / 22.8 |
| AMCL σ / real error, position · yaw | 1.9 · 3.8 | 1.7 · 3.3 | 1.1 · 2.1 |
| driver pose along-track err mean (cm, + = ahead) | 1.5 | 2.6 | 3.7 |
| → equivalent delay (ms, + = late) | -19 | -33 | -47 |
| EKF vx − true vx, curves / straights (mm/s) | 0 / 2 | 3 / 3 | 2 / 3 |
| EKF yaw-rate error RMS in curves (°/s) | 0.51 | 0.34 | 0.33 |
| true rear-axle lateral (slip) speed RMS in curves (mm/s) | 7.1 | 7.3 | 7.1 |
| steering-rate std / max (°/s) | 10.9 / 68 | 11.2 / 84 | 10.9 / 109 |
| largest driver pose step (cm) | 7.8 | 12.9 | 8.3 |
| frozen pose ticks (%) | 0.0 | 0.0 | 0.0 |
| AMCL pose rate / scan rate (Hz) | 12.0 / 30.0 | 12.0 / 30.0 | 20.2 / 30.0 |
| CPU % amcl / ekf / driver / relay | 10 / 4 / 42 / 70 | 11 / 3 / 42 / 69 | 16 / 4 / 43 / 69 |

All runs, all fields: `table_all_runs.md`.

### Stop classification
| run | stop | class |
|---|---|---|
| 0.9, 1.0 (30 Hz and 8.65 Hz), 1.5 (30 Hz), val2 | guard: stopping tube obstructed, car on path | **controller limit** (truth stops at the same places) |
| truth 1.0, truth 1.5 with scaled limits | guard: same | controller limit |
| 0.8 · 30 Hz (1st run) | pose step 0.219 m: AMCL jump *away* from truth (0.03 → 0.21 m along-track) | **real localization error** (rerun not applicable: it is not a false trip) |
| truth 1.5 | pose step 0.239 m: pose froze, fell 0.19 m behind, then caught up | **false trip** (pipeline lag) → rerun with pose-step / AMCL-jump limits scaled by speed × dt: then a guard stop at the same place |
| 2.0 (×2), 3.0 | stale map→base TF / no fresh scan + TF, before moving | **false trip** (driver start-up). Not rerun with scaled limits: those limits were not involved |

## Step 3: separating the causes
- **At 1.0 m/s (first failing speed), the truth-driven run stopped at the same corner as the AMCL run** →
  controller / vehicle limit.
- Same at 1.5 m/s.
- The localization changes were still run, one at a time, at the one speed where localization stopped a run
  (0.8 m/s, 30 Hz): table C. Neither is better than the baseline; both completed 5 laps, like the baseline repeat.
  - **EKF Q_vx 0.05 → 0.5:** driver pose RMSE 5.4 vs 4.2 cm, largest step 12.9 vs 7.8 cm.
  - **AMCL `update_min_d` 0.05 → 0.025:** AMCL updates 20 vs 12 Hz, CPU 16 vs 10 %, RMSE 7.5 vs 5.5 cm.
    Its σ/real ratio is closer to 1 (1.1 vs 1.9), but its error is larger.
  - Single runs: these differences are within what one run can show, so the conclusion is "no gain", not "worse".

## Scan rate in Tests 1–4 (before Test 6)

The relay's 0.1 s throttle has been in `fusion_scan_relay.py` since the team's first commit (6e67662, 29 Oct).
My 7 Nov commit d0b63d8 only made it the parameter `min_period_s` (default 0.1). No archived bag recorded a scan
topic, so the rate comes from the consumers' message counts (`ros2 bag` metadata) and from today's measurement of
the same code (0.1 → 8.3–8.6 Hz, 0.0 → 30.0 Hz).

| test | scan topic → consumer | real scan rate |
|---|---|---|
| early EKF / rf2o R×gate (C1–C4, E2, S1meas) | `/fusion/scan` → rf2o | **8.4–8.5 Hz** (rf2o output count) |
| rf2o lag test B_base / B_smooth | `/fusion/scan` → rf2o | **8.3–8.6 Hz** |
| rf2o lag test B_base+fast / B_smooth+fast ("30 Hz") | relay 0.0 → rf2o (`freq` 30) | input **30 Hz**; rf2o produced only **15.7–19.5 Hz** at 86–97 % of a core |
| AMCL shadow A/B/C/P, closed loop A/C/P | `/fusion/scan` → AMCL | **8.4 Hz** (AMCL pose 7.0–7.5 Hz) |
| Test 2 / 3 / 3b mapping (T, C, CR, L, LT) | `/fusion/scan` → slam_toolbox (+ rf2o in CR) | **8.4 Hz** (`full_autonomy.py` starts the relay without parameters; 8.4 Hz measured on 13 Nov) |
| Test 4 AMCL shadow + closed loop | `/fusion/scan` → AMCL | **8.4 Hz** (AMCL pose 7.2–7.5 Hz) |
| Test 4 particle filter | `/fusion/scan` → 1:1 `/pf/scan` → PF | **8.4 Hz** (PF pose 25–27 Hz is driven by odometry, not scans) |

**rf2o "30 Hz no gain":**
- The relay did **not** throttle it: the "fast" runs had relay 0.0 and 30 Hz input.
- rf2o was **CPU-bound at ~19 Hz**. The result therefore means "no gain at ~19 Hz", not at 30 Hz.
- The **Test 2 CR** mapping result ("rf2o no reliable gain") **was** at the throttled 8.4 Hz input.

## Deviations and incidents
- **Watchdog margin:** `--max-speed` = speed + 0.05 (Test 4 used 0.85 for 0.8), because the ramp can overshoot by 1e-16.
- **Seed:** "seed 42" is the sensor-noise seed (`fusion_sim_sensors_calibrated.yaml`). Nav2 AMCL has no seed parameter.
- **Batch runner broke once (my mistake):** I edited the runner while a batch was using it. The 3.0 m/s run broke
  after the simulator started, and the simulator was left running. I stopped it with one SIGINT to its process
  group; the check was clean afterwards. Later batches ran from frozen copies (`run_t6_frozen_b2.sh`, `_b3.sh`).
- **Clean check matched my own shell once:** its command line contained "amcl". All runs of that batch aborted
  before starting anything (correct behaviour), and the batch was rerun after making the check ignore its own
  parent shells.
- **Shutdown:** every run was stopped with one SIGINT per process group, followed by a wait. No `kill -9` was
  needed. The clean check was empty before and after every run.
- **Driver file changed mid-sweep:** while the first sweep was running, I added the opt-in
  `scale_jump_limits_with_speed` parameter (default off). Later runs loaded that file version with the option off,
  i.e. the same behaviour.

## Files
- **Changed (uncommitted):**
  - `tools/navigation/drive_watchdog.py` (team, approved)
  - `tools/navigation/profiled_amcl_drive_moatasem.py`: bound 2.0 → 5.0, opt-in `scale_jump_limits_with_speed`
  - `tools/localization/amcl_shadow.launch.py`: `scan_min_period_s`, `tf_topic`, `tf_static_topic`; defaults = old behaviour
- **New:**
  - `tools/localization/moatasem_truth_pose_relay.py` (Step 3 stand-in localizer, sim only)
  - this folder (summary, 3 plots, `table_all_runs.md`)
- **Raw data (untracked, 842 MB):** `results/test6_speed_sweep/`: `runs/<run>/` (bag, driver CSV, logs, cpu,
  eval.json/npz), `routes/`, `configs/` (the two one-change configs), `scripts/` (runner, grader `eval_t6.py`,
  `analyze_t6.py`, `check_clean.sh`, `tf_probe.py`, …).

Plots: `errors_vs_speed.png`, `stops_and_guard_tube.png`, `trajectories_1.0mps.png`.
