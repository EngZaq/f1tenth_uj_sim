# Closed-loop test: AMCL + EKF (C) vs AMCL + raw VESC (A_play, A)

Sim, new_track map, route `new_track_body_optimized.csv` (own speed profile, capped 0.8 m/s), 2 laps per run,
simulator restarted every run, runs interleaved, EKF debug logging OFF. Truth (Gym) used ONLY here, at the rear axle,
route mapped slam_map -> sim with the fixed SE(2) from calibration run CAL_NT (residual 0.031 m).

Controller: `tools/navigation/profiled_amcl_drive_moatasem.py` (copy of the team's profiled_amcl_drive.py; adds
`odom_topic`, `uncertainty_slowdown_start_m`, `max_allowed_speed_mps`) -> `/mission/drive` -> team `drive_watchdog.py`
(--max-speed 0.85) -> `/drive`. AMCL = amcl_shadow.launch.py, team parameters, seed (0,0,0) on /fusion/initialpose.

## Uncertainty threshold (one setting, identical for all configurations)
Measured in shadow laps (SH_C, SH_P, ~0.75 m/s): AMCL across-track sigma median 0.026 / max 0.053 m; along-track
sigma median 0.12 / max 0.188 m; yaw sigma max 5.3 deg. Set with margin above the max:
`max_across_track_std_m 0.08` (team value), `max_along_track_std_m 0.30` (pause), `max_major_position_std_m 0.40`
(fault), `uncertainty_slowdown_start_m 0.225`, `max_heading_std_deg 10`. No run was paused or stopped by it.

## Results (mean ± run-to-run std)
| metric | C · AMCL + EKF (VESC+IMU) (5 runs) | A_play · AMCL + raw VESC, play 1°+1° (5 runs) | A · AMCL + raw VESC, perfect steering (3 runs) |
|---|---|---|---|
| true laps completed | 2.00 ± 0.00 | 1.78 ± 0.50 | 2.00 ± 0.00 |
| collisions | 0 ± 0 | 0 ± 0 | 0 ± 0 |
| lap 1 time (s) | 49.17 ± 0.06 | 50.59 ± 0.42 | 49.24 ± 0.08 |
| lap 2 time (s) | 48.55 ± 0.91 | 50.56 ± 0.42 | 48.17 ± 0.02 |
| TRUE cross-track error mean (m) | 0.015 ± 0.000 | 0.029 ± 0.000 | 0.016 ± 0.000 |
| TRUE CTE p95 (m) | 0.035 ± 0.001 | 0.057 ± 0.001 | 0.035 ± 0.002 |
| TRUE CTE max (m) | 0.047 ± 0.005 | 0.076 ± 0.003 | 0.048 ± 0.004 |
| controller pose error RMSE (m) | 0.052 ± 0.007 | 0.052 ± 0.009 | 0.063 ± 0.004 |
| controller pose error max (m) | 0.177 ± 0.067 | 0.145 ± 0.018 | 0.173 ± 0.020 |
| controller heading error mean (°) | 0.79 ± 0.19 | 1.83 ± 0.21 | 0.87 ± 0.13 |
| controller heading error max (°) | 3.98 ± 0.93 | 6.09 ± 0.50 | 3.65 ± 0.26 |
| steering angle std (°) | 6.17 ± 0.03 | 6.33 ± 0.04 | 6.20 ± 0.03 |
| steering rate std (°/s) | 10.1 ± 0.6 | 14.0 ± 1.0 | 10.0 ± 0.5 |
| steering rate max (°/s) | 59.0 ± 34.4 | 105.5 ± 41.4 | 53.1 ± 7.3 |
| largest AMCL jump (m) | 0.072 ± 0.083 | 0.104 ± 0.084 | 0.042 ± 0.037 |
| largest AMCL jump (°) | 1.51 ± 1.54 | 2.24 ± 1.61 | 1.20 ± 0.76 |
| CPU AMCL (% core) | 15.4 ± 0.5 | 15.0 ± 0.4 | 15.4 ± 0.1 |
| CPU odometry node (% core) | 10.9 ± 0.6 | 7.6 ± 0.1 | 7.7 ± 0.2 |

Stops / ends per run:
  C_1: end=laps  controller hold: completed 2 estimated laps  watchdog: -
  C_2: end=laps  controller hold: completed 2 estimated laps  watchdog: -
  C_3: end=laps  controller hold: completed 2 estimated laps  watchdog: -
  C_4: end=laps  controller hold: completed 2 estimated laps  watchdog: -
  C_5: end=laps  controller hold: completed 2 estimated laps  watchdog: -
  P_1: end=laps  controller hold: completed 2 estimated laps  watchdog: -
  P_2: end=laps  controller hold: completed 2 estimated laps  watchdog: -
  P_3: end=laps  controller hold: completed 2 estimated laps  watchdog: -
  P_4: end=laps  controller hold: completed 2 estimated laps  watchdog: -
  P_5: end=hold  controller hold: no safe local speed: measured/proposed stopping tube obstructed  watchdog: -
  A_1: end=laps  controller hold: completed 2 estimated laps  watchdog: -
  A_2: end=laps  controller hold: completed 2 estimated laps  watchdog: -
  A_3: end=laps  controller hold: completed 2 estimated laps  watchdog: -
  VOID A_2 (rerun): controller hold: stale map->base TF — controller TF listener starved while my offline bag evaluation ran concurrently; bag shows TF published without gaps
  VOID C_3 (rerun): controller hold: estimated pose discontinuity — controller TF listener starved while my offline bag evaluation ran concurrently; bag shows TF published without gaps

### C · AMCL + EKF (VESC+IMU) minus A_play · AMCL + raw VESC, play 1°+1° (Welch t; "beyond spread" = p < 0.05)
| metric | difference | t | p | beyond spread? |
|---|---|---|---|---|
| true laps completed | +0.22 | +1.00 | 0.373 | no |
| collisions | +0.00 | — | — | identical |
| lap 1 time (s) | -1.42 | -6.76 | 0.006 | **yes** |
| lap 2 time (s) | -2.01 | -4.37 | 0.005 | **yes** |
| TRUE cross-track error mean (m) | -0.014 | -58.25 | 0.000 | **yes** |
| TRUE CTE p95 (m) | -0.022 | -29.46 | 0.000 | **yes** |
| TRUE CTE max (m) | -0.029 | -12.20 | 0.000 | **yes** |
| controller pose error RMSE (m) | -0.000 | -0.01 | 0.989 | no |
| controller pose error max (m) | +0.033 | +1.05 | 0.347 | no |
| controller heading error mean (°) | -1.04 | -8.39 | 0.000 | **yes** |
| controller heading error max (°) | -2.11 | -4.47 | 0.004 | **yes** |
| steering angle std (°) | -0.15 | -6.87 | 0.000 | **yes** |
| steering rate std (°/s) | -3.91 | -7.29 | 0.000 | **yes** |
| steering rate max (°/s) | -46.48 | -1.93 | 0.091 | no |
| largest AMCL jump (m) | -0.032 | -0.60 | 0.562 | no |
| largest AMCL jump (°) | -0.73 | -0.73 | 0.484 | no |
| CPU AMCL (% core) | +0.42 | +1.35 | 0.214 | no |
| CPU odometry node (% core) | +3.22 | +11.44 | 0.000 | **yes** |

### C · AMCL + EKF (VESC+IMU) minus A · AMCL + raw VESC, perfect steering (Welch t; "beyond spread" = p < 0.05)
| metric | difference | t | p | beyond spread? |
|---|---|---|---|---|
| true laps completed | +0.00 | +0.56 | 0.612 | no |
| collisions | +0.00 | — | — | identical |
| lap 1 time (s) | -0.07 | -1.40 | 0.245 | no |
| lap 2 time (s) | +0.38 | +0.93 | 0.405 | no |
| TRUE cross-track error mean (m) | -0.001 | -1.72 | 0.162 | no |
| TRUE CTE p95 (m) | -0.000 | -0.28 | 0.798 | no |
| TRUE CTE max (m) | -0.001 | -0.33 | 0.755 | no |
| controller pose error RMSE (m) | -0.011 | -2.74 | 0.034 | **yes** |
| controller pose error max (m) | +0.004 | +0.12 | 0.909 | no |
| controller heading error mean (°) | -0.08 | -0.74 | 0.491 | no |
| controller heading error max (°) | +0.33 | +0.74 | 0.492 | no |
| steering angle std (°) | -0.03 | -1.21 | 0.287 | no |
| steering rate std (°/s) | +0.06 | +0.16 | 0.876 | no |
| steering rate max (°/s) | +5.89 | +0.37 | 0.728 | no |
| largest AMCL jump (m) | +0.030 | +0.71 | 0.505 | no |
| largest AMCL jump (°) | +0.31 | +0.37 | 0.721 | no |
| CPU AMCL (% core) | -0.03 | -0.14 | 0.896 | no |
| CPU odometry node (% core) | +3.16 | +10.54 | 0.000 | **yes** |


## Collision guard (team swept-tube check) — minimum predicted body gap and ticks slowed by the guard
| config | min predicted gap per run (m) | ticks speed-limited by guard (all runs) |
|---|---|---|
| C | 0.154, 0.193, 0.189, 0.096, 0.178 | 1 |
| A_play | 0.093, 0.095, 0.154, 0.086, **0.078 (stopped)** | 13 |
| A | 0.201, 0.180, 0.180 | 0 |

## Stops
- P_5 (A_play): SAFETY HOLD "no safe local speed: stopping tube obstructed" at 45 s (route index 301-314, 0.89 lap).
  No collision; true CTE at that time <= 0.075 m. Predicted gap fell to 0.078 m vs 0.07 m margin — the same spot where
  P_4 dropped to 0.086 m. Counted as a result (not rerun).
- VOID A_2, VOID C_3 (first attempts, rerun): "stale map->base TF" / "estimated pose discontinuity". The controller's
  Python TF listener froze for 0.25-0.45 s while I ran offline bag evaluation on the same PC; the recorded /tf had no
  gap > 0.07 s. Test-setup fault, rerun with nothing else running. Note for the car: the team controller holds when
  its TF listener is starved — keep heavy processes off the car computer while driving.
- STEP 4 (~1.2 m/s) NOT run: not all configurations completed cleanly (P_5), and the team watchdog caps at 1.0 m/s.

Plots: true_path_vs_route.png, errors_vs_time.png, steering_vs_time.png
