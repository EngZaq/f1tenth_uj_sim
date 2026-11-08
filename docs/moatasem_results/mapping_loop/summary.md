# Test 3 — mapping with loop closing ON, my EKF only (simulation)

Same setup as Test 2 config C (config/fusion_ekf_moatasem.yaml + config/fusion_sim_sensors_mapping_mine.yaml, no rf2o,
same seed 42 and calibrated noise, explore speed 0.18, mapping-only), plus
`--slam-config config/slam_sim_loop_closing.yaml` (copy of slam_sim.yaml with only do_loop_closing: true;
debug_logging stays false). Live `do_loop_closing` = **True** in every L run (False in every C run). 3 runs, compared
with C's 3 Test 2 runs (loop closing OFF).

**Loop closures** are counted indirectly (slam_toolbox prints nothing about them without debug_logging): abrupt
slam_map->fusion_odom steps > 0.15 m or > 1° (larger than anything in the 9 loop-off runs of Test 2), merged within
2 s. **Correction-step audit:** run with `--max-gap-s 2.0`. slam_toolbox stops publishing the TF while it optimises,
so with the default 0.10 s gap the audit (and my first count) MISSED the big jump (it reported 0.07 m / 2.2°).
The 9 loop-off runs give identical numbers with either gap.

## Results (mean ± run-to-run std)
| metric | C · my EKF (VESC+IMU) + my R (3 runs) | L · my EKF, loop closing ON (3 runs) |
|---|---|---|
| mission success (COMPLETED_MAPPING_AND_PLANNING) | 1.00 ± 0.00 | 1.00 ± 0.00 |
| lap length, estimated (m) | 34.79 ± 0.02 | 34.76 ± 0.08 |
| distance driven to the return (m) | 34.79 ± 0.02 | 34.76 ± 0.08 |
| exploration duration (s) | 231.0 ± 0.2 | 230.4 ± 0.6 |
| return error lap_pose.json, EKF (m) | 0.848 ± 0.002 | 0.848 ± 0.001 |
| return yaw error lap_pose.json (°) | 13.52 ± 1.01 | 13.77 ± 0.82 |
| SLAM finish distance from start (m) | 0.676 ± 0.117 | 0.365 ± 0.056 |
| wall error RMS (m) | 0.1001 ± 0.0088 | 0.4771 ± 0.0189 |
| wall error median (m) | 0.0431 ± 0.0007 | 0.2285 ± 0.0050 |
| wall error p95 (m) | 0.2306 ± 0.0225 | 1.0492 ± 0.0497 |
| wall error max (m) | 0.3213 ± 0.0246 | 1.5482 ± 0.0808 |
| wall cells within 0.05 m (%) | 56.1 ± 0.3 | 14.1 ± 0.7 |
| wall cells within 0.10 m (%) | 75.2 ± 1.7 | 26.4 ± 1.9 |
| start/finish doubling offset (m) | 0.0696 ± 0.0019 | 1.4161 ± 0.1140 |
| doubling visible (fraction of runs) | 1.00 ± 0.00 | 1.00 ± 0.00 |
| loop-closure correction events (indirect, >0.15 m or >1° step) | 0.0 ± 0.0 | 3.0 ± 1.0 |
| largest SLAM correction step (m) | 0.0925 ± 0.0219 | 1.6167 ± 0.0932 |
| largest SLAM correction step (°) | 0.467 ± 0.115 | 24.196 ± 0.226 |
| audit violations (0.25 m / 5°) | 0.0 ± 0.0 | 1.3 ± 0.6 |
| CPU EKF (% core) | 13.6 ± 0.2 | 13.1 ± 0.4 |
| CPU rf2o (% core) | — | — |
| CPU slam_toolbox (% core) | 33.3 ± 0.9 | 35.2 ± 0.4 |

Per run:
| run | stage | live do_loop_closing | lap (m) | duration (s) | return EKF (m / °) | wall RMS (m) | p95 (m) | doubling (m) | largest SLAM step (m / °) | loop events | CPU ekf / rf2o / slam (%) | new /var/crash |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| C_1 | COMPLETED_MAPPING_AND_PLANNING | Boolean value is: False | 34.77 | 231 | 0.849 / 13.7 | 0.0999 | 0.2357 | 0.069 | 0.0917 / 0.400 | 0 | 13.4 / — / 32.3 | none |
| C_2 | COMPLETED_MAPPING_AND_PLANNING | Boolean value is: False | 34.80 | 231 | 0.846 / 14.4 | 0.1090 | 0.2502 | 0.072 | 0.1148 / 0.600 | 0 | 13.5 / — / 34.0 | none |
| C_3 | COMPLETED_MAPPING_AND_PLANNING | Boolean value is: False | 34.81 | 231 | 0.848 / 12.4 | 0.0914 | 0.2060 | 0.068 | 0.0710 / 0.400 | 0 | 13.8 / — / 33.7 | none |
| L_1 | COMPLETED_MAPPING_AND_PLANNING | Boolean value is: True | 34.83 | 231 | 0.847 / 13.3 | 0.4897 | 1.0873 | 1.305 | 1.5669 / 24.315 | 3 | 12.7 / — / 35.1 | none |
| L_2 | COMPLETED_MAPPING_AND_PLANNING | Boolean value is: True | 34.67 | 230 | 0.847 / 13.3 | 0.4861 | 1.0672 | 1.411 | 1.5590 / 24.337 | 2 | 13.5 / — / 34.8 | none |
| L_3 | COMPLETED_MAPPING_AND_PLANNING | Boolean value is: True | 34.80 | 230 | 0.849 / 14.7 | 0.4554 | 0.9930 | 1.532 | 1.7242 / 23.935 | 4 | 13.1 / — / 35.6 | none |

### L minus C (Welch t; "beyond spread" = p < 0.05; n = 3 vs 3)
| metric | difference | t | p | beyond spread? |
|---|---|---|---|---|
| mission success (COMPLETED_MAPPING_AND_PLANNING) | +0.00 | — | — | identical |
| lap length, estimated (m) | -0.03 | -0.58 | 0.612 | no |
| distance driven to the return (m) | -0.03 | -0.58 | 0.612 | no |
| exploration duration (s) | -0.5 | -1.40 | 0.269 | no |
| return error lap_pose.json, EKF (m) | +0.000 | +0.00 | 1.000 | no |
| return yaw error lap_pose.json (°) | +0.25 | +0.34 | 0.754 | no |
| SLAM finish distance from start (m) | -0.311 | -4.17 | 0.027 | **yes** |
| wall error RMS (m) | +0.3770 | +31.39 | 0.000 | **yes** |
| wall error median (m) | +0.1854 | +63.30 | 0.000 | **yes** |
| wall error p95 (m) | +0.8185 | +25.99 | 0.000 | **yes** |
| wall error max (m) | +1.2269 | +25.17 | 0.001 | **yes** |
| wall cells within 0.05 m (%) | -42.0 | -99.14 | 0.000 | **yes** |
| wall cells within 0.10 m (%) | -48.8 | -33.39 | 0.000 | **yes** |
| start/finish doubling offset (m) | +1.3465 | +20.46 | 0.002 | **yes** |
| doubling visible (fraction of runs) | +0.00 | — | — | identical |
| loop-closure correction events (indirect, >0.15 m or >1° step) | +3.0 | +5.20 | 0.035 | **yes** |
| largest SLAM correction step (m) | +1.5242 | +27.58 | 0.001 | **yes** |
| largest SLAM correction step (°) | +23.729 | +161.94 | 0.000 | **yes** |
| audit violations (0.25 m / 5°) | +1.3 | +4.00 | 0.057 | no |
| CPU EKF (% core) | -0.5 | -1.79 | 0.171 | no |
| CPU rf2o (% core) | — | — | — | n/a |
| CPU slam_toolbox (% core) | +1.8 | +3.20 | 0.055 | no |


## Truth diagnostic (offline, Gym pose from the /tf bag, start-aligned, rear axle)
| run | EKF heading drift end | EKF position drift end | SLAM pose error end | SLAM pose error RMS / max |
|---|---|---|---|---|
| C_1 | +3.90° | 0.494 m | 0.353 m, +2.70° | 0.196 / 0.357 m |
| C_2 | +3.06° | 0.324 m | 0.361 m, +2.26° | 0.197 / 0.377 m |
| C_3 | +3.61° | 0.500 m | 0.257 m, +1.21° | 0.133 / 0.272 m |
| L_1 | +3.95° | 0.446 m | **0.783 m, -13.76°** | 0.340 / **1.632 m** |
| L_2 | +2.25° | 0.356 m | **0.765 m, -13.59°** | 0.333 / **1.582 m** |
| L_3 | +3.80° | 0.363 m | **0.781 m, -14.23°** | 0.341 / **1.514 m** |

- The EKF odometry is the same with loop closing on or off (it does not see SLAM). Up to ~32.5 m of the 34.7 m lap
  the SLAM pose error of L follows C (~0.2-0.45 m). At ~33 m, as the car re-enters the doubled start area,
  slam_toolbox accepts one loop closure that moves slam_map->fusion_odom by 1.56-1.72 m / ~24° in one step: the
  SLAM pose error jumps from ~0.25-0.47 m to 1.3-1.6 m and its heading from ~+1.5° to ~-25°. Smaller follow-up
  corrections (2.2-2.4°) pull it back to ~0.78 m / -14° by the lap end. The correction is FALSE (it moves the pose
  away from the truth), and the whole saved map is bent: wall RMS 0.46-0.49 m.
- Same failure as the team's Exp 11 (1.6085 m / 25.333°, wall RMSE 0.225 m) — reproduced with my EKF and with
  debug_logging OFF, so it is not caused by debug logging or by the team EKF.
- Start/finish doubling with loop closing ON (1.3-1.5 m) is not a two-copy offset any more: the whole map is
  distorted, so read it as "broken", not as a measured offset.

## Failures / stops
None: 3/3 reached COMPLETED_MAPPING_AND_PLANNING; no new /var/crash files. The mission does NOT detect the bad map:
capture_map_pose.py only refuses jumps > 2 m, and route planning succeeded on all three distorted maps.

Plots: maps_best_worst.png (here = the 3 C maps (best) and the 3 L maps (worst)), wall_error_per_run.png, drift_vs_map.png
