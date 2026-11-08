# Test 2 — mapping: team EKF vs my EKF, rf2o yes/no (simulation)

Mapping-only missions (`tools/mission/full_autonomy.py --autonomous --start-simulator --mapping-only --explore-speed 0.18`),
3 runs per configuration, interleaved T, C, CR x3, simulator restarted every run (the mission starts it), nothing else
running, EKF debug off, slam_toolbox `do_loop_closing` verified **False** live in every run. Identical injected sensor noise
(calibrated: gyro bias 0.0003 rad/s, VESC scale 1.003, VESC noise 0.03 m/s, gyro noise 0.003 rad/s, seed 42).

| config | --ekf-config | --sensor-config | rf2o |
|---|---|---|---|
| T | config/fusion_ekf.yaml (template Q, no smoothing) | fusion_sim_sensors_mapping_team.yaml (R by formula: vx 0.000905, gyro 1e-4) | no |
| C | config/fusion_ekf_moatasem.yaml (Q_vx 0.05, smooth_lagged_data) | fusion_sim_sensors_mapping_mine.yaml (R vx 0.0009, gyro 2.5e-5) | no |
| CR | config/fusion_ekf_moatasem_rf2o.yaml | fusion_sim_sensors_mapping_mine.yaml | yes (C1 R, no gate) |

Map wall error: `tools/validation/evaluate_map_walls.py` (rigid ICP to maps/uj_technical_loop_v1; reference used offline only).
Validation: reference itself 0.012 m RMS (grid floor); autonomous_20261103_114924_818822 0.033 m RMS (made with the old
*unbiased* sensor file, i.e. no gyro bias / scale error — so it is not comparable to the runs below).
CPU: /proc ticks while EXPLORING; rf2o from its own per-scan execution-time log (the /proc sample hit the `ros2 run` wrapper).

## Results (mean ± run-to-run std, 3 runs each)
| metric | T · team EKF + team R (3 runs) | C · my EKF (VESC+IMU) + my R (3 runs) | CR · my EKF + rf2o + my R (3 runs) |
|---|---|---|---|
| mission success (COMPLETED_MAPPING_AND_PLANNING) | 1.00 ± 0.00 | 1.00 ± 0.00 | 1.00 ± 0.00 |
| lap length, estimated (m) | 34.70 ± 0.13 | 34.79 ± 0.02 | 34.73 ± 0.11 |
| distance driven to the return (m) | 34.70 ± 0.13 | 34.79 ± 0.02 | 34.73 ± 0.11 |
| exploration duration (s) | 230.2 ± 0.7 | 231.0 ± 0.2 | 230.5 ± 0.6 |
| return error lap_pose.json, EKF (m) | 0.847 ± 0.002 | 0.848 ± 0.002 | 0.847 ± 0.001 |
| return yaw error lap_pose.json (°) | 14.26 ± 1.58 | 13.52 ± 1.01 | 14.14 ± 1.96 |
| SLAM finish distance from start (m) | 0.731 ± 0.097 | 0.676 ± 0.117 | 0.701 ± 0.264 |
| wall error RMS (m) | 0.0667 ± 0.0212 | 0.1001 ± 0.0088 | 0.0675 ± 0.0285 |
| wall error median (m) | 0.0358 ± 0.0061 | 0.0431 ± 0.0007 | 0.0351 ± 0.0075 |
| wall error p95 (m) | 0.1462 ± 0.0567 | 0.2306 ± 0.0225 | 0.1495 ± 0.0747 |
| wall error max (m) | 0.2113 ± 0.0905 | 0.3213 ± 0.0246 | 0.2108 ± 0.0986 |
| wall cells within 0.05 m (%) | 63.6 ± 8.3 | 56.1 ± 0.3 | 65.5 ± 11.8 |
| wall cells within 0.10 m (%) | 87.5 ± 10.6 | 75.2 ± 1.7 | 86.7 ± 12.3 |
| start/finish doubling offset (m) | 0.0764 ± 0.0348 | 0.0696 ± 0.0019 | 0.0833 ± 0.0457 |
| doubling visible (fraction of runs) | 1.00 ± 0.00 | 1.00 ± 0.00 | 1.00 ± 0.00 |
| largest SLAM correction step (m) | 0.0780 ± 0.0054 | 0.0925 ± 0.0219 | 0.0745 ± 0.0133 |
| largest SLAM correction step (°) | 0.400 ± 0.000 | 0.467 ± 0.115 | 0.400 ± 0.000 |
| audit violations (0.25 m / 5°) | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.0 ± 0.0 |
| CPU EKF (% core) | 13.5 ± 0.3 | 13.6 ± 0.2 | 13.5 ± 0.2 |
| CPU rf2o (% core) | — | — | 46.4 ± 2.9 |
| CPU slam_toolbox (% core) | 35.0 ± 1.7 | 33.3 ± 0.9 | 34.4 ± 0.5 |

Per run:
| run | stage | live do_loop_closing | lap (m) | duration (s) | return EKF (m / °) | wall RMS (m) | p95 (m) | doubling (m) | largest SLAM step (m / °) | CPU ekf / rf2o / slam (%) | new /var/crash |
|---|---|---|---|---|---|---|---|---|---|---|---|
| T_1 | COMPLETED_MAPPING_AND_PLANNING | Boolean value is: False | 34.55 | 230 | 0.845 / 15.2 | 0.0744 | 0.1635 | 0.058 | 0.0810 / 0.400 | 13.9 / — / 37.0 | none |
| C_1 | COMPLETED_MAPPING_AND_PLANNING | Boolean value is: False | 34.77 | 231 | 0.849 / 13.7 | 0.0999 | 0.2357 | 0.069 | 0.0917 / 0.400 | 13.4 / — / 32.3 | none |
| CR_1 | COMPLETED_MAPPING_AND_PLANNING | Boolean value is: False | 34.80 | 231 | 0.847 / 12.8 | 0.0387 | 0.0745 | 0.056 | 0.0834 / 0.400 | 13.5 / 44.7 / 33.8 | none |
| T_2 | COMPLETED_MAPPING_AND_PLANNING | Boolean value is: False | 34.73 | 230 | 0.846 / 15.2 | 0.0427 | 0.0829 | 0.054 | 0.0812 / 0.400 | 13.3 / — / 33.7 | none |
| C_2 | COMPLETED_MAPPING_AND_PLANNING | Boolean value is: False | 34.80 | 231 | 0.846 / 14.4 | 0.1090 | 0.2502 | 0.072 | 0.1148 / 0.600 | 13.5 / — / 34.0 | none |
| CR_2 | COMPLETED_MAPPING_AND_PLANNING | Boolean value is: False | 34.60 | 230 | 0.847 / 16.4 | 0.0956 | 0.2239 | 0.136 | 0.0809 / 0.400 | 13.3 / 49.7 / 34.8 | none |
| T_3 | COMPLETED_MAPPING_AND_PLANNING | Boolean value is: False | 34.82 | 231 | 0.849 / 12.4 | 0.0830 | 0.1923 | 0.117 | 0.0717 / 0.400 | 13.3 / — / 34.4 | none |
| C_3 | COMPLETED_MAPPING_AND_PLANNING | Boolean value is: False | 34.81 | 231 | 0.848 / 12.4 | 0.0914 | 0.2060 | 0.068 | 0.0710 / 0.400 | 13.8 / — / 33.7 | none |
| CR_3 | COMPLETED_MAPPING_AND_PLANNING | Boolean value is: False | 34.79 | 231 | 0.848 / 13.2 | 0.0683 | 0.1500 | 0.058 | 0.0593 / 0.400 | 13.6 / 44.8 / 34.6 | none |

### C minus T (Welch t; "beyond spread" = p < 0.05; n = 3 vs 3)
| metric | difference | t | p | beyond spread? |
|---|---|---|---|---|
| mission success (COMPLETED_MAPPING_AND_PLANNING) | +0.00 | — | — | identical |
| lap length, estimated (m) | +0.09 | +1.18 | 0.352 | no |
| distance driven to the return (m) | +0.09 | +1.18 | 0.352 | no |
| exploration duration (s) | +0.8 | +1.80 | 0.188 | no |
| return error lap_pose.json, EKF (m) | +0.001 | +0.67 | 0.542 | no |
| return yaw error lap_pose.json (°) | -0.74 | -0.68 | 0.539 | no |
| SLAM finish distance from start (m) | -0.055 | -0.63 | 0.562 | no |
| wall error RMS (m) | +0.0334 | +2.52 | 0.097 | no |
| wall error median (m) | +0.0073 | +2.05 | 0.174 | no |
| wall error p95 (m) | +0.0844 | +2.40 | 0.109 | no |
| wall error max (m) | +0.1101 | +2.03 | 0.163 | no |
| wall cells within 0.05 m (%) | -7.5 | -1.58 | 0.254 | no |
| wall cells within 0.10 m (%) | -12.3 | -1.99 | 0.178 | no |
| start/finish doubling offset (m) | -0.0068 | -0.34 | 0.766 | no |
| doubling visible (fraction of runs) | +0.00 | — | — | identical |
| largest SLAM correction step (m) | +0.0145 | +1.12 | 0.370 | no |
| largest SLAM correction step (°) | +0.067 | +1.00 | 0.423 | no |
| audit violations (0.25 m / 5°) | +0.0 | — | — | identical |
| CPU EKF (% core) | +0.1 | +0.29 | 0.792 | no |
| CPU rf2o (% core) | — | — | — | n/a |
| CPU slam_toolbox (% core) | -1.7 | -1.50 | 0.230 | no |

### CR minus C (Welch t; "beyond spread" = p < 0.05; n = 3 vs 3)
| metric | difference | t | p | beyond spread? |
|---|---|---|---|---|
| mission success (COMPLETED_MAPPING_AND_PLANNING) | +0.00 | — | — | identical |
| lap length, estimated (m) | -0.06 | -0.91 | 0.452 | no |
| distance driven to the return (m) | -0.06 | -0.91 | 0.452 | no |
| exploration duration (s) | -0.5 | -1.32 | 0.289 | no |
| return error lap_pose.json, EKF (m) | -0.000 | -0.35 | 0.751 | no |
| return yaw error lap_pose.json (°) | +0.62 | +0.49 | 0.658 | no |
| SLAM finish distance from start (m) | +0.025 | +0.15 | 0.890 | no |
| wall error RMS (m) | -0.0326 | -1.89 | 0.178 | no |
| wall error median (m) | -0.0080 | -1.85 | 0.204 | no |
| wall error p95 (m) | -0.0812 | -1.80 | 0.194 | no |
| wall error max (m) | -0.1105 | -1.88 | 0.186 | no |
| wall cells within 0.05 m (%) | +9.4 | +1.38 | 0.301 | no |
| wall cells within 0.10 m (%) | +11.5 | +1.61 | 0.244 | no |
| start/finish doubling offset (m) | +0.0137 | +0.52 | 0.656 | no |
| doubling visible (fraction of runs) | +0.00 | — | — | identical |
| largest SLAM correction step (m) | -0.0180 | -1.22 | 0.304 | no |
| largest SLAM correction step (°) | -0.067 | -1.00 | 0.423 | no |
| audit violations (0.25 m / 5°) | +0.0 | — | — | identical |
| CPU EKF (% core) | -0.1 | -0.67 | 0.542 | no |
| CPU rf2o (% core) | — | — | — | n/a |
| CPU slam_toolbox (% core) | +1.1 | +1.76 | 0.171 | no |

## Truth diagnostic (offline, /tf bag: Gym pose vs EKF odometry and vs SLAM pose, start-aligned, rear axle)
| config | EKF heading drift at lap end (°) | EKF position drift at lap end (m) | SLAM pose error at lap end (m) | SLAM pose error RMS (m) |
|---|---|---|---|---|
| T | 2.31, 2.82, 2.32 | 0.24, 0.23, 0.46 | 0.267, 0.138, 0.271 | 0.162, 0.103, 0.109 |
| C | 3.90, 3.06, 3.61 | 0.49, 0.32, 0.50 | 0.353, 0.361, 0.257 | 0.196, 0.197, 0.133 |
| CR | 3.04, 1.14, 3.28 | 0.46, 0.05, 0.44 | 0.129, 0.265, 0.248 | 0.091, 0.117, 0.145 |

- Map wall RMS follows the SLAM pose error at the lap end (Pearson r = +0.94, p < 0.001, 9 runs) and NOT the EKF
  odometry drift (heading r = -0.01, position r = -0.04). The bad maps are offset on the last stretch before the start
  (bottom of the track): drift accumulated by the end of the lap, never corrected because loop closing is off.
- EKF heading drift C - T = +1.0° (p = 0.031, beyond the spread with 3 runs). Both show the injected gyro bias (~3e-4 rad/s);
  the extra part is a 0.1-0.7 % turn-proportional term that varies run to run in every configuration. The only EKF
  differences T vs C are Q_vx, gyro R and smooth_lagged_data; which one (if any) causes it is not tested here.
- SLAM end error C - T +0.10 m (p = 0.15), CR - C -0.11 m (p = 0.12): not beyond the spread.

## Failures / stops
None: 9/9 reached COMPLETED_MAPPING_AND_PLANNING; no new /var/crash files; correction-step audit PASS in every run
(largest step 0.059-0.115 m, 0.4-0.6°). The Python tracebacks at the end of sensor_emulator.log / cov_relay.log are
rclpy ExternalShutdownException at mission shutdown.

Plots: maps_best_worst.png, wall_error_per_run.png, drift_vs_map.png
