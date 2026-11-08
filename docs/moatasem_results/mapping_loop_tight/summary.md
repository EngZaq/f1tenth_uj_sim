# Test 3b — loop closing ON with tight loop-closure settings, my EKF only (simulation)

Config C (config/fusion_ekf_moatasem.yaml + config/fusion_sim_sensors_mapping_mine.yaml, no rf2o, seed 42, calibrated
noise, explore 0.18, mapping-only) + `--slam-config config/slam_sim_loop_closing_tight.yaml`: loop closing ON with
loop_search_space_dimension 2.0 (was 8.0), loop_search_maximum_distance 2.0 (was 3.0),
loop_match_minimum_response_coarse 0.5 (was 0.35), loop_match_minimum_response_fine 0.6 (was 0.45).
All five values verified on the live slam_toolbox node in every run. Audit: mapping_correction_audit.py --max-gap-s 2.0.
Compared with C (Test 2, loop OFF) and L (Test 3, loop ON default), 3 runs each.

**Network:** the first three LT attempts (10:28-10:34) are VOID: the Wi-Fi hotspot dropped at 10:32:48
("ssid-not-found"); CycloneDDS then could not create nodes (scan_relay / slam_toolbox died at PREFLIGHT, slam_toolbox
crash file) and LT_1's mission stopped in PLANNING on a timed-out `ros2 topic pub`. Rerun 11:05-11:20 with a network
guard (Wi-Fi connected and no state change for 60 s before each run; no wlp0s20f3 state change during each run).
No Wi-Fi change occurred during Test 2, Test 3 or the closed-loop test.

## Results (mean ± run-to-run std)
| metric | C · my EKF (VESC+IMU) + my R (3 runs) | L · my EKF, loop closing ON (3 runs) | LT · my EKF, loop closing ON, tight (3 runs) |
|---|---|---|---|
| mission success (COMPLETED_MAPPING_AND_PLANNING) | 1.00 ± 0.00 | 1.00 ± 0.00 | 1.00 ± 0.00 |
| lap length, estimated (m) | 34.79 ± 0.02 | 34.76 ± 0.08 | 34.80 ± 0.21 |
| distance driven to the return (m) | 34.79 ± 0.02 | 34.76 ± 0.08 | 34.80 ± 0.21 |
| exploration duration (s) | 231.0 ± 0.2 | 230.4 ± 0.6 | 230.9 ± 1.0 |
| return error lap_pose.json, EKF (m) | 0.848 ± 0.002 | 0.848 ± 0.001 | 0.847 ± 0.001 |
| return yaw error lap_pose.json (°) | 13.52 ± 1.01 | 13.77 ± 0.82 | 13.24 ± 1.48 |
| SLAM finish distance from start (m) | 0.676 ± 0.117 | 0.365 ± 0.056 | 0.314 ± 0.119 |
| wall error RMS (m) | 0.1001 ± 0.0088 | 0.4771 ± 0.0189 | 0.3263 ± 0.2092 |
| wall error median (m) | 0.0431 ± 0.0007 | 0.2285 ± 0.0050 | 0.1602 ± 0.1015 |
| wall error p95 (m) | 0.2306 ± 0.0225 | 1.0492 ± 0.0497 | 0.7208 ± 0.4641 |
| wall error max (m) | 0.3213 ± 0.0246 | 1.5482 ± 0.0808 | 1.0250 ± 0.6706 |
| wall cells within 0.05 m (%) | 56.1 ± 0.3 | 14.1 ± 0.7 | 28.2 ± 23.0 |
| wall cells within 0.10 m (%) | 75.2 ± 1.7 | 26.4 ± 1.9 | 45.1 ± 29.4 |
| start/finish doubling offset (m) | 0.0696 ± 0.0019 | 1.4161 ± 0.1140 | 1.0655 ± 0.8125 |
| doubling visible (fraction of runs) | 1.00 ± 0.00 | 1.00 ± 0.00 | 1.00 ± 0.00 |
| loop-closure correction events (indirect, >0.15 m or >1° step) | 0.0 ± 0.0 | 3.0 ± 1.0 | 1.7 ± 0.6 |
| largest SLAM correction step (m) | 0.0925 ± 0.0219 | 1.6167 ± 0.0932 | 1.2400 ± 0.0166 |
| largest SLAM correction step (°) | 0.467 ± 0.115 | 24.196 ± 0.226 | 17.423 ± 4.544 |
| audit violations (0.25 m / 5°) | 0.0 ± 0.0 | 1.3 ± 0.6 | 1.3 ± 0.6 |
| CPU EKF (% core) | 13.6 ± 0.2 | 13.1 ± 0.4 | 13.1 ± 0.2 |
| CPU rf2o (% core) | — | — | — |
| CPU slam_toolbox (% core) | 33.3 ± 0.9 | 35.2 ± 0.4 | 34.7 ± 0.8 |

Per run:
| run | stage | live do_loop_closing | lap (m) | duration (s) | return EKF (m / °) | wall RMS (m) | p95 (m) | doubling (m) | largest SLAM step (m / °) | loop events | CPU ekf / rf2o / slam (%) | new /var/crash |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| C_1 | COMPLETED_MAPPING_AND_PLANNING | Boolean value is: False | 34.77 | 231 | 0.849 / 13.7 | 0.0999 | 0.2357 | 0.069 | 0.0917 / 0.400 | 0 | 13.4 / — / 32.3 | none |
| C_2 | COMPLETED_MAPPING_AND_PLANNING | Boolean value is: False | 34.80 | 231 | 0.846 / 14.4 | 0.1090 | 0.2502 | 0.072 | 0.1148 / 0.600 | 0 | 13.5 / — / 34.0 | none |
| C_3 | COMPLETED_MAPPING_AND_PLANNING | Boolean value is: False | 34.81 | 231 | 0.848 / 12.4 | 0.0914 | 0.2060 | 0.068 | 0.0710 / 0.400 | 0 | 13.8 / — / 33.7 | none |
| L_1 | COMPLETED_MAPPING_AND_PLANNING | Boolean value is: True | 34.83 | 231 | 0.847 / 13.3 | 0.4897 | 1.0873 | 1.305 | 1.5669 / 24.315 | 3 | 12.7 / — / 35.1 | none |
| L_2 | COMPLETED_MAPPING_AND_PLANNING | Boolean value is: True | 34.67 | 230 | 0.847 / 13.3 | 0.4861 | 1.0672 | 1.411 | 1.5590 / 24.337 | 2 | 13.5 / — / 34.8 | none |
| L_3 | COMPLETED_MAPPING_AND_PLANNING | Boolean value is: True | 34.80 | 230 | 0.849 / 14.7 | 0.4554 | 0.9930 | 1.532 | 1.7242 / 23.935 | 4 | 13.1 / — / 35.6 | none |
| LT_1 | COMPLETED_MAPPING_AND_PLANNING | Boolean value is: True | 35.04 | 232 | 0.847 / 12.3 | 0.0847 | 0.1861 | 0.127 | 1.2252 / 12.177 | 2 | 12.9 / — / 34.3 | none |
| LT_2 | COMPLETED_MAPPING_AND_PLANNING | Boolean value is: True | 34.63 | 230 | 0.846 / 14.9 | 0.4514 | 1.0187 | 1.546 | 1.2579 / 19.949 | 2 | 13.2 / — / 34.3 | none |
| LT_3 | COMPLETED_MAPPING_AND_PLANNING | Boolean value is: True | 34.74 | 231 | 0.847 / 12.5 | 0.4427 | 0.9576 | 1.524 | 1.2370 / 20.142 | 1 | 13.1 / — / 35.6 | none |

### LT minus C (Welch t; "beyond spread" = p < 0.05; n = 3 vs 3)
| metric | difference | t | p | beyond spread? |
|---|---|---|---|---|
| mission success (COMPLETED_MAPPING_AND_PLANNING) | +0.00 | — | — | identical |
| lap length, estimated (m) | +0.01 | +0.08 | 0.940 | no |
| distance driven to the return (m) | +0.01 | +0.08 | 0.940 | no |
| exploration duration (s) | -0.1 | -0.17 | 0.881 | no |
| return error lap_pose.json, EKF (m) | -0.001 | -1.06 | 0.379 | no |
| return yaw error lap_pose.json (°) | -0.28 | -0.27 | 0.802 | no |
| SLAM finish distance from start (m) | -0.362 | -3.77 | 0.020 | **yes** |
| wall error RMS (m) | +0.2262 | +1.87 | 0.202 | no |
| wall error median (m) | +0.1171 | +2.00 | 0.184 | no |
| wall error p95 (m) | +0.4902 | +1.83 | 0.209 | no |
| wall error max (m) | +0.7037 | +1.82 | 0.211 | no |
| wall cells within 0.05 m (%) | -27.8 | -2.10 | 0.170 | no |
| wall cells within 0.10 m (%) | -30.1 | -1.77 | 0.218 | no |
| start/finish doubling offset (m) | +0.9959 | +2.12 | 0.168 | no |
| doubling visible (fraction of runs) | +0.00 | — | — | identical |
| loop-closure correction events (indirect, >0.15 m or >1° step) | +1.7 | +5.00 | 0.038 | **yes** |
| largest SLAM correction step (m) | +1.1475 | +72.37 | 0.000 | **yes** |
| largest SLAM correction step (°) | +16.956 | +6.46 | 0.023 | **yes** |
| audit violations (0.25 m / 5°) | +1.3 | +4.00 | 0.057 | no |
| CPU EKF (% core) | -0.5 | -3.35 | 0.032 | **yes** |
| CPU rf2o (% core) | — | — | — | n/a |
| CPU slam_toolbox (% core) | +1.4 | +2.06 | 0.111 | no |

### LT minus L (Welch t; "beyond spread" = p < 0.05; n = 3 vs 3)
| metric | difference | t | p | beyond spread? |
|---|---|---|---|---|
| mission success (COMPLETED_MAPPING_AND_PLANNING) | +0.00 | — | — | identical |
| lap length, estimated (m) | +0.04 | +0.30 | 0.785 | no |
| distance driven to the return (m) | +0.04 | +0.30 | 0.785 | no |
| exploration duration (s) | +0.4 | +0.67 | 0.547 | no |
| return error lap_pose.json, EKF (m) | -0.001 | -1.34 | 0.274 | no |
| return yaw error lap_pose.json (°) | -0.53 | -0.55 | 0.621 | no |
| SLAM finish distance from start (m) | -0.051 | -0.67 | 0.551 | no |
| wall error RMS (m) | -0.1508 | -1.24 | 0.338 | no |
| wall error median (m) | -0.0683 | -1.16 | 0.364 | no |
| wall error p95 (m) | -0.3284 | -1.22 | 0.345 | no |
| wall error max (m) | -0.5232 | -1.34 | 0.309 | no |
| wall cells within 0.05 m (%) | +14.1 | +1.07 | 0.398 | no |
| wall cells within 0.10 m (%) | +18.7 | +1.10 | 0.386 | no |
| start/finish doubling offset (m) | -0.3506 | -0.74 | 0.534 | no |
| doubling visible (fraction of runs) | +0.00 | — | — | identical |
| loop-closure correction events (indirect, >0.15 m or >1° step) | -1.3 | -2.00 | 0.134 | no |
| largest SLAM correction step (m) | -0.3767 | -6.89 | 0.017 | **yes** |
| largest SLAM correction step (°) | -6.773 | -2.58 | 0.123 | no |
| audit violations (0.25 m / 5°) | +0.0 | +0.00 | 1.000 | no |
| CPU EKF (% core) | -0.0 | -0.13 | 0.903 | no |
| CPU rf2o (% core) | — | — | — | n/a |
| CPU slam_toolbox (% core) | -0.4 | -0.88 | 0.442 | no |


## Key question
**Any SLAM correction step > 0.5 m or > 5°?** YES, in every tight run: 1.23 m / 12.2° (LT_1), 1.26 m / 19.9° (LT_2),
1.24 m / 20.1° (LT_3). All happen 4-8 s before the lap is detected (SAVING_MAP) and ~11-15 s before the map file is
written, i.e. inside the saved map. Smaller than the default (1.56-1.72 m / ~24°) but the same kind of false closure.

**Does wall RMS beat C's 0.100 m?** Mean NO: 0.326 ± 0.209 m. One run yes: LT_1 0.085 m (p95 0.186) — its jump came
later (231 s, 12°) and bent only the start/left section; LT_2 / LT_3 are as broken as the default L maps (0.45 / 0.44 m).

## Truth diagnostic (offline, start-aligned, rear axle)
| run | EKF heading drift end | SLAM pose error end | SLAM pose error RMS / max |
|---|---|---|---|
| LT_1 | +3.59° | 0.450 m, -7.59° | 0.331 / 0.767 m |
| LT_2 | +3.50° | 0.911 m, -15.15° | 0.223 / 1.124 m |
| LT_3 | +2.69° | 0.732 m, -12.95° | 0.194 / 1.186 m |
| C (Test 2) | +3.1 to +3.9° | 0.26-0.36 m, +1.2 to +2.7° | 0.13-0.20 / 0.27-0.38 m |
| L (Test 3) | +2.3 to +4.0° | 0.77-0.78 m, ~-14° | 0.33-0.34 / 1.51-1.63 m |

Every loop-closure correction moved the SLAM pose AWAY from the truth (heading error flips from ~+2° to -8...-15°).

Plots: maps_tight_runs.png (all three LT maps next to C_3 and L_2), maps_best_worst.png (best/worst 3 of all 9),
wall_error_per_run.png, drift_vs_map.png
