# AMCL shadow test: raw VESC odometry (A) vs three-sensor EKF (B) vs raw VESC + 1 deg steering play (A_play)
Map: results/missions/autonomous_20261103_114924_818822/new_track.yaml (no-loop; seed (0,0,0), sigma 0.10 m / 5 deg on /fusion/initialpose).
Team AMCL parameters unchanged (config/fusion_amcl_moatasem.yaml). Truth-driven Pure Pursuit, 1 lap per run, sim restarted per run, runs interleaved.
Pose graded = tf2 lookup slam_map->fusion_base (latest) at 30 Hz, replayed from each run's bag in receive order; truth at the same wall time, CoG -> rear axle.
Map alignment: one SE(2) from calibration run CAL_NT (B, not graded):  rotation 123.761 deg, translation (-5.733, -2.191) m; calibration-run residual RMS 0.031 m, max 0.087 m

### Per configuration (mean ± std over 5 runs)
| metric | A · raw VESC odometry | B · three-sensor EKF | A_play · raw VESC, steer play 1°+1° |
|---|---|---|---|
| position RMSE (m) | 0.039 ± 0.005 | 0.045 ± 0.011 | 0.047 ± 0.004 |
| position max (m) | 0.105 ± 0.022 | 0.119 ± 0.021 | 0.112 ± 0.027 |
| position p95 (m) | 0.069 ± 0.011 | 0.085 ± 0.023 | 0.084 ± 0.014 |
| heading mean |err| (°) | 0.66 ± 0.06 | 0.86 ± 0.28 | 2.41 ± 0.13 |
| heading max |err| (°) | 2.38 ± 0.21 | 2.85 ± 0.98 | 7.06 ± 0.57 |
| start-anchored RMSE (m) | 0.092 ± 0.049 | 0.189 ± 0.054 | 0.684 ± 0.084 |
| AMCL corrections in lap | 245 ± 1 | 247 ± 1 | 245 ± 0 |
| jump median (m) | 0.0055 ± 0.0003 | 0.0051 ± 0.0007 | 0.0070 ± 0.0003 |
| largest jump (m) | 0.029 ± 0.003 | 0.024 ± 0.002 | 0.029 ± 0.005 |
| largest jump (°) | 1.00 ± 0.13 | 0.84 ± 0.12 | 1.80 ± 0.09 |
| jumps > 0.05 m | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.0 ± 0.0 |
| jumps > 2° | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.0 ± 0.0 |
| lost episodes (err > 0.5 m) | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.0 ± 0.0 |
| particles (mean) | 501 ± 0 | 501 ± 0 | 501 ± 0 |
| AMCL σxy, first quarter (m) | 0.171 ± 0.013 | 0.172 ± 0.021 | 0.173 ± 0.023 |
| AMCL σxy, last quarter (m) | 0.135 ± 0.005 | 0.136 ± 0.006 | 0.140 ± 0.008 |
| AMCL σyaw mean (°) | 4.61 ± 0.03 | 4.53 ± 0.09 | 4.68 ± 0.15 |
| TF lookup age, median (ms) | 14 ± 1 | 28 ± 16 | 14 ± 0 |
| CPU AMCL (% core) | 15.5 ± 0.9 | 15.9 ± 0.4 | 14.8 ± 0.5 |
| CPU odometry node (% core) | 7.9 ± 0.7 | 11.5 ± 0.4 | 7.4 ± 0.2 |
| CPU rf2o (% core) | 0.0 ± 0.0 | 41.8 ± 0.5 | 0.0 ± 0.0 |

per-run RMSE: {'A': [0.044, 0.038, 0.037, 0.033, 0.045], 'B': [0.039, 0.033, 0.04, 0.055, 0.058], 'P': [0.049, 0.044, 0.047, 0.051, 0.042]}

### B · three-sensor EKF  minus  A · raw VESC odometry  (Welch t-test; "beyond spread" = p < 0.05)
| metric | difference | pooled run-to-run std | t | p | beyond spread? |
|---|---|---|---|---|---|
| position RMSE (m) | +0.006 | 0.009 | +1.06 | 0.333 | no |
| position max (m) | +0.015 | 0.022 | +1.08 | 0.311 | no |
| position p95 (m) | +0.016 | 0.018 | +1.36 | 0.224 | no |
| heading mean |err| (°) | +0.20 | 0.21 | +1.52 | 0.196 | no |
| heading max |err| (°) | +0.47 | 0.71 | +1.05 | 0.348 | no |
| start-anchored RMSE (m) | +0.097 | 0.052 | +2.98 | 0.018 | **yes** |
| AMCL corrections in lap | +2.20 | 1.12 | +3.11 | 0.014 | **yes** |
| jump median (m) | -0.0004 | 0.0005 | -1.10 | 0.316 | no |
| largest jump (m) | -0.006 | 0.002 | -3.88 | 0.005 | **yes** |
| largest jump (°) | -0.17 | 0.13 | -2.12 | 0.067 | no |
| jumps > 0.05 m | +0.0 | 0 | — | — | identical |
| jumps > 2° | +0.0 | 0 | — | — | identical |
| lost episodes (err > 0.5 m) | +0.0 | 0 | — | — | identical |
| particles (mean) | +0.0 | 0 | — | — | identical |
| AMCL σxy, first quarter (m) | +0.001 | 0.017 | +0.08 | 0.941 | no |
| AMCL σxy, last quarter (m) | +0.000 | 0.006 | +0.11 | 0.917 | no |
| AMCL σyaw mean (°) | -0.08 | 0.07 | -1.88 | 0.118 | no |
| TF lookup age, median (ms) | +14.11 | 11.52 | +1.94 | 0.124 | no |
| CPU AMCL (% core) | +0.42 | 0.68 | +0.98 | 0.364 | no |
| CPU odometry node (% core) | +3.62 | 0.59 | +9.71 | 0.000 | **yes** |
| CPU rf2o (% core) | +41.84 | 0.35 | +189.79 | 0.000 | **yes** |

### A_play · raw VESC, steer play 1°+1°  minus  A · raw VESC odometry  (Welch t-test; "beyond spread" = p < 0.05)
| metric | difference | pooled run-to-run std | t | p | beyond spread? |
|---|---|---|---|---|---|
| position RMSE (m) | +0.007 | 0.004 | +2.60 | 0.034 | **yes** |
| position max (m) | +0.007 | 0.025 | +0.46 | 0.656 | no |
| position p95 (m) | +0.015 | 0.013 | +1.82 | 0.108 | no |
| heading mean |err| (°) | +1.75 | 0.10 | +26.53 | 0.000 | **yes** |
| heading max |err| (°) | +4.68 | 0.43 | +17.29 | 0.000 | **yes** |
| start-anchored RMSE (m) | +0.592 | 0.069 | +13.61 | 0.000 | **yes** |
| AMCL corrections in lap | +0.60 | 0.87 | +1.10 | 0.321 | no |
| jump median (m) | +0.0015 | 0.0003 | +7.84 | 0.000 | **yes** |
| largest jump (m) | -0.000 | 0.004 | -0.16 | 0.878 | no |
| largest jump (°) | +0.79 | 0.11 | +11.19 | 0.000 | **yes** |
| jumps > 0.05 m | +0.0 | 0 | — | — | identical |
| jumps > 2° | +0.0 | 0 | — | — | identical |
| lost episodes (err > 0.5 m) | +0.0 | 0 | — | — | identical |
| particles (mean) | +0.0 | 0 | — | — | identical |
| AMCL σxy, first quarter (m) | +0.001 | 0.019 | +0.11 | 0.913 | no |
| AMCL σxy, last quarter (m) | +0.005 | 0.007 | +1.17 | 0.283 | no |
| AMCL σyaw mean (°) | +0.07 | 0.11 | +1.02 | 0.362 | no |
| TF lookup age, median (ms) | -0.53 | 0.73 | -1.14 | 0.304 | no |
| CPU AMCL (% core) | -0.72 | 0.71 | -1.60 | 0.156 | no |
| CPU odometry node (% core) | -0.42 | 0.53 | -1.25 | 0.275 | no |
| CPU rf2o (% core) | +0.0 | 0 | — | — | identical |

plots written to docs/moatasem_results/amcl_shadow
