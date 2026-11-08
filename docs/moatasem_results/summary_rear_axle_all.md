# All configurations with bags, graded at the REAR AXLE (5 runs each, mean ± run-to-run std)

## EKF only (odometry frame, start-aligned like evaluate_fusion.py; debug log ON in these runs)
| config | sensors | rf2o R | rf2o gate | RMSE m | max m | p95 m | heading mean ° | heading max ° | sim stalls > 0.4° | rf2o CPU |
|---|---|---|---|---|---|---|---|---|---|---|
| C1 | VESC+IMU+rf2o | old | off | 0.041 ± 0.005 | 0.075 ± 0.010 | 0.068 ± 0.009 | 0.30 ± 0.06 | 0.78 ± 0.07 | 0/5 | ~42 % |
| C2 | VESC+IMU+rf2o | new | off | 0.047 ± 0.011 | 0.086 ± 0.021 | 0.079 ± 0.021 | 0.39 ± 0.18 | 0.91 ± 0.32 | 1/5 | ~42 % |
| C3 | VESC+IMU+rf2o | old | 3.7 | 0.055 ± 0.016 | 0.106 ± 0.060 | 0.095 ± 0.050 | 0.33 ± 0.19 | 0.83 ± 0.34 | 2/5 | ~42 % |
| C4 (committed) | VESC+IMU+rf2o | new | 3.7 | 0.055 ± 0.013 | 0.097 ± 0.030 | 0.086 ± 0.022 | 0.32 ± 0.08 | 0.85 ± 0.28 | 2/5 | ~42 % |
| E2 | VESC+IMU | — | — | 0.079 ± 0.024 | 0.176 ± 0.099 | 0.157 ± 0.083 | 0.57 ± 0.18 | 1.37 ± 0.53 | 3/5 | (running, unused) |

## AMCL on new_track (map frame, tf2 lookup slam_map->fusion_base at 30 Hz; debug log OFF)
| config | odometry into AMCL | RMSE m | max m | p95 m | heading mean ° | heading max ° | largest jump cm / ° | CPU AMCL / odom / rf2o % | sim stalls > 0.4° |
|---|---|---|---|---|---|---|---|---|---|
| A | raw VESC (commanded steering) | 0.039 ± 0.005 | 0.105 ± 0.022 | 0.069 ± 0.011 | 0.66 ± 0.06 | 2.38 ± 0.21 | 2.9 / 1.00 | 15.5 / 7.9 / 0.0 | 0/5 |
| A_play | raw VESC + 1°+1° steering play | 0.047 ± 0.004 | 0.112 ± 0.027 | 0.084 ± 0.014 | 2.41 ± 0.13 | 7.06 ± 0.57 | 2.9 / 1.80 | 14.8 / 7.4 / 0.0 | 0/5 |
| C | EKF VESC+IMU | 0.048 ± 0.012 | 0.123 ± 0.040 | 0.098 ± 0.035 | 0.87 ± 0.17 | 3.54 ± 1.36 | 2.7 / 0.78 | 14.4 / 9.9 / 0.0 | 0/5 |
| B | EKF VESC+IMU+rf2o (committed) | 0.045 ± 0.011 | 0.119 ± 0.021 | 0.085 ± 0.023 | 0.86 ± 0.28 | 2.85 ± 0.98 | 2.4 / 0.84 | 15.9 / 11.5 / 41.8 | 0/5 |
