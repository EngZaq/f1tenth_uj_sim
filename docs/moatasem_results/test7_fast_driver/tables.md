
#### Stage C

| metric | C_truth_a3.0 | C_truth_a4.5 | C_truth_a6.0 | C_truth_a3.0_v2.5 | C_truth_a3.0_v2.0 | C_truth_a3.0_v1.75 | C_truth_a3.0_v1.5 | C_truth_a3.0_v1.25 | C_truth_a3.0_v1.0 |
|---|---|---|---|---|---|---|---|---|---|
| end | no safe local speed: measured/proposed stopping tube obstructed | no safe local speed: measured/proposed stopping tube obstructed | no safe local speed: measured/proposed stopping tube obstructed | no safe local speed: measured/proposed stopping tube obstructed | no safe local speed: measured/proposed stopping tube obstructed | no safe local speed: measured/proposed stopping tube obstructed | completed 3 estimated laps | completed 3 estimated laps | completed 3 estimated laps |
| true laps | 0.07 | 0.07 | 0.10 | 0.20 | 0.37 | 0.90 | 3.01 | 3.00 | 3.00 |
| lap times (s) | – | – | – | – | – | – | 32.7, 33.1, 32.1 | 33.4, 32.9, 33.6 | 37.8, 37.1, 37.3 |
| max / 2-s sustained / mean true speed (m/s) | 2.65 / 1.32 / 1.32 | 2.72 / 1.28 / 1.28 | 2.64 / 1.78 / 1.52 | 2.49 / 1.86 / 1.56 | 2.00 / 1.96 / 1.42 | 1.75 / 1.73 / 0.98 | 1.50 / 1.50 / 1.09 | 1.25 / 1.25 / 1.06 | 1.00 / 1.00 / 0.94 |
| true CTE mean / max (cm) | 2.7 / 7.8 | 2.3 / 5.8 | 3.3 / 8.3 | 2.9 / 7.2 | 3.1 / 9.1 | 3.7 / 7.5 | 2.8 / 6.0 | 2.1 / 5.2 | 1.4 / 4.3 |
| min wall clearance, body (m) | 0.345 | 0.376 | 0.290 | 0.297 | 0.308 | 0.301 | 0.301 | 0.306 | 0.308 |
| collisions | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| driver heading err mean / max (°) | 0.80 / 5.02 | 1.17 / 7.73 | 0.99 / 4.96 | 0.50 / 3.98 | 0.76 / 2.30 | 0.42 / 1.67 | 0.53 / 2.12 | 0.57 / 2.04 | 0.54 / 1.73 |
| driver pose err RMSE / max (cm) | 10.3 / 39.1 | 18.1 / 57.8 | 10.5 / 37.1 | 5.3 / 29.3 | 4.9 / 8.8 | 2.1 / 5.5 | 2.9 / 7.0 | 3.1 / 6.4 | 2.9 / 5.1 |
| AMCL err RMSE / max (cm) | – (truth) | – (truth) | – (truth) | – (truth) | – (truth) | – (truth) | – (truth) | – (truth) | – (truth) |
| AMCL σ/real pos · yaw | – (truth) | – (truth) | – (truth) | – (truth) | – (truth) | – (truth) | – (truth) | – (truth) | – (truth) |
| driver along-track err (cm, + ahead) / equiv. delay (ms) | -7.4 / 39 | -11.5 / 53 | -7.8 / 38 | -3.5 / 19 | -4.4 / 29 | -2.0 / 16 | -2.6 / 23 | -2.9 / 27 | -2.7 / 29 |
| AMCL along-track at receive (cm) / delay (ms) | – (truth) | – (truth) | – (truth) | – (truth) | – (truth) | – (truth) | – (truth) | – (truth) | – (truth) |
| EKF vx − true vx curve / straight (mm/s) | -3 / -39 | 19 / -40 | 4 / -42 | 26 / -8 | 7 / 1 | 9 / -1 | 6 / 3 | 5 / 3 | 3 / 3 |
| EKF yaw-rate err RMS curve (°/s) | 0.18 | 0.15 | 0.20 | 0.43 | 0.21 | 0.36 | 0.40 | 0.31 | 0.28 |
| true rear lateral slip RMS curve (mm/s) / max slip angle (°) | 56.6 / 1.54 | 68.9 / 2.27 | 65.4 / 2.54 | 62.2 / 2.07 | 41.8 / 1.74 | 33.6 / 2.10 | 23.1 / 1.85 | 15.0 / 1.13 | 9.9 / 0.86 |
| steering-rate std / max (°/s) | 14.4 / 71 | 16.5 / 80 | 13.3 / 79 | 11.5 / 78 | 9.9 / 62 | 10.3 / 60 | 9.9 / 65 | 9.0 / 51 | 8.0 / 42 |
| largest pose step (cm) / frozen ticks (%) | 18.2 / 4.8 | 15.6 / 10.5 | 21.7 / 6.7 | 19.2 / 2.6 | 15.2 / 0.0 | 13.6 / 0.0 | 12.4 / 0.0 | 10.2 / 0.0 | 8.3 / 0.0 |
| AMCL pose / scan rate (Hz) | – (truth) | – (truth) | – (truth) | – (truth) | – (truth) | – (truth) | – (truth) | – (truth) | – (truth) |
| CPU % amcl / ekf / driver / relay | 0 / 4 / 47 / 0 | 0 / 4 / 52 / 0 | 0 / 4 / 56 / 0 | 0 / 4 / 48 / 0 | 0 / 4 / 46 / 0 | 0 / 4 / 42 / 0 | 0 / 4 / 41 / 0 | 0 / 3 / 39 / 0 | 0 / 3 / 37 / 0 |

#### Stage D

| metric | D_amcl_a3.0_v1.5 | D_amcl_a3.0_v1.25 |
|---|---|---|
| end | completed 5 estimated laps | completed 5 estimated laps |
| true laps | 5.00 | 5.01 |
| lap times (s) | 32.8, 31.7, 34.6, 31.9, 34.3 | 34.4, 32.7, 33.1, 32.0, 33.7 |
| max / 2-s sustained / mean true speed (m/s) | 1.50 / 1.50 / 1.07 | 1.25 / 1.25 / 1.07 |
| true CTE mean / max (cm) | 2.6 / 8.4 | 2.0 / 6.1 |
| min wall clearance, body (m) | 0.335 | 0.343 |
| collisions | 0 | 0 |
| driver heading err mean / max (°) | 0.72 / 4.13 | 0.67 / 2.67 |
| driver pose err RMSE / max (cm) | 4.8 / 19.1 | 4.4 / 14.0 |
| AMCL err RMSE / max (cm) | 5.7 / 20.4 | 5.8 / 16.5 |
| AMCL σ/real pos · yaw | 2.9 · 5.6 | 2.8 · 6.0 |
| driver along-track err (cm, + ahead) / equiv. delay (ms) | 1.4 / -15 | 1.5 / -16 |
| AMCL along-track at receive (cm) / delay (ms) | -0.2 / 0 | 1.6 / -16 |
| EKF vx − true vx curve / straight (mm/s) | 5 / 4 | 3 / 3 |
| EKF yaw-rate err RMS curve (°/s) | 0.36 | 0.31 |
| true rear lateral slip RMS curve (mm/s) / max slip angle (°) | 22.2 / 2.00 | 14.6 / 1.14 |
| steering-rate std / max (°/s) | 11.3 / 78 | 9.6 / 57 |
| largest pose step (cm) / frozen ticks (%) | 13.3 / 0.0 | 10.3 / 0.0 |
| AMCL pose / scan rate (Hz) | 7.9 / 8.4 | 8.3 / 8.4 |
| CPU % amcl / ekf / driver / relay | 7 / 3 / 38 / 68 | 7 / 3 / 37 / 67 |

#### Stage E

| metric | E_shadow_a3.0 | E_shadow_a3.0_r2 | E_shadow_a3.0_r3 | E_shadow_a3.0_r4 | E_shadow_a6.0 | E_shadow_a6.0_r2 | E_shadow_a6.0_r3 | E_shadow_a6.0_r4 | E_shadow_a3.0_v2.5 | E_shadow_a3.0_v2.5_r2 | E_shadow_a3.0_v2.5_r3 | E_shadow_a3.0_v2.5_r4 | E_shadow_a3.0_v2.0 | E_shadow_a3.0_v2.0_r2 | E_shadow_a3.0_v2.0_r3 | E_shadow_a3.0_v2.0_r4 | E_shadow_a3.0_v1.5 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| end | no safe local speed: measured/proposed stopping tube obstructed | estimated pose discontinuity | no safe local speed: measured/proposed stopping tube obstructed | no safe local speed: measured/proposed stopping tube obstructed | no safe local speed: measured/proposed stopping tube obstructed | no safe local speed: measured/proposed stopping tube obstructed | no safe local speed: measured/proposed stopping tube obstructed | no safe local speed: measured/proposed stopping tube obstructed | no safe local speed: measured/proposed stopping tube obstructed | no safe local speed: measured/proposed stopping tube obstructed | no safe local speed: measured/proposed stopping tube obstructed | no safe local speed: measured/proposed stopping tube obstructed | no safe local speed: measured/proposed stopping tube obstructed | no safe local speed: measured/proposed stopping tube obstructed | no safe local speed: measured/proposed stopping tube obstructed | no safe local speed: measured/proposed stopping tube obstructed | completed 5 estimated laps |
| true laps | 0.21 | 0.07 | 0.12 | 0.19 | 0.12 | 0.12 | 0.06 | 0.07 | 0.32 | 0.21 | 0.21 | 0.18 | 0.68 | 0.37 | 0.37 | 0.37 | 5.01 |
| lap times (s) | – | – | – | – | – | – | – | – | – | – | – | – | – | – | – | – | 31.7, 33.5, 30.8, 31.1, 31.3 |
| max / 2-s sustained / mean true speed (m/s) | 2.65 / 2.04 / 1.29 | 2.51 / 1.30 / 1.30 | 2.80 / 2.05 / 1.59 | 2.73 / 2.06 / 1.20 | 3.11 / 2.06 / 1.78 | 3.10 / 2.10 / 1.76 | 2.71 / 1.31 / 1.31 | 2.69 / 1.39 / 1.39 | 2.49 / 2.22 / 1.28 | 2.49 / 1.89 / 1.54 | 2.49 / 1.90 / 1.55 | 2.46 / 1.89 / 1.54 | 2.00 / 1.98 / 1.05 | 2.00 / 1.95 / 1.38 | 2.00 / 1.95 / 1.40 | 2.00 / 1.97 / 1.43 | 1.50 / 1.50 / 1.12 |
| true CTE mean / max (cm) | 3.7 / 8.6 | 2.2 / 5.5 | 4.2 / 9.0 | 3.8 / 8.1 | 4.1 / 13.4 | 4.6 / 12.7 | 2.1 / 5.7 | 2.3 / 7.4 | 3.9 / 8.6 | 2.7 / 7.2 | 2.7 / 7.2 | 2.8 / 7.0 | 4.0 / 8.5 | 3.1 / 8.7 | 3.1 / 8.5 | 3.0 / 8.9 | 2.7 / 6.4 |
| min wall clearance, body (m) | 0.279 | 0.366 | 0.270 | 0.279 | 0.183 | 0.200 | 0.366 | 0.338 | 0.313 | 0.312 | 0.308 | 0.290 | 0.323 | 0.306 | 0.308 | 0.308 | 0.301 |
| collisions | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| driver heading err mean / max (°) | 0.64 / 5.22 | 0.67 / 5.07 | 0.83 / 2.02 | 0.76 / 6.89 | 0.90 / 4.65 | 0.67 / 3.30 | 0.72 / 3.54 | 1.01 / 7.43 | 0.59 / 2.23 | 0.57 / 4.25 | 0.86 / 6.59 | 0.43 / 1.52 | 0.58 / 2.40 | 0.51 / 2.53 | 0.55 / 1.94 | 0.41 / 1.51 | 0.45 / 1.70 |
| driver pose err RMSE / max (cm) | 6.6 / 36.4 | 7.1 / 27.0 | 6.4 / 11.2 | 12.2 / 65.0 | 8.7 / 37.0 | 6.1 / 23.4 | 8.5 / 28.3 | 16.9 / 57.8 | 4.0 / 8.9 | 6.0 / 29.6 | 9.0 / 48.0 | 4.0 / 17.6 | 3.5 / 9.2 | 3.5 / 7.7 | 3.8 / 7.6 | 3.0 / 7.8 | 2.3 / 5.8 |
| AMCL err RMSE / max (cm) | 7.8 / 11.8 | 9.6 / 11.1 | 11.0 / 13.0 | 5.1 / 9.2 | 8.5 / 10.2 | 10.8 / 13.1 | 6.7 / 7.3 | 10.3 / 11.0 | 5.8 / 9.6 | 10.8 / 14.9 | 12.5 / 15.5 | 4.6 / 6.8 | 8.4 / 17.7 | 9.7 / 17.3 | 4.3 / 10.7 | 3.6 / 8.0 | 6.0 / 15.3 |
| AMCL σ/real pos · yaw | 2.6 · 10.2 | 1.2 · 9.6 | 1.4 · 16.2 | 4.1 · 6.6 | 1.6 · 21.8 | 1.4 · 12.1 | 2.0 · 8.0 | 1.5 · 18.5 | 3.8 · 6.7 | 2.1 · 6.9 | 1.7 · 6.3 | 3.5 · 7.9 | 2.3 · 4.8 | 1.8 · 5.0 | 5.4 · 6.0 | 5.0 · 5.2 | 2.8 · 5.7 |
| driver along-track err (cm, + ahead) / equiv. delay (ms) | -5.7 / 32 | -5.0 / 28 | -5.9 / 30 | -8.1 / 47 | -6.9 / 33 | -5.0 / 28 | -6.9 / 39 | -10.2 / 47 | -3.8 / 23 | -4.0 / 23 | -6.2 / 34 | -3.2 / 19 | -3.4 / 24 | -2.8 / 19 | -3.1 / 21 | -2.2 / 15 | -2.1 / 18 |
| AMCL along-track at receive (cm) / delay (ms) | 1.7 / -20 | 3.0 / -20 | 3.5 / -27 | -1.2 / 5 | 1.1 / -12 | 2.3 / -25 | 2.2 / -23 | 3.9 / -37 | 0.9 / -14 | 5.3 / -47 | 7.6 / -62 | 0.3 / -4 | -0.5 / -9 | 1.0 / -5 | -5.4 / 38 | -3.5 / 21 | 1.5 / -14 |
| EKF vx − true vx curve / straight (mm/s) | 16 / -14 | 19 / -40 | 22 / -36 | 20 / -13 | -8 / -35 | 13 / -44 | 4 / -38 | -17 / -43 | 25 / -16 | 28 / -8 | 21 / -10 | 16 / -6 | 19 / -1 | 11 / 2 | 4 / 3 | 11 / -2 | 6 / 3 |
| EKF yaw-rate err RMS curve (°/s) | 0.46 | 0.57 | 0.22 | 0.26 | 0.30 | 0.21 | 0.20 | 0.17 | 0.21 | 0.31 | 0.48 | 1.01 | 0.48 | 0.41 | 0.65 | 0.24 | 0.29 |
| true rear lateral slip RMS curve (mm/s) / max slip angle (°) | 65.5 / 2.41 | 65.4 / 2.10 | 83.3 / 2.35 | 72.7 / 2.48 | 83.6 / 2.18 | 98.9 / 2.56 | 54.8 / 1.51 | 61.5 / 1.73 | 69.8 / 3.01 | 61.0 / 2.14 | 61.2 / 2.25 | 65.6 / 2.16 | 48.7 / 2.44 | 42.7 / 2.02 | 42.5 / 1.85 | 42.8 / 1.86 | 23.9 / 1.72 |
| steering-rate std / max (°/s) | 9.7 / 70 | 13.3 / 72 | 11.2 / 71 | 7.3 / 32 | 12.0 / 71 | 11.7 / 70 | 5.5 / 21 | 16.7 / 80 | 7.4 / 37 | 6.0 / 27 | 5.8 / 28 | 4.0 / 14 | 11.0 / 74 | 10.3 / 60 | 10.5 / 59 | 9.8 / 55 | 9.9 / 63 |
| largest pose step (cm) / frozen ticks (%) | 18.4 / 2.6 | 16.1 / 4.5 | 19.3 / 0.0 | 19.8 / 4.2 | 21.1 / 3.2 | 21.2 / 6.1 | 23.5 / 5.6 | 15.7 / 16.7 | 17.5 / 0.0 | 17.9 / 2.4 | 20.2 / 2.5 | 17.8 / 1.4 | 15.9 / 0.0 | 15.5 / 0.0 | 15.1 / 0.0 | 16.1 / 0.0 | 12.2 / 0.0 |
| AMCL pose / scan rate (Hz) | 6.3 / 8.4 | 5.7 / 8.5 | 6.5 / 8.4 | 6.0 / 8.3 | 6.6 / 8.7 | 7.3 / 8.8 | 5.8 / 8.3 | 6.4 / 8.8 | 6.4 / 8.2 | 7.5 / 8.2 | 7.9 / 8.6 | 7.8 / 8.6 | 6.3 / 8.4 | 8.0 / 8.5 | 8.0 / 8.7 | 8.2 / 8.5 | 7.8 / 8.4 |
| CPU % amcl / ekf / driver / relay | 0 / 4 / 51 / 88 | 0 / 4 / 46 / 88 | 0 / 3 / 48 / 86 | 0 / 4 / 51 / 86 | 0 / 3 / 54 / 89 | 0 / 4 / 54 / 88 | 0 / 4 / 54 / 92 | 0 / 4 / 56 / 90 | 0 / 4 / 46 / 85 | 0 / 4 / 49 / 87 | 0 / 4 / 49 / 88 | 0 / 4 / 48 / 88 | 0 / 4 / 51 / 96 | 0 / 4 / 45 / 85 | 0 / 4 / 46 / 86 | 0 / 4 / 46 / 84 | 0 / 3 / 42 / 75 |
