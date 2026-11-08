# rf2o R x gate test (sim, base = commit 3553ae8, calibrated sensors, 3 sensors, smoothing on)

## STEP 1: rf2o error vs truth (lap S1meas, matched at header stamp)
### S1meas: 248 rf2o messages inside the lap
| axis | match | mean (bias) | std | robust std | |err| p50 | p95 | p99 | max | > 5 robust std |
|---|---|---|---|---|---|---|---|---|---|
| vx (m/s) | stamp | -0.0290 | 0.1464 | 0.0363 | 0.0250 | 0.1732 | 0.7370 | 0.8533 | 12 |
| vx (m/s) | interval avg | -0.0275 | 0.1402 | 0.0326 | 0.0224 | 0.1624 | 0.7305 | 0.8396 | 13 |
| vyaw (rad/s) | stamp | +0.0102 | 0.0579 | 0.0380 | 0.0254 | 0.1140 | 0.2229 | 0.2476 | 6 |
| vyaw (rad/s) | interval avg | +0.0087 | 0.0405 | 0.0104 | 0.0070 | 0.0589 | 0.2135 | 0.2248 | 13 |
NEW R vx = (1.5 x 0.0363)^2 = 0.002957
NEW R vyaw = (1.5 x 0.0380)^2 = 0.003243
spike messages (t since first lap msg, err_vx, err_vyaw): [(0.0, -0.49, 0.232), (0.13, -0.63, 0.045), (0.23, -0.704, 0.159), (0.36, -0.797, 0.121), (0.46, -0.853, 0.23), (0.59, -0.749, 0.203), (0.69, -0.718, 0.195), (0.79, -0.724, 0.214), (0.93, -0.638, 0.248), (1.06, -0.559, 0.16), (14.03, -0.399, 0.096), (15.66, -0.322, 0.13)]

## STEP 2/3 analysis (debug on in all 20 runs; runs interleaved C1..C4 x5)
### a. Per configuration (mean ± run-to-run std over 5 runs)
| config | runs | RMSE m | max m | p95 m | heading mean ° | heading max ° | RMSE run-to-run std | RMSE range |
|---|---|---|---|---|---|---|---|---|
| C1 R old, gate off | 5 | 0.220 ± 0.006 | 0.348 ± 0.014 | 0.334 ± 0.014 | 0.30 ± 0.06 | 0.77 ± 0.07 | 0.006 | 0.214–0.229 |
| C2 R new, gate off | 5 | 0.225 ± 0.020 | 0.358 ± 0.036 | 0.344 ± 0.034 | 0.39 ± 0.18 | 0.91 ± 0.32 | 0.020 | 0.199–0.247 |
| C3 R old, gate 3.7 | 5 | 0.224 ± 0.019 | 0.343 ± 0.029 | 0.329 ± 0.027 | 0.33 ± 0.19 | 0.83 ± 0.34 | 0.019 | 0.205–0.245 |
| C4 R new, gate 3.7 | 5 | 0.231 ± 0.019 | 0.356 ± 0.027 | 0.342 ± 0.028 | 0.32 ± 0.09 | 0.85 ± 0.28 | 0.019 | 0.209–0.259 |

per run RMSE: {'C1': [0.216, 0.222, 0.217, 0.214, 0.229], 'C2': [0.247, 0.244, 0.214, 0.223, 0.199], 'C3': [0.217, 0.243, 0.245, 0.205, 0.208], 'C4': [0.221, 0.232, 0.259, 0.209, 0.235]}

### b. 2x2 effects (cell means m1..m4; pooled within-cell variance, df = 4(N-1))
effect R = mean(C2,C4) - mean(C1,C3); effect gate = mean(C3,C4) - mean(C1,C2); interaction = (C4-C3) - (C2-C1)
| metric | pooled run-to-run std | effect R | t | p | effect gate | t | p | interaction | t | p |
|---|---|---|---|---|---|---|---|---|---|---|
| RMSE | 0.0170 | +0.0067 | +0.88 | 0.392 | +0.0049 | +0.64 | 0.529 | +0.0018 | +0.12 | 0.907 |
| max | 0.0278 | +0.0114 | +0.92 | 0.372 | -0.0040 | -0.32 | 0.752 | +0.0032 | +0.13 | 0.899 |
| p95 | 0.0268 | +0.0118 | +0.98 | 0.339 | -0.0032 | -0.27 | 0.793 | +0.0032 | +0.13 | 0.895 |
| hmean | 0.1415 | +0.0410 | +0.65 | 0.526 | -0.0230 | -0.36 | 0.721 | -0.0940 | -0.74 | 0.468 |
| hmax | 0.2763 | +0.0780 | +0.63 | 0.537 | +0.0000 | +0.00 | 1.000 | -0.1160 | -0.47 | 0.645 |

### a2. Consistency: does the gate shrink the run-to-run spread? (RMSE)
  gate off: pooled within-config std = 0.0150 m
  gate on: pooled within-config std = 0.0189 m
  variance ratio on/off F = 1.59 (df 8,8), two-sided p = 0.525; Brown-Forsythe p = 0.948

### c. rf2o gate rejections (lap window only; truth error from the bags)
spike = |err - STEP-1 median| > 5 robust std (vx 0.181 m/s, vyaw 0.190 rad/s) on either axis; normal = <= 3 robust std on both axes
| run | rf2o msgs | rejected | % | max consecutive (msgs / s) | spikes | spikes rejected | rejected that are spikes | rejected 3-5 rstd | FALSE rejections (normal truth error) |
|---|---|---|---|---|---|---|---|---|---|
| C3_1 | 249 | 11 | 4.4 | 9 / 0.96 | 18 | 11 | 11/11 | 0 | 0 | (109 unmatched)
| C3_2 | 249 | 9 | 3.6 | 8 / 0.86 | 14 | 9 | 9/9 | 0 | 0 | (108 unmatched)
| C3_3 | 247 | 6 | 2.4 | 2 / 0.14 | 10 | 6 | 6/6 | 0 | 0 | (107 unmatched)
| C3_4 | 246 | 2 | 0.8 | 1 / 0.00 | 8 | 2 | 2/2 | 0 | 0 | (112 unmatched)
| C3_5 | 244 | 5 | 2.0 | 3 / 0.26 | 15 | 5 | 5/5 | 0 | 0 | (110 unmatched)
| C4_1 | 246 | 17 | 6.9 | 9 / 0.93 | 18 | 17 | 17/17 | 0 | 0 | (110 unmatched)
| C4_2 | 246 | 21 | 8.5 | 11 / 1.17 | 21 | 21 | 21/21 | 0 | 0 | (110 unmatched)
| C4_3 | 244 | 8 | 3.3 | 3 / 0.26 | 13 | 8 | 8/8 | 0 | 0 | (108 unmatched)
| C4_4 | 245 | 7 | 2.9 | 2 / 0.14 | 9 | 7 | 7/7 | 0 | 0 | (104 unmatched)
| C4_5 | 247 | 13 | 5.3 | 8 / 0.84 | 14 | 13 | 13/13 | 0 | 0 | (111 unmatched)

C3 innovation size, all 5 runs pooled (median [p95]):
  accepted (n=1202): |y_vx| 0.025 [0.110] m/s  |y_vyaw| 0.018 [0.088] rad/s  d2 0.328 [3.858]  truth error / robust std 1.047 [3.432]
  rejected (n=33): |y_vx| 0.492 [0.719] m/s  |y_vyaw| 0.154 [0.238] rad/s  d2 29.177 [61.957]  truth error / robust std 13.570 [19.934]

C4 innovation size, all 5 runs pooled (median [p95]):
  accepted (n=1162): |y_vx| 0.025 [0.098] m/s  |y_vyaw| 0.018 [0.087] rad/s  d2 0.530 [3.959]  truth error / robust std 1.058 [2.962]
  rejected (n=66): |y_vx| 0.443 [0.717] m/s  |y_vyaw| 0.121 [0.216] rad/s  d2 56.426 [134.456]  truth error / robust std 12.294 [19.687]
C1 (gate off) for reference: 1236 msgs, spikes 58, would exceed 3.7 sigma: 32 (2.6 %), of which spikes 32
C2 (gate off) for reference: 1238 msgs, spikes 74, would exceed 3.7 sigma: 69 (5.6 %), of which spikes 69

### d. NIS per sensor (all runs of each config pooled; debug on in every run). Expected mean 1 / median 0.455 / 0.27 % > 9.0 (VESC, IMU); 2 / 1.386 / 0.27 % > 11.8 (rf2o)
| config | sensor | updates | mean | median | % above bound | per-run means |
|---|---|---|---|---|---|---|
| C1 | VESC | 12065 | 0.63 | 0.241 | 0.35 | 0.64, 0.64, 0.60, 0.64, 0.62 |
| C1 | IMU | 22139 | 0.95 | 0.097 | 3.51 | 0.96, 0.95, 0.88, 0.96, 0.99 |
| C1 | rf2o | 2265 | 1.31 | 0.112 | 2.16 | 2.22, 1.40, 1.32, 0.72, 0.91 |
| C2 | VESC | 12019 | 0.85 | 0.253 | 0.47 | 0.90, 0.95, 0.70, 0.97, 0.72 |
| C2 | IMU | 22103 | 1.04 | 0.096 | 3.63 | 1.07, 1.06, 1.03, 1.07, 0.96 |
| C2 | rf2o | 2255 | 4.24 | 0.221 | 3.99 | 5.42, 2.60, 2.17, 8.05, 2.93 |
| C3 | VESC | 12043 | 0.66 | 0.244 | 0.31 | 0.82, 0.65, 0.62, 0.61, 0.61 |
| C3 | IMU | 22093 | 1.00 | 0.095 | 3.66 | 1.11, 0.95, 1.02, 0.98, 0.97 |
| C3 | rf2o | 2264 | 1.83 | 0.100 | 2.16 | 4.42, 1.50, 1.03, 0.84, 1.35 |
| C3 | rf2o accepted only | 2221 | 0.64 | 0.097 | 0.27 | |
| C4 | VESC | 12038 | 0.70 | 0.245 | 0.25 | 0.62, 0.62, 0.61, 0.82, 0.82 |
| C4 | IMU | 22073 | 1.00 | 0.094 | 3.53 | 0.97, 0.93, 0.92, 1.09, 1.07 |
| C4 | rf2o | 2256 | 3.97 | 0.204 | 3.99 | 3.72, 3.51, 2.13, 4.87, 5.63 |
| C4 | rf2o accepted only | 2174 | 0.69 | 0.179 | 0.37 | |

## Simulator truth self-consistency (async wall-clock sim stalls)
```
C1_1 yaw: pose-rate +0.03 deg (max |cum| 0.29)  path: pose-rate +0.034 m of 33.9 m  stall events 0  max stamp gap 18 ms
C1_2 yaw: pose-rate -0.11 deg (max |cum| 0.22)  path: pose-rate +0.045 m of 34.0 m  stall events 0  max stamp gap 12 ms
C1_3 yaw: pose-rate +0.00 deg (max |cum| 0.36)  path: pose-rate +0.033 m of 33.9 m  stall events 1  max stamp gap 17 ms
C1_4 yaw: pose-rate +0.08 deg (max |cum| 0.28)  path: pose-rate +0.045 m of 33.9 m  stall events 0  max stamp gap 15 ms
C1_5 yaw: pose-rate -0.03 deg (max |cum| 0.20)  path: pose-rate +0.046 m of 34.0 m  stall events 0  max stamp gap 11 ms
C2_1 yaw: pose-rate -0.00 deg (max |cum| 0.25)  path: pose-rate +0.037 m of 33.9 m  stall events 0  max stamp gap 15 ms
C2_2 yaw: pose-rate -0.64 deg (max |cum| 0.72)  path: pose-rate +0.027 m of 34.0 m  stall events 1  max stamp gap 16 ms
C2_3 yaw: pose-rate -0.00 deg (max |cum| 0.21)  path: pose-rate +0.046 m of 33.9 m  stall events 0  max stamp gap 11 ms
C2_4 yaw: pose-rate +0.03 deg (max |cum| 0.20)  path: pose-rate +0.042 m of 33.9 m  stall events 0  max stamp gap 15 ms
C2_5 yaw: pose-rate -0.04 deg (max |cum| 0.28)  path: pose-rate +0.048 m of 33.9 m  stall events 0  max stamp gap 11 ms
C3_1 yaw: pose-rate -0.18 deg (max |cum| 0.31)  path: pose-rate +0.050 m of 33.9 m  stall events 0  max stamp gap 12 ms
C3_2 yaw: pose-rate +1.14 deg (max |cum| 1.41)  path: pose-rate -0.040 m of 34.0 m  stall events 4  max stamp gap 17 ms
C3_3 yaw: pose-rate -0.09 deg (max |cum| 0.26)  path: pose-rate +0.047 m of 33.9 m  stall events 1  max stamp gap 16 ms
C3_4 yaw: pose-rate +0.48 deg (max |cum| 0.67)  path: pose-rate -0.001 m of 34.0 m  stall events 1  max stamp gap 18 ms
C3_5 yaw: pose-rate +0.22 deg (max |cum| 0.35)  path: pose-rate +0.043 m of 33.9 m  stall events 0  max stamp gap 10 ms
C4_1 yaw: pose-rate +0.43 deg (max |cum| 0.55)  path: pose-rate -0.028 m of 33.9 m  stall events 4  max stamp gap 17 ms
C4_2 yaw: pose-rate +0.00 deg (max |cum| 0.27)  path: pose-rate +0.044 m of 33.9 m  stall events 0  max stamp gap 10 ms
C4_3 yaw: pose-rate +1.53 deg (max |cum| 1.65)  path: pose-rate -0.011 m of 33.9 m  stall events 3  max stamp gap 16 ms
C4_4 yaw: pose-rate +0.11 deg (max |cum| 0.25)  path: pose-rate +0.033 m of 33.9 m  stall events 0  max stamp gap 13 ms
C4_5 yaw: pose-rate -0.05 deg (max |cum| 0.28)  path: pose-rate +0.048 m of 33.9 m  stall events 0  max stamp gap 10 ms
```
With max |truth yaw inconsistency| as covariate: R +0.0060 m (p 0.39), gate -0.0017 m (p 0.82), RxG +0.0032 (p 0.82), stall +0.022 m/deg (p 0.034); residual std 0.0170 -> 0.0151 m.
