### Per run
| run | RMSE | max | p95 | heading mean / max ° | rf2o delay ms min/mean/max (rate Hz) | CPU ekf / rf2o % |
|---|---|---|---|---|---|---|
| Bbase1 | 0.251 | 0.382 | 0.370 | 0.44 / 1.06 | 56 / 98 / 194 (8.4) | 12 / 45 |
| Bbase2 | 0.233 | 0.362 | 0.347 | 0.38 / 0.86 | 38 / 96 / 226 (8.4) | 12 / 46 |
| Bbase3 | 0.220 | 0.344 | 0.331 | 0.33 / 0.78 | 50 / 92 / 217 (8.5) | 12 / 46 |
| Bbase_dbg (debug run, not in mean) | 0.224 | 0.331 | 0.322 | 0.20 / 0.58 | 40 / 92 / 268 (8.3) | 27 / 47 |
| Bsmooth1 | 0.246 | 0.387 | 0.376 | 0.49 / 1.15 | 60 / 103 / 245 (8.4) | 12 / 46 |
| Bsmooth2 | 0.237 | 0.375 | 0.360 | 0.47 / 0.91 | 48 / 93 / 209 (8.4) | 12 / 48 |
| Bsmooth3 | 0.181 | 0.290 | 0.279 | 0.18 / 0.61 | 52 / 99 / 210 (8.6) | 13 / 48 |
| Bsmooth_dbg (debug run, not in mean) | 0.216 | 0.315 | 0.307 | 0.23 / 0.66 | 58 / 118 / 188 (8.4) | 41 / 49 |
| Bbfast1 | 0.244 | 0.387 | 0.368 | 0.35 / 0.80 | 41 / 68 / 168 (19.3) | 12 / 97 |
| Bbfast2 | 0.243 | 0.388 | 0.371 | 0.47 / 1.09 | 41 / 68 / 136 (19.5) | 12 / 97 |
| Bbfast3 | 0.231 | 0.364 | 0.347 | 0.45 / 1.16 | 41 / 69 / 143 (19.2) | 12 / 96 |
| Bbfast_dbg (debug run, not in mean) | 0.232 | 0.359 | 0.346 | 0.34 / 0.73 | 43 / 70 / 143 (18.8) | 30 / 96 |
| Bsfast1 | 0.201 | 0.306 | 0.291 | 0.21 / 0.66 | 45 / 84 / 249 (15.7) | 11 / 86 |
| Bsfast2 | 0.236 | 0.372 | 0.358 | 0.38 / 0.80 | 42 / 68 / 133 (19.2) | 13 / 96 |
| Bsfast3 | 0.254 | 0.401 | 0.390 | 0.56 / 1.10 | 42 / 68 / 171 (19.3) | 13 / 97 |
| Bsfast_dbg (debug run, not in mean) | 0.182 | 0.295 | 0.275 | 0.33 / 0.82 | 44 / 78 / 178 (17.0) | 46 / 92 |

### Per configuration (mean ± std over the 3 non-debug runs; NIS from the debug run)
| config | runs | RMSE | max | p95 | heading mean / max ° | rf2o NIS mean / median / % > 11.8 / trimmed mean | rf2o delay mean ms | EKF CPU % |
|---|---|---|---|---|---|---|---|---|
| B_base | 3 | 0.235 ± 0.016 | 0.363 ± 0.019 | 0.349 ± 0.020 | 0.38 ± 0.06 / 0.90 ± 0.14 | 2.37 / 0.17 / 3.6 % / 1.70 | 96 ± 3 | 12 ± 0 |
| B_smooth | 3 | 0.221 ± 0.035 | 0.351 ± 0.053 | 0.338 ± 0.052 | 0.38 ± 0.17 / 0.89 ± 0.27 | 5.51 / 0.12 / 3.3 % / 1.08 | 98 ± 5 | 12 ± 1 |
| B_base+fast | 3 | 0.239 ± 0.007 | 0.380 ± 0.014 | 0.362 ± 0.013 | 0.42 ± 0.06 / 1.02 ± 0.19 | 2.74 / 0.39 / 3.6 % / 1.88 | 68 ± 1 | 12 ± 0 |
| B_smooth+fast | 3 | 0.230 ± 0.027 | 0.360 ± 0.049 | 0.346 ± 0.051 | 0.38 ± 0.18 / 0.85 ± 0.22 | 4.75 / 0.28 / 3.1 % / 1.28 | 74 ± 9 | 12 ± 1 |

### NIS per sensor (debug run of each config). Expected: mean 1 / median 0.455 (1 dof), 2 / 1.386 (rf2o, 2 dof); 0.27 % above bound
| config (debug run) | sensor | unique updates | mean | median | % above bound | trimmed mean (top 1 % removed) |
|---|---|---|---|---|---|---|
| B_base (Bbase_dbg) | VESC | 2258 | 0.63 | 0.242 | 0.27 | 0.52 |
| B_base (Bbase_dbg) | IMU | 4141 | 1.03 | 0.110 | 3.65 | 0.77 |
| B_base (Bbase_dbg) | rf2o | 416 | 2.37 | 0.172 | 3.61 | 1.70 |
| B_smooth (Bsmooth_dbg) | VESC | 2257 | 0.87 | 0.226 | 0.31 | 0.51 |
| B_smooth (Bsmooth_dbg) | IMU | 4140 | 1.13 | 0.098 | 3.84 | 0.77 |
| B_smooth (Bsmooth_dbg) | rf2o | 422 | 5.51 | 0.118 | 3.32 | 1.08 |
| B_base+fast (Bbfast_dbg) | VESC | 2285 | 0.65 | 0.259 | 0.35 | 0.53 |
| B_base+fast (Bbfast_dbg) | IMU | 4189 | 1.04 | 0.106 | 3.99 | 0.81 |
| B_base+fast (Bbfast_dbg) | rf2o | 1014 | 2.74 | 0.388 | 3.65 | 1.88 |
| B_smooth+fast (Bsfast_dbg) | VESC | 2277 | 0.87 | 0.257 | 0.18 | 0.55 |
| B_smooth+fast (Bsfast_dbg) | IMU | 4151 | 1.11 | 0.095 | 3.64 | 0.72 |
| B_smooth+fast (Bsfast_dbg) | rf2o | 949 | 4.75 | 0.285 | 3.06 | 1.28 |

### Difference vs first config (RMSE)
B_smooth - B_base: -0.013 m  (Welch t = -0.60; ranges B_base [0.220, 0.251] B_smooth [0.181, 0.246] overlap=True)
B_base+fast - B_base: +0.005 m  (Welch t = +0.47; ranges B_base [0.220, 0.251] B_base+fast [0.231, 0.244] overlap=True)
B_smooth+fast - B_base: -0.004 m  (Welch t = -0.24; ranges B_base [0.220, 0.251] B_smooth+fast [0.201, 0.254] overlap=True)
