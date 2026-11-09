# Test 4: localizer check — Nav2 AMCL laser model (Part 1) and F1TENTH particle filter vs tuned AMCL (Part 2)

Simulation only (F1TENTH Gym, ROS 2 Humble), run overnight 9–10 Nov 2026, unattended. Map:
`results/missions/autonomous_20261103_114924_818822/new_track.yaml`. Odometry under every localizer: EKF "C"
(`config/fusion_ekf_moatasem.yaml`, `config/fusion_sim_sensors_calibrated.yaml`, debug off). Simulator truth was used
**only** for offline grading. Every localizer was seeded once on `/fusion/initialpose` (seed (0, 0, 0), never
`/initialpose`). The TF audit of every run confirmed one owner per edge: `slam_map → fusion_odom` = localizer,
`fusion_odom → fusion_base` = EKF (every transform stamped from `/odometry/filtered`), no frame with two parents.

## Answer

**Use Nav2 AMCL (+ EKF C) on the car, not the F1TENTH particle filter.**
AMCL parameters: the team file with **`sigma_hit: 0.05`, `z_rand: 0.1`** (config A3,
`config/fusion_amcl_moatasem_A3.yaml`; all other values unchanged).

- **Particle filter.** It localizes, but worse than AMCL and at a much higher cost:
  - **Error (shadow):** true position RMSE 0.138 vs 0.046 m, and 0.071 m even after its 5 s start-up phase. It makes 55 heading jumps > 2° per lap; AMCL makes none.
  - **CPU:** one full core plus 12 % for the adapter, vs 15 % for AMCL.
  - **Closed loop:** it finished 1 of 5 runs. The other 4 ended in collision-guard stops caused by its heading noise (steering-rate std 21.4 vs 9.2 °/s). Tuned AMCL finished 5 of 5.
- **AMCL tuning (A3).** A3 met the selection rule: no worse true error and no more jumps than A0, with the reported σ closest to the real error.
  - **Calibration gain is small.** The ratio of reported to real error went from 3.8 to 3.0 (position) and 4.6 to 4.6 (yaw). That is not beyond the run-to-run spread (p = 0.14).
  - **AMCL still overstates its σ about 3–4.5× with every laser setting.** So the laser model is not what makes AMCL's covariance pessimistic.
  - **On the car,** sigma_hit 0.05 m assumes the map and LiDAR are as good as in the sim (0.01 m range noise). Re-check it on the first real laps; if real errors are larger, fall back to A0 (true error was identical in sim).

## Setup and decisions taken during the night (no questions could be asked)

| Item | Decision | Why |
|---|---|---|
| AMCL A1–A3 | copies of `config/fusion_amcl_moatasem.yaml`, only `sigma_hit` / `z_rand` changed; **`z_hit` kept 0.7** (A2/A3 weights sum to 0.8) | the task said "only these values changed"; Nav2's likelihood-field model does not require z_hit + z_rand = 1 |
| A0 | the team file itself, unchanged | |
| Shadow runs | archived `run_amcl2.sh` procedure, unchanged steps: fresh simulator, seed, TF audit, bag + sampler, ONE truth-driven Pure Pursuit lap (`uj_technical_slam_01`, ~29 s) | "measured exactly the same way" |
| Grading | archived method: tf2 controller-view replay of the bag at 30 Hz, rear axle, the **same fixed SE(2)** from calibration run CAL_NT (`se2_CAL_NT.json`); jumps from every change of `slam_map → fusion_odom` | same as AMCL-shadow / closed-loop tests |
| "Reported sigma" | sqrt of the `/amcl_pose` covariance x, y, yaw; RMS over the lap. "Real" = RMS of the true error along slam_map x / y and heading. Ratio = reported / real (1 = honest, > 1 = pessimistic) | |
| Tuned-AMCL rule | qualifies if, vs A0, RMSE and heading mean are not higher beyond the spread (Welch p < 0.05) and the mean jump count (> 0.1 m or > 2°) is not higher; then the smallest \|ln ratio_pos\| + \|ln ratio_yaw\| | all four qualified; scores A0 2.86, A1 2.69, A2 3.29, **A3 2.63** |
| Run order | interleaved (P_1 A0_1 A1_1 A2_1 A3_1, A0_2 … P_2, A0_3 … P_3; closed loop T_1 F_1 T_2 F_2 …) | |
| Network guard | as Test 3b before every run (connected + no wlp0s20f3 state change for 60 s), checked after every run | **no Wi-Fi change during any run; no run void** |

### Part 2a — install (finished in ~5 min of the 60 min timebox)
- Packages: **github.com/f1tenth/particle_filter** and **github.com/f1tenth/range_libc**, both branch `humble-devel` (pf commit 672ea43, 2026-10-06; range_libc f55480a). This is the maintained ROS 2 port under the f1tenth org (mit-racecar is ROS 1 only).
- Built in a new workspace **`~/Data/pf_ws`**, with no sudo and no system pip:
  - RangeLibc: built in place (`pywrapper/`, Cython 3.0.12 from a build venv `pf_ws/.venv_build`).
  - particle_filter: `colcon build`.
  - `pf_ws/.venv_pf` holds only `transforms3d 0.4.2`. The Ubuntu 0.3.1 crashes with the sim venv's numpy 1.26 (`np.float`).
- **No CUDA** on this PC (no NVIDIA GPU).

### Part 2b — integration (new files)
- **`tools/localization/pf_shadow.launch.py`**: map_server, EKF C, and the particle filter + adapter. Remaps on the particle filter:
  - `/tf → /pf/tf_raw`
  - `/initialpose → /fusion/initialpose`
  - `/map_server/map → /fusion_map_server/map`. Without this it would load the simulator's own map server.
- **`tools/localization/pf_map_odom_adapter.py`** (adapter, needed): the package hard-codes ONE TF output, `/map → /laser`, with no option to disable it or to publish map→odom. A map→laser edge would give `fusion_laser` a second parent. The adapter therefore:
  - takes the PF laser pose;
  - removes the static `fusion_base → fusion_laser` (0.271 m);
  - composes with the inverse of the EKF `odom → base` pose **of the same odometry message the filter used** (exact stamp match; 0 unmatched updates);
  - publishes **only `slam_map → fusion_odom`**, stamped odometry stamp + 0.3 s like AMCL.
- **Adapter, other jobs:**
  - Moves the package's particle covariance, which `/pf/pose/odom` packs as a 3×3 in `covariance[0:9]`, into the standard 6×6 slots, propagated to the rear axle. Publishes it with the base pose on `/amcl_pose` (only one localizer runs at a time), so the unchanged controller, sampler and seed script work.
  - Relays `/fusion/scan` 1:1 to `/pf/scan`: the filter subscribes RELIABLE and `/fusion/scan` is BEST_EFFORT.
- **`config/particle_filter_moatasem.yaml`**: the package's recommended F1TENTH parameters, unchanged: 4000 particles, angle_step 18 (61 beams), squash 2.2, max_range 10 m, z_hit 0.75 / z_short 0.01 / z_max 0.07 / z_rand 0.12, sigma_hit 8.0 px (= 0.20 m on our 0.025 m map), motion dispersion 0.05 / 0.025 m / 0.25 rad, theta_discretization 112, rangelib_variant 2. Changes:
  - topics;
  - **range_method `rmgpu` → `rm`.** rm is the same ray-marching algorithm on the CPU. Not the README's CPU default `cddt`: RangeLibc's CDDT **segfaults** when particles fall outside the map grid. The package's fixed 0.5 m seed spread does that at our start pose, 0.8 m from the map edge; the first smoke test crashed and I reproduced it offline (`rl_crash_test.py`). `glt` also bounds-checks and is ~8× faster (2.8 vs 21.8 ms per update), but it is a discretized lookup table.
- **Threads:** the particle filter and the adapter run with `OPENBLAS/OMP/MKL_NUM_THREADS=1`. In the first PF run numpy's OpenBLAS busy-waited on every core (268.6 % of a core), and my first adapter used a full `/tf` listener (70 %). Both were fixed after P_1. P_1 was **rerun** and the first attempt kept in `raw_logs/test4/shadow/superseded_P_1_blas8/`; its accuracy was the same (RMSE 0.119 m).
- The package's covariance is **usable**:
  - finite and positive definite;
  - the same order as the real error (ratio 0.77 position, 0.90 yaw);
  - always inside the controller's limits (max across 0.042 / along 0.23 m, yaw 5.8°).
- **Decision:** the uncertainty gate was kept ON for the PF with AMCL's documented limits (`max_across_track_std_m 0.08`, `max_along_track_std_m 0.30`, `max_major_position_std_m 0.40`, `max_heading_std_deg 10`, slowdown start 0.225). **Nothing was disabled.**

### Part 2d — closed loop
- Same as the earlier closed-loop test: archived `run_cl.sh` steps; `profiled_amcl_drive_moatasem.py` (unchanged) → `/mission/drive` → team `drive_watchdog.py --max-speed 0.85`; route `new_track_body_optimized.csv`; cap 0.8 m/s; 2 laps; collision guard on (margin 0.07 m); stale-TF stop on.
- **Guard stops count as results** (not rerun), as in the earlier test.

## Notes, issues and deviations
- **Particle filter start phase.** Most of its error comes in the first ~5 s of the lap (RMSE 0.295 m, up to 0.48 m), in the start/finish area this map holds twice (lap never closed). It starts from its fixed 0.5 m / 0.4 rad seed spread, which ignores the seed covariance. After 5 s its RMSE is 0.071 m vs 0.046 m for A3.
- **Why the PF stops in closed loop.** True CTE stays small (max 0.046 m, same as AMCL), but each PF update (~27 Hz) moves the heading by up to 3.5°:
  - The package adds a fixed 0.25 rad of heading noise per update, at the EKF rate.
  - Pure pursuit turns that into steering jumps: steering-rate std 21.4 vs 9.2 °/s, max 108 vs 43 °/s.
  - In F_5 the guard snapshot shows one update flipping the aim from +8.5° (left, in a left turn) to −1.4°. The predicted stopping tube then crossed the outer wall at every speed → `SAFETY HOLD: no safe local speed`.
  - Three of the four stops are at the same corner (route index 123–137); F_4 stopped at index 323, where A_play also stopped in the earlier test.
  - No collision in any run.
- **Lap times of the PF** come from 2 runs (lap 1) and 1 run (lap 2), so its lap-time t-test means little. Lap times are set by the speed profile, not the localizer.
- **Orphan process (not touched).** `lifecycle_manager_localization` (PID 305196, domain 43) has been running since 8 Nov 10:28, orphaned from an earlier simulator. Stopping it was blocked by the permission system. It is invisible on DDS (`ros2 node list` shows nothing for it) and cannot manage this test's nodes. The run scripts leave it alone.
- **Lifecycle manager leftovers (fixed).** The first runs left the simulator's own lifecycle manager behind each time; the archived scripts killed every lifecycle manager, my first version did not. They manage only the simulator's visual `map_server`, never `fusion_map_server`, and every TF audit passed. I stopped them by PID and the scripts now clean them up.
- **Archived file timestamp changed.** While validating my copy of `eval_cl.py`, I overwrote the archived `raw_logs/cl/eval_C_1.npz`. I regenerated it from the same bag with the unmodified archived `eval_cl.py`. Its keys and values again match the archived `eval_C_1.txt` line exactly; only its timestamp changed. The copy also reproduced the published C_1 numbers, which confirms Test 4's closed-loop metrics are computed the same way.
- **Raw data (not in git):**
  - Runs and logs: `~/Data/f1tenth_results_archive/raw_logs/test4/` (`shadow/`, `cl/`, `smoke/`).
  - Scripts: `~/Data/f1tenth_results_archive/scripts/test4/` (`run_shadow.sh`, `run_cl4.sh`, `guard_batch.sh`, `tf_extract4.py`, `grade4.py`, `sigma_proj4.py`, `eval_cl4.py`, `analyze4_shadow.py`, `analyze4_cl.py`, specs).
  - PF workspace: `~/Data/pf_ws`.

Plots:
- Shadow: `error_vs_time_shadow.png`, `path_on_track_shadow.png`, `sigma_vs_real_shadow.png`, `cpu_bar_shadow.png`.
- Closed loop: `errors_vs_time.png`, `true_path_vs_route.png`, `steering_vs_time.png`, `cpu_rate_closed_loop.png`.

## Part 1 + 2c — shadow laps (car driven by truth; 3 runs per configuration)

### Per configuration (mean ± std over 3 runs)
| metric | A0 · team (σhit 0.10, z_rand 0.3) | A1 · σhit 0.05, z_rand 0.3 | A2 · σhit 0.10, z_rand 0.1 | A3 · σhit 0.05, z_rand 0.1 | PF · F1TENTH particle filter |
|---|---|---|---|---|---|
| TRUE position RMSE (m) | 0.041 ± 0.004 | 0.044 ± 0.008 | 0.040 ± 0.009 | 0.046 ± 0.003 | 0.138 ± 0.002 |
| TRUE position error mean (m) | 0.036 ± 0.003 | 0.036 ± 0.005 | 0.034 ± 0.006 | 0.037 ± 0.003 | 0.098 ± 0.004 |
| TRUE position max (m) | 0.116 ± 0.002 | 0.129 ± 0.038 | 0.103 ± 0.038 | 0.130 ± 0.010 | 0.439 ± 0.034 |
| TRUE position p95 (m) | 0.075 ± 0.009 | 0.089 ± 0.024 | 0.075 ± 0.022 | 0.098 ± 0.015 | 0.352 ± 0.003 |
| TRUE position RMSE, first 5 s of lap (m) | 0.037 ± 0.005 | 0.051 ± 0.009 | 0.042 ± 0.016 | 0.044 ± 0.016 | 0.295 ± 0.009 |
| TRUE position RMSE, from 5 s on (m) | 0.042 ± 0.005 | 0.042 ± 0.009 | 0.039 ± 0.008 | 0.046 ± 0.004 | 0.071 ± 0.004 |
| TRUE position max, from 5 s on (m) | 0.116 ± 0.002 | 0.129 ± 0.038 | 0.103 ± 0.038 | 0.125 ± 0.019 | 0.202 ± 0.030 |
| TRUE heading mean |err| (°) | 0.85 ± 0.13 | 0.76 ± 0.21 | 0.58 ± 0.17 | 0.70 ± 0.13 | 1.85 ± 0.07 |
| TRUE heading max |err| (°) | 2.75 ± 0.70 | 3.01 ± 0.69 | 2.63 ± 0.93 | 2.67 ± 0.40 | 6.73 ± 0.39 |
| real error RMS, x (m) | 0.026 ± 0.005 | 0.027 ± 0.001 | 0.024 ± 0.004 | 0.036 ± 0.004 | 0.092 ± 0.003 |
| real error RMS, y (m) | 0.032 ± 0.001 | 0.034 ± 0.012 | 0.031 ± 0.009 | 0.027 ± 0.008 | 0.103 ± 0.000 |
| real heading error RMS (°) | 1.03 ± 0.17 | 0.96 ± 0.29 | 0.75 ± 0.26 | 0.92 ± 0.19 | 2.46 ± 0.02 |
| reported σx (m) | 0.098 ± 0.006 | 0.089 ± 0.006 | 0.099 ± 0.003 | 0.088 ± 0.001 | 0.088 ± 0.002 |
| reported σy (m) | 0.122 ± 0.009 | 0.107 ± 0.002 | 0.120 ± 0.006 | 0.107 ± 0.003 | 0.060 ± 0.000 |
| reported σyaw (°) | 4.61 ± 0.11 | 4.15 ± 0.11 | 4.60 ± 0.06 | 4.09 ± 0.03 | 2.22 ± 0.02 |
| ratio reported/real, x | 3.92 ± 1.11 | 3.29 ± 0.35 | 4.15 ± 0.60 | 2.47 ± 0.35 | 0.95 ± 0.01 |
| ratio reported/real, y | 3.82 ± 0.39 | 3.54 ± 1.46 | 4.03 ± 1.04 | 4.19 ± 1.48 | 0.59 ± 0.00 |
| ratio reported/real, position | 3.82 ± 0.61 | 3.25 ± 0.52 | 4.06 ± 0.88 | 3.03 ± 0.23 | 0.77 ± 0.00 |
| ratio reported/real, yaw | 4.59 ± 0.89 | 4.55 ± 1.20 | 6.58 ± 2.00 | 4.56 ± 0.91 | 0.90 ± 0.01 |
| jumps > 0.1 m or > 2° | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.0 ± 0.0 | 55.0 ± 5.3 |
| jumps > 0.1 m | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.0 ± 0.0 |
| jumps > 2° | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.0 ± 0.0 | 55.0 ± 5.3 |
| largest jump (m) | 0.029 ± 0.005 | 0.024 ± 0.001 | 0.029 ± 0.004 | 0.024 ± 0.003 | 0.051 ± 0.001 |
| largest jump (°) | 0.84 ± 0.18 | 0.79 ± 0.12 | 0.82 ± 0.03 | 0.83 ± 0.16 | 3.57 ± 0.15 |
| lost episodes (err > 0.5 m) | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.0 ± 0.0 |
| pose updates during lap (Hz) | 8.4 ± 0.0 | 8.4 ± 0.0 | 8.5 ± 0.1 | 8.5 ± 0.0 | 24.8 ± 2.4 |
| CPU localizer (% core) | 15.6 ± 0.4 | 15.0 ± 1.4 | 15.4 ± 0.4 | 15.4 ± 1.1 | 99.4 ± 1.3 |
| CPU adapter (% core) | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.0 ± 0.0 | 12.2 ± 0.9 |
| CPU EKF (% core) | 10.5 ± 0.5 | 11.0 ± 0.5 | 10.2 ± 0.3 | 10.9 ± 0.1 | 11.2 ± 0.1 |
| TF lookup age, median (ms) | 38 ± 1 | 28 ± 17 | 8 ± 3 | 15 ± 11 | 18 ± 12 |

per-run RMSE (m): {'A0': [0.037, 0.044, 0.043], 'A1': [0.035, 0.047, 0.049], 'A2': [0.05, 0.037, 0.032], 'A3': [0.043, 0.048, 0.047], 'P': [0.139, 0.136, 0.139]}
per-run mean error (m): {'A0': [0.032, 0.039, 0.037], 'A1': [0.031, 0.038, 0.04], 'A2': [0.041, 0.033, 0.029], 'A3': [0.034, 0.037, 0.041], 'P': [0.099, 0.101, 0.094]}


### Tuned-AMCL selection
Rule: a candidate qualifies if, vs A0, (1) its TRUE position RMSE and heading mean error are not higher beyond the spread (Welch p < 0.05 with a higher mean), and (2) its mean number of jumps > 0.1 m or > 2° is not higher. Among A0 and the qualifiers, pick the one whose reported sigma is closest to the real error: score = |ln(ratio position)| + |ln(ratio yaw)| (0 = reported equals real). Ties -> A0.
| config | RMSE worse? (p) | heading worse? (p) | jumps (mean) | qualifies | ratio pos | ratio yaw | score |
|---|---|---|---|---|---|---|---|
| A0 | no (1.000) | no (1.000) | 0.0 | yes | 3.82 | 4.59 | 2.86 |
| A1 | no (0.673) | no (0.556) | 0.0 | yes | 3.25 | 4.55 | 2.69 |
| A2 | no (0.772) | no (0.104) | 0.0 | yes | 4.06 | 6.58 | 3.29 |
| A3 | no (0.172) | no (0.239) | 0.0 | yes | 3.03 | 4.56 | 2.63 |

**Tuned AMCL = A3** (A3 · σhit 0.05, z_rand 0.1)


#### A3 · σhit 0.05, z_rand 0.1 minus A0 · team (σhit 0.10, z_rand 0.3) (Welch t; "beyond spread" = p < 0.05)
| metric | difference | t | p | beyond spread? |
|---|---|---|---|---|
| TRUE position RMSE (m) | +0.005 | +1.67 | 0.172 | no |
| TRUE position error mean (m) | +0.001 | +0.55 | 0.613 | no |
| TRUE position max (m) | +0.015 | +2.41 | 0.125 | no |
| TRUE position p95 (m) | +0.024 | +2.37 | 0.090 | no |
| TRUE position RMSE, first 5 s of lap (m) | +0.007 | +0.73 | 0.532 | no |
| TRUE position RMSE, from 5 s on (m) | +0.004 | +0.97 | 0.387 | no |
| TRUE position max, from 5 s on (m) | +0.009 | +0.82 | 0.498 | no |
| TRUE heading mean |err| (°) | -0.15 | -1.38 | 0.239 | no |
| TRUE heading max |err| (°) | -0.08 | -0.16 | 0.882 | no |
| real error RMS, x (m) | +0.010 | +2.62 | 0.061 | no |
| real error RMS, y (m) | -0.005 | -0.98 | 0.426 | no |
| real heading error RMS (°) | -0.10 | -0.71 | 0.516 | no |
| reported σx (m) | -0.010 | -2.65 | 0.110 | no |
| reported σy (m) | -0.016 | -2.95 | 0.078 | no |
| reported σyaw (°) | -0.52 | -7.53 | 0.011 | **yes** |
| ratio reported/real, x | -1.45 | -2.16 | 0.142 | no |
| ratio reported/real, y | +0.37 | +0.42 | 0.713 | no |
| ratio reported/real, position | -0.79 | -2.09 | 0.144 | no |
| ratio reported/real, yaw | -0.03 | -0.04 | 0.972 | no |
| jumps > 0.1 m or > 2° | +0.00 | — | — | identical |
| jumps > 0.1 m | +0.00 | — | — | identical |
| jumps > 2° | +0.00 | — | — | identical |
| largest jump (m) | -0.005 | -1.56 | 0.219 | no |
| largest jump (°) | -0.01 | -0.08 | 0.942 | no |
| lost episodes (err > 0.5 m) | +0.00 | — | — | identical |
| pose updates during lap (Hz) | +0.09 | +3.50 | 0.029 | **yes** |
| CPU localizer (% core) | -0.20 | -0.29 | 0.796 | no |
| CPU adapter (% core) | +0.00 | — | — | identical |
| CPU EKF (% core) | +0.40 | +1.37 | 0.302 | no |
| TF lookup age, median (ms) | -22.93 | -3.72 | 0.062 | no |


#### PF · F1TENTH particle filter minus A3 · σhit 0.05, z_rand 0.1 (Welch t; "beyond spread" = p < 0.05)
| metric | difference | t | p | beyond spread? |
|---|---|---|---|---|
| TRUE position RMSE (m) | +0.092 | +44.81 | 0.000 | **yes** |
| TRUE position error mean (m) | +0.061 | +22.64 | 0.000 | **yes** |
| TRUE position max (m) | +0.309 | +15.21 | 0.002 | **yes** |
| TRUE position p95 (m) | +0.254 | +29.25 | 0.001 | **yes** |
| TRUE position RMSE, first 5 s of lap (m) | +0.251 | +23.94 | 0.000 | **yes** |
| TRUE position RMSE, from 5 s on (m) | +0.025 | +7.13 | 0.002 | **yes** |
| TRUE position max, from 5 s on (m) | +0.077 | +3.69 | 0.028 | **yes** |
| TRUE heading mean |err| (°) | +1.15 | +13.34 | 0.001 | **yes** |
| TRUE heading max |err| (°) | +4.06 | +12.52 | 0.000 | **yes** |
| real error RMS, x (m) | +0.056 | +19.07 | 0.000 | **yes** |
| real error RMS, y (m) | +0.076 | +15.97 | 0.004 | **yes** |
| real heading error RMS (°) | +1.54 | +14.03 | 0.005 | **yes** |
| reported σx (m) | -0.001 | -0.76 | 0.494 | no |
| reported σy (m) | -0.046 | -28.90 | 0.001 | **yes** |
| reported σyaw (°) | -1.88 | -79.86 | 0.000 | **yes** |
| ratio reported/real, x | -1.52 | -7.58 | 0.017 | **yes** |
| ratio reported/real, y | -3.60 | -4.22 | 0.052 | no |
| ratio reported/real, position | -2.26 | -17.07 | 0.003 | **yes** |
| ratio reported/real, yaw | -3.66 | -7.00 | 0.020 | **yes** |
| jumps > 0.1 m or > 2° | +55.00 | +18.00 | 0.003 | **yes** |
| jumps > 0.1 m | +0.00 | — | — | identical |
| jumps > 2° | +55.00 | +18.00 | 0.003 | **yes** |
| largest jump (m) | +0.027 | +16.01 | 0.001 | **yes** |
| largest jump (°) | +2.74 | +21.50 | 0.000 | **yes** |
| lost episodes (err > 0.5 m) | +0.00 | — | — | identical |
| pose updates during lap (Hz) | +16.36 | +11.59 | 0.007 | **yes** |
| CPU localizer (% core) | +83.97 | +85.65 | 0.000 | **yes** |
| CPU adapter (% core) | +12.23 | +24.91 | 0.002 | **yes** |
| CPU EKF (% core) | +0.30 | +6.36 | 0.003 | **yes** |
| TF lookup age, median (ms) | +3.08 | +0.33 | 0.757 | no |


### Particle filter: reported sigma projected on the route like the controller does (vs its limits)
```
P_1 across  sigma m: median 0.034  p95 0.039  p99 0.040  max 0.041  > controller limit 0.08: 0.0 % of updates
P_1 along   sigma m: median 0.072  p95 0.191  p99 0.225  max 0.231  > controller limit 0.3: 0.0 % of updates
P_1 major   sigma m: median 0.073  p95 0.191  p99 0.225  max 0.231  > controller limit 0.4: 0.0 % of updates
P_1 yaw     sigma deg: median 1.96  p99 5.41  max 5.83  > 10 deg: 0.0 %  (643 pose updates)
P_2 across  sigma m: median 0.035  p95 0.039  p99 0.040  max 0.042  > controller limit 0.08: 0.0 % of updates
P_2 along   sigma m: median 0.070  p95 0.181  p99 0.214  max 0.233  > controller limit 0.3: 0.0 % of updates
P_2 major   sigma m: median 0.072  p95 0.181  p99 0.214  max 0.233  > controller limit 0.4: 0.0 % of updates
P_2 yaw     sigma deg: median 1.89  p99 4.92  max 5.52  > 10 deg: 0.0 %  (763 pose updates)
P_3 across  sigma m: median 0.035  p95 0.039  p99 0.041  max 0.043  > controller limit 0.08: 0.0 % of updates
P_3 along   sigma m: median 0.071  p95 0.187  p99 0.233  max 0.247  > controller limit 0.3: 0.0 % of updates
P_3 major   sigma m: median 0.072  p95 0.188  p99 0.233  max 0.247  > controller limit 0.4: 0.0 % of updates
P_3 yaw     sigma deg: median 1.90  p99 5.29  max 6.04  > 10 deg: 0.0 %  (770 pose updates)
```

## Part 2d — closed loop (car steered by the localizer pose; 5 runs each, 2 laps, 0.8 m/s cap)

| metric | AMCL tuned (A3) + EKF C (5 runs) | F1TENTH particle filter + EKF C (5 runs) |
|---|---|---|
| true laps completed | 2.00 ± 0.00 | 0.99 ± 0.70 |
| collisions | 0 ± 0 | 0 ± 0 |
| lap 1 time (s) | 49.13 ± 0.03 | 49.08 ± 0.01 |
| lap 2 time (s) | 48.14 ± 0.01 | 48.07 ± nan |
| TRUE cross-track error mean (m) | 0.015 ± 0.000 | 0.023 ± 0.006 |
| TRUE CTE p95 (m) | 0.036 ± 0.001 | 0.041 ± 0.002 |
| TRUE CTE max (m) | 0.046 ± 0.003 | 0.046 ± 0.001 |
| controller pose error RMSE (m) | 0.047 ± 0.007 | 0.155 ± 0.025 |
| controller pose error max (m) | 0.136 ± 0.025 | 0.392 ± 0.022 |
| controller heading error mean (°) | 0.65 ± 0.09 | 2.30 ± 0.25 |
| controller heading error max (°) | 2.91 ± 0.30 | 7.25 ± 0.56 |
| steering angle std (°) | 6.13 ± 0.03 | 5.69 ± 1.35 |
| steering rate std (°/s) | 9.2 ± 0.1 | 21.4 ± 3.1 |
| steering rate max (°/s) | 42.6 ± 0.9 | 108.2 ± 14.7 |
| largest map->odom jump (m) | 0.067 ± 0.041 | 0.055 ± 0.010 |
| largest map->odom jump (°) | 1.38 ± 0.78 | 3.19 ± 0.35 |
| min predicted body gap, guard (m) | 0.188 ± 0.018 | 0.107 ± 0.054 |
| ticks speed-limited by guard | 0.0 ± 0.0 | 6.6 ± 6.2 |
| controller ticks in localization pause | 0.0 ± 0.0 | 0.0 ± 0.0 |
| max reported across-track σ (m) | 0.024 ± 0.001 | 0.042 ± 0.001 |
| max reported along-track σ (m) | 0.170 ± 0.025 | 0.209 ± 0.019 |
| localizer pose updates while driving (Hz) | 8.3 ± 0.0 | 27.2 ± 0.1 |
| map->odom updates while driving (Hz) | 8.4 ± 0.0 | 27.2 ± 0.1 |
| CPU localizer (% core) | 14.6 ± 0.5 | 100.1 ± 0.1 |
| CPU adapter (% core) | 0.0 ± 0.0 | 12.1 ± 0.2 |
| CPU EKF (% core) | 10.6 ± 0.3 | 10.5 ± 0.2 |

Stops / ends per run:
  T_1: end=laps  controller hold: completed 2 estimated laps  watchdog: -
  T_2: end=laps  controller hold: completed 2 estimated laps  watchdog: -
  T_3: end=laps  controller hold: completed 2 estimated laps  watchdog: -
  T_4: end=laps  controller hold: completed 2 estimated laps  watchdog: -
  T_5: end=laps  controller hold: completed 2 estimated laps  watchdog: -
  F_1: end=laps  controller hold: completed 2 estimated laps  watchdog: -
  F_2: end=hold  controller hold: no safe local speed: measured/proposed stopping tube obstructed  watchdog: -
  F_3: end=hold  controller hold: no safe local speed: measured/proposed stopping tube obstructed  watchdog: -
  F_4: end=hold  controller hold: no safe local speed: measured/proposed stopping tube obstructed  watchdog: -
  F_5: end=hold  controller hold: no safe local speed: measured/proposed stopping tube obstructed  watchdog: -

### F1TENTH particle filter + EKF C minus AMCL tuned (A3) + EKF C (Welch t; "beyond spread" = p < 0.05)
| metric | difference | t | p | beyond spread? |
|---|---|---|---|---|
| true laps completed | -1.01 | -3.21 | 0.033 | **yes** |
| collisions | +0.00 | — | — | identical |
| lap 1 time (s) | -0.05 | -2.96 | 0.032 | **yes** |
| lap 2 time (s) | -0.07 | +nan | nan | no |
| TRUE cross-track error mean (m) | +0.008 | +2.89 | 0.044 | **yes** |
| TRUE CTE p95 (m) | +0.005 | +4.40 | 0.005 | **yes** |
| TRUE CTE max (m) | +0.000 | +0.28 | 0.787 | no |
| controller pose error RMSE (m) | +0.108 | +9.22 | 0.000 | **yes** |
| controller pose error max (m) | +0.255 | +17.00 | 0.000 | **yes** |
| controller heading error mean (°) | +1.66 | +13.78 | 0.000 | **yes** |
| controller heading error max (°) | +4.34 | +15.28 | 0.000 | **yes** |
| steering angle std (°) | -0.43 | -0.72 | 0.512 | no |
| steering rate std (°/s) | +12.20 | +8.72 | 0.001 | **yes** |
| steering rate max (°/s) | +65.61 | +9.95 | 0.001 | **yes** |
| largest map->odom jump (m) | -0.012 | -0.63 | 0.557 | no |
| largest map->odom jump (°) | +1.81 | +4.73 | 0.004 | **yes** |
| min predicted body gap, guard (m) | -0.081 | -3.21 | 0.025 | **yes** |
| ticks speed-limited by guard | +6.60 | +2.38 | 0.076 | no |
| controller ticks in localization pause | +0.00 | — | — | identical |
| max reported across-track σ (m) | +0.017 | +30.97 | 0.000 | **yes** |
| max reported along-track σ (m) | +0.040 | +2.85 | 0.023 | **yes** |
| localizer pose updates while driving (Hz) | +18.95 | +288.98 | 0.000 | **yes** |
| map->odom updates while driving (Hz) | +18.82 | +288.26 | 0.000 | **yes** |
| CPU localizer (% core) | +85.48 | +354.94 | 0.000 | **yes** |
| CPU adapter (% core) | +12.14 | +117.91 | 0.000 | **yes** |
| CPU EKF (% core) | -0.14 | -0.81 | 0.448 | no |

## Appendix — A1 and A2 vs A0 (Welch)

#### A1 · σhit 0.05, z_rand 0.3 minus A0 · team (σhit 0.10, z_rand 0.3) (Welch t; "beyond spread" = p < 0.05)
| metric | difference | t | p | beyond spread? |
|---|---|---|---|---|
| TRUE position RMSE (m) | +0.002 | +0.47 | 0.673 | no |
| TRUE position error mean (m) | +0.000 | +0.15 | 0.892 | no |
| TRUE position max (m) | +0.013 | +0.59 | 0.613 | no |
| TRUE position p95 (m) | +0.015 | +0.99 | 0.405 | no |
| TRUE position RMSE, first 5 s of lap (m) | +0.014 | +2.38 | 0.101 | no |
| TRUE position RMSE, from 5 s on (m) | -0.000 | -0.06 | 0.959 | no |
| TRUE position max, from 5 s on (m) | +0.013 | +0.59 | 0.613 | no |
| TRUE heading mean |err| (°) | -0.09 | -0.65 | 0.556 | no |
| TRUE heading max |err| (°) | +0.26 | +0.46 | 0.669 | no |
| real error RMS, x (m) | +0.001 | +0.39 | 0.733 | no |
| real error RMS, y (m) | +0.001 | +0.22 | 0.848 | no |
| real heading error RMS (°) | -0.06 | -0.33 | 0.764 | no |
| reported σx (m) | -0.009 | -1.76 | 0.154 | no |
| reported σy (m) | -0.015 | -2.93 | 0.092 | no |
| reported σyaw (°) | -0.46 | -5.08 | 0.007 | **yes** |
| ratio reported/real, x | -0.63 | -0.93 | 0.434 | no |
| ratio reported/real, y | -0.28 | -0.32 | 0.774 | no |
| ratio reported/real, position | -0.56 | -1.22 | 0.292 | no |
| ratio reported/real, yaw | -0.04 | -0.05 | 0.962 | no |
| jumps > 0.1 m or > 2° | +0.00 | — | — | identical |
| jumps > 0.1 m | +0.00 | — | — | identical |
| jumps > 2° | +0.00 | — | — | identical |
| largest jump (m) | -0.005 | -1.68 | 0.230 | no |
| largest jump (°) | -0.05 | -0.41 | 0.703 | no |
| lost episodes (err > 0.5 m) | +0.00 | — | — | identical |
| pose updates during lap (Hz) | -0.01 | -0.18 | 0.867 | no |
| CPU localizer (% core) | -0.57 | -0.67 | 0.563 | no |
| CPU adapter (% core) | +0.00 | — | — | identical |
| CPU EKF (% core) | +0.53 | +1.26 | 0.275 | no |
| TF lookup age, median (ms) | -9.39 | -0.95 | 0.441 | no |

#### A2 · σhit 0.10, z_rand 0.1 minus A0 · team (σhit 0.10, z_rand 0.3) (Welch t; "beyond spread" = p < 0.05)
| metric | difference | t | p | beyond spread? |
|---|---|---|---|---|
| TRUE position RMSE (m) | -0.002 | -0.32 | 0.772 | no |
| TRUE position error mean (m) | -0.002 | -0.45 | 0.685 | no |
| TRUE position max (m) | -0.013 | -0.57 | 0.628 | no |
| TRUE position p95 (m) | +0.000 | +0.01 | 0.989 | no |
| TRUE position RMSE, first 5 s of lap (m) | +0.005 | +0.51 | 0.655 | no |
| TRUE position RMSE, from 5 s on (m) | -0.003 | -0.59 | 0.591 | no |
| TRUE position max, from 5 s on (m) | -0.013 | -0.57 | 0.628 | no |
| TRUE heading mean |err| (°) | -0.27 | -2.13 | 0.104 | no |
| TRUE heading max |err| (°) | -0.12 | -0.18 | 0.865 | no |
| real error RMS, x (m) | -0.002 | -0.48 | 0.658 | no |
| real error RMS, y (m) | -0.001 | -0.16 | 0.887 | no |
| real heading error RMS (°) | -0.28 | -1.55 | 0.207 | no |
| reported σx (m) | +0.001 | +0.23 | 0.831 | no |
| reported σy (m) | -0.002 | -0.41 | 0.707 | no |
| reported σyaw (°) | -0.01 | -0.14 | 0.901 | no |
| ratio reported/real, x | +0.23 | +0.31 | 0.775 | no |
| ratio reported/real, y | +0.21 | +0.32 | 0.772 | no |
| ratio reported/real, position | +0.24 | +0.39 | 0.718 | no |
| ratio reported/real, yaw | +1.99 | +1.58 | 0.220 | no |
| jumps > 0.1 m or > 2° | +0.00 | — | — | identical |
| jumps > 0.1 m | +0.00 | — | — | identical |
| jumps > 2° | +0.00 | — | — | identical |
| largest jump (m) | -0.001 | -0.15 | 0.893 | no |
| largest jump (°) | -0.02 | -0.21 | 0.855 | no |
| lost episodes (err > 0.5 m) | +0.00 | — | — | identical |
| pose updates during lap (Hz) | +0.06 | +1.45 | 0.240 | no |
| CPU localizer (% core) | -0.23 | -0.67 | 0.539 | no |
| CPU adapter (% core) | +0.00 | — | — | identical |
| CPU EKF (% core) | -0.27 | -0.79 | 0.484 | no |
| TF lookup age, median (ms) | -29.48 | -16.76 | 0.000 | **yes** |

