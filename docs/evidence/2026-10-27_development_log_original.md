# Engineering development log — 27 October 2026

Project: University of Jordan F1TENTH graduation project. All results below are **simulation** results unless stated otherwise. No VESC or IMU hardware commissioning was done today.

## 1. Unknown-track exploration and SLAM mapping

Explored the technical closed-loop track using laser observations; generated and saved a SLAM Toolbox occupancy map and graph. Map outputs (local laptop workspace):

| Artifact | Approximate size | Purpose |
|---|---:|---|
| `uj_technical_slam_01.pgm` | 52 KB | Occupancy image |
| `uj_technical_slam_01.yaml` | 138 B | Occupancy map metadata |
| `uj_technical_slam_01.posegraph` | 30 MB | Serialized pose graph |
| `uj_technical_slam_01.data` | 8.4 MB | Serialized dataset |

Important qualification: the mapping experiment used simulator odometry as the pose input. LiDAR exploration did not mean fully onboard state estimation was already solved. Keep all four artifacts together.

## 2. Trajectory extraction, registration and speed profile

- Extracted centerline from generated occupancy map: `reference/generated/uj_technical_slam_01_centerline.csv`.
- Generated a curvature-aware speed profile: `reference/generated/uj_technical_slam_01_speed_profile.csv`.
- Discovered that the raw SLAM map's coordinate system was offset from the simulator world frame. An unregistered trajectory caused a wall collision.
- Simulator-assisted registration produced `reference/generated/uj_technical_slam_01_registered.csv`; diagnostic correction included approximately **−2.24°** rotation, **0.018 m** start discrepancy after alignment, **0.408 m** minimum path clearance.
- This registration is **not** an acceptable substitute for independent map-frame localization on the physical car.

## 3. Pure Pursuit racing successes

The racing controller continues to use `/ego_racecar/odom` (ground-truth simulator pose), follows registered waypoints, uses a speed-profile cap, records cross-track error and contains a local collision-latch fix.

Five consistent laps, lookahead 0.50 m, `speed_scale=1.0` in the reported faster run:

| Lap | Lap time | Mean CTE | Max CTE |
|---|---:|---:|---:|
| 1 | 26.584 s | 0.026 m | 0.091 m |
| 2 | 26.537 s | 0.026 m | 0.058 m |
| 3 | 26.551 s | 0.026 m | 0.054 m |
| 4 | 26.540 s | 0.026 m | 0.056 m |
| 5 | 26.528 s | 0.026 m | 0.057 m |

Mean commanded speed ≈1.29 m/s (range 1.00–1.32 m/s); configured target `5.0 m/s` is *not* the achieved speed. Two later laps at lower speed completed in 29.203 s and 29.177 s, mean CTE 0.028 m and 0.027 m, mean commanded speed ≈1.17 m/s.

## 4. Independent laser odometry and map localization

Developed and tested experimental components in `tools/localization/`:

- `sim_scan_relay.py`: republishes sensor `/scan` in `estimated_laser` frame and creates `estimated_base → estimated_laser` static transform (+0.275 m x in the sim).
- `rf2o_sim.yaml`: configures 20 Hz RF2O to publish `/estimated/rf2o_odom` and `estimated_odom → estimated_base`. `init_pose_from_topic: ""` avoids waiting for privileged ground-truth pose.
- `slam_localization_sim.yaml`: loads saved SLAM pose graph, sets `slam_map` as global frame, uses estimated odom and estimated scan stream; publishes map as `/localized_map` rather than conflicting with simulator `/map`.
- `scan_rate_limiter.py`: throttles the SLAM scan branch to ~8–10 Hz (RF2O still consumes the unthrottled relay scan).
- `compare_localization.py` and `compare_rf2o.py`: early experimental monitors; not timestamp-aligned and can mislead if estimators restart.
- `compare_sources_synced.py`: measures both estimation branches against ground truth using ROS timestamps and a common logging program.
- `command_model_odom.py`: simulation-only dead reckoning from commanded velocity and steering (NOT measured sensors; do not run with RF2O on same TF edge).

The original simulator map briefly disappeared in Foxglove because SLAM also published `/map`. Changing SLAM's output topic to `/localized_map` restored the track visualization. A transient `slam_map frame does not exist` message on startup resolved once TF broadcasting began.

## 5. Tested alternatives and results

These recordings are **different durations and settings**, so comparisons are diagnostic rather than controlled algorithm benchmarks.

| Test | Duration / samples | Mean position error | Maximum error | Key finding |
|---|---|---:|---:|---|
| Early long SLAM comparison | 260.4 s / 1301 | 1.090 m | 3.325 m | Run may have included localization restart; unreliable baseline |
| RF2O only (earlier long trial) | 945 samples | 1.639 m | 10.367 m | Eventually large yaw drift (max 156.9°) |
| RF2O + SLAM (same earlier trial, unsynchronized) | 883 samples | 0.279 m | 1.788 m | SLAM partly corrected RF2O but occasionally jumped |
| Command-bicycle + SLAM | 227 samples | 0.537 m | 1.809 m | Replacing RF2O with command integration did not fix localization |
| RF2O + scan limiter + SLAM | 221 samples / 44.4 s | 0.551 m | 1.532 m | 8–10 Hz scan input insufficient to fix |
| SLAM minimum interval = 0.08 | 357 samples / 71.6 s | 0.651 m | 1.947 m | `/pose` output ~8 Hz, but still wrong |
| **RF2O synchronized** | **1549 samples** | **0.285 m** | **0.593 m** | No >3 m/s estimated-speed spikes; max yaw error 1.99° |
| **SLAM synchronized** | **554 samples** | **0.649 m** | **1.806 m** | Three >3 m/s jumps, peak 17.33 m/s; max heading 17.27° |

The last run's monitor console and later CSV summary had different RF2O sample counts (live print 2001 vs CSV 1549), likely reflecting overwritten/logging windows; use each dataset with its provenance rather than merging as one clean run.

Key synchronized trace: `JUMP SLAM: est=17.33m/s true=1.17m/s`. This is a map-pose correction, not physical vehicle acceleration. RF2O retained smoother heading but its translation accumulated drift.

Parameter diagnostics:

- Original localization `minimum_time_interval: 0.5` accepted a maximum ~2 Hz. New `0.08` allowed ~8 Hz `/pose` messages; errors persisted.
- Scan limiter nominal 10 Hz measured about 8.4 Hz; errors persisted.
- `scan_queue_size: 5`, `minimum_travel_distance: 0.1`, `minimum_travel_heading: 0.1`, `resolution: 0.05`, `map_start_pose: [-5.8,-2.1,2.1686]` during the experiments.
- Initial results could be invalidated if SLAM restarts while driving but the monitor keeps its old initial pose.

## 6. Conclusions / what is actually finished

**Validated for simulation:** Mapping outputs saved, centerline + speed profile generated, simulator path registered, several repeatable low-speed Pure Pursuit laps with low cross-track error.

**Not validated:** end-to-end autonomous racing with onboard-only pose; physical VESC/IMU integration; SLAM map localization robust to lookalike track segments; state estimation with measured wheel/IMU readings; full fail-safe hardware stop chain.

**Do not wire SLAM's current map estimate directly to Pure Pursuit.** Preserve the proven simulator ground-truth racing controller as the reference behavior.

## 7. Recommended next development phase

1. Build a realistic simulated VESC speed + IMU model with measured/bounded noise and bias, then fuse through `robot_localization` EKF. On real hardware, calibrate motor ERPM (pole pairs, gear ratio, tire circumference); steer command is not steer measurement.
2. Add map-relative localization via particle filter (or validated alternative) using saved occupancy map, LiDAR and EKF odometry. Quantify initial localization, jump rejection, drift and confidence.
3. Test map-frame pose against the map-frame path, **not** the simulator-registered path by mistake.
4. Integrate state-estimate watchdog, emergency stop, and conservative speeds before commanding the real vehicle.
5. Move to Jetson only after simulation milestones and hardware power/actuation safeguards are reviewed.

## 8. Files to keep / archival guidance

**Keep intact:** main simulator and controller scripts, stable exploration baseline, mapping config, all 4 map outputs, generated centerline + registered path + speed profile, synchronized monitor, scan relay/RF2O+SLAM configs, representative metrics, and team docs.

**Likely safe to move to archive or delete after review:** `.bak`, temporary `*.tmp`, Python `__pycache__/`, `.pytest_cache/`, duplicate scratch metrics, and obsolete debug prototypes. Do **not** delete unknown untracked source or overwrite prior successful results without checking differences. Use the included script's dry-run and back up before cleanup.
