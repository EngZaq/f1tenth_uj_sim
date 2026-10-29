# UJ F1TENTH — Autonomous Racing Simulation

## Automatic mission - experimental, 29 October 2026

Run `zsh scripts/run_full_autonomy.zsh` from the engineering review checkout for
automatic exploration, fresh mapping, route preparation, AMCL startup and three
estimated-pose laps. The shared laser mounting is corrected to the measured
0.100 m from Gym’s CoG; the rear-axle fusion frame uses the composed 0.27145 m
laser offset. A separate command watchdog owns `/drive`. This integrated workflow
completed exploration, fresh mapping and route preparation in the first laptop
run. The SLAM-to-AMCL handoff now uses bounded fresh ROS graph checks to avoid
stale CLI node listings. A complete live ROS/Gym race remains unverified.
The second laptop run initialized AMCL and drove, then exposed a stricter pause
recovery bound than its normal driving bound. Recovery now uses four fresh,
stationary observations. Later full tests led to route-relative confidence,
uncertainty-based speed reduction and commanded-motion local swept checks. A separate full
ROS/Gym CI workflow exercises the entire mission and retains each attempt.
See the [automatic mission runbook](docs/2026-10-29-autonomous-mission.md) for the
review branch, run command, stopping behavior, evidence and remaining limits.

<!-- 2026-10-28-STATUS-UPDATE -->
## Project status - 28 October 2026

An EKF + Nav2 AMCL estimated-pose controller completed one low-speed lap on the **previous saved technical map**. The integrated **unknown-track first-lap exploration and SLAM mapping** saved a new 225 x 296 occupancy map. A clearance-first extractor generated a 377-waypoint, 37.67 m centerline; the steering-feasible route is 37.579 m. The restarted **new-map** estimated-pose drive reached waypoint 56/377 and stopped safely on excessive estimated CTE (0.170 m) and then stale AMCL pose; that lap is **not complete**. No physical robot actuation is demonstrated in this simulator repository.

**Complete supervisor documentation:** [F1TENTH Simulator Technical Report (PDF)](docs/F1TENTH_UJ_Simulator_Technical_Report_2026-10-28.pdf) and [editable Markdown](docs/F1TENTH_UJ_Simulator_Technical_Report_2026-10-28.md).


**University of Jordan | ROS 2 Humble | F1TENTH Gym | Development status: 27 October 2026**

This repository is the simulation workspace for a 1/10-scale autonomous race car. Our long-term goal is to explore an initially unknown circuit, generate and save a map, derive a safe driving trajectory, estimate the vehicle's pose using onboard sensors, and race with robust safety monitoring. Physical hardware is developed separately in [EngZaq/f1tenth_uj](https://github.com/EngZaq/f1tenth_uj).

> **Current status:** Automated exploration, saved SLAM mapping, map-derived centerline/trajectory generation, simulator-assisted registration, speed profiling, and repeated Pure Pursuit laps have been demonstrated. **Localization from independent LiDAR data is NOT yet validated for driving.** Racing presently uses *simulator ground-truth pose* (`/ego_racecar/odom`).

## Historical results at a glance (through 27 October)

| Subsystem | Result | Important limit |
|---|---|---|
| Unknown-track exploration | Autonomous LiDAR-guided exploration tested | Mapping used simulator odometry |
| Saved mapping | SLAM `.pgm`, `.yaml`, `.posegraph`, `.data` | Pose graph and dataset belong together |
| Planning | Extracted centerline, speed profile and registered trajectory | Registration used privileged simulator geometry |
| Racing | Five consecutive ~26.5 s laps; mean CTE 0.026 m | Controller still reads truth pose |
| RF2O | Smooth estimated odometry in synchronized test | Mean relative position error 0.285 m |
| SLAM localization | ~8 Hz corrections obtained | Mean relative error 0.649 m; unstable jumps |
| EKF (VESC + IMU) | Planned | No Jetson/VESC/IMU integration today |
| Particle-filter localization | Planned | Not installed in simulation workspace |

Full explanations, configurations, experiments and failure analysis: **[docs/2026-10-27-development-log.md](docs/2026-10-27-development-log.md)**.

## Workspace organization

```text
f1tenth_sim/
├── README.md
├── docs/2026-10-27-development-log.md
├── scripts/
│   ├── sim_env.zsh              # ROS domain 42, CycloneDDS, venv, workspace
│   └── run_sim.zsh              # Simulator + Foxglove
├── maps/                        # Technical circuit + previous map fixtures
├── config/                      # SLAM mapping configuration
├── src/f1tenth_control/         # Pure Pursuit racing ROS package
├── tools/
│   ├── exploration/             # Exploration and safe baseline
│   ├── planning/                # Centerline extraction and map registration
│   ├── localization/            # Scan relay, RF2O, SLAM, monitors (experimental)
│   └── generate_speed_profile.py
├── reference/
│   ├── levine/                  # Existing reference experiments
│   └── generated/               # Generated SLAM track paths (created locally)
└── results/
    ├── mapping/                 # Saved map + serialized SLAM graph (created locally)
    └── localization/            # Evaluation data (created locally)
```

**GitHub synchronization caveat:** As of documentation review, the remote `main` branch did not yet contain all of today's new local tools or map artifacts. Files marked “created locally” must be reviewed, staged and pushed *from the development laptop*. Do not claim that everything in the layout is already on GitHub.

## How to run the successful racing baseline

Use **separate zsh terminals**. Confirm `reference/generated/uj_technical_slam_01_registered.csv` and `reference/generated/uj_technical_slam_01_speed_profile.csv` exist on the local laptop.

**Terminal 1 — simulator:**

```bash
cd ~/f1tenth_sim
zsh scripts/run_sim.zsh
```

**Terminal 2 — racing controller (simulation only):**

```bash
cd ~/f1tenth_sim
source scripts/sim_env.zsh
python3 src/f1tenth_control/f1tenth_control/racing_pure_pursuit.py \
  --ros-args \
  -p waypoints_file:="$HOME/f1tenth_sim/reference/generated/uj_technical_slam_01_registered.csv" \
  -p speed_profile_file:="$HOME/f1tenth_sim/reference/generated/uj_technical_slam_01_speed_profile.csv" \
  -p results_file:="$HOME/f1tenth_sim/results/racing_replay.csv" \
  -p use_speed_profile:=true \
  -p speed_scale:=1.0 \
  -p lookahead_distance:=0.5
```

Only **one node** should publish driving commands to `/drive`. The track starts near `(-5.8, -2.1, 2.17 rad)`. The controller uses simulator ground truth and must **not** be run directly on the physical car. The reported 5.0 m/s target parameter is not the actual commanded speed when the profile and scaling are applied.

## Mapping and planning artifacts

- `results/mapping/uj_technical_slam_01.pgm`: occupancy raster.
- `results/mapping/uj_technical_slam_01.yaml`: raster resolution, origin, image reference.
- `results/mapping/uj_technical_slam_01.posegraph`: serialized graph (~30 MB).
- `results/mapping/uj_technical_slam_01.data`: serialized SLAM dataset (~8.4 MB).
- `reference/generated/uj_technical_slam_01_centerline.csv`: centerline in original SLAM-map coordinates.
- `reference/generated/uj_technical_slam_01_registered.csv`: trajectory registered to the simulator world frame.
- `reference/generated/uj_technical_slam_01_speed_profile.csv`: curvature-based planned velocities.

**Critical frame rule:** Simulator `/map` and SLAM `slam_map` are distinct frames. Never feed registered waypoints (simulator frame) to a controller using SLAM-frame pose without an explicit validated transform. The raw map centerline belongs with SLAM-frame localization.

## Localization experiment (diagnostic, not racing-ready)

```text
/scan --relay--> /estimated/scan ---> RF2O ---> estimated_odom -> estimated_base
                           \
                            -> limiter -> /estimated/slam_scan -> SLAM Toolbox
                                                           |
                                                   slam_map -> estimated_odom

/ego_racecar/odom --> comparison monitor ONLY; no truth injected into estimator
```

Use these commands in **separate terminals** after starting the simulator; in each terminal run `cd ~/f1tenth_sim; source scripts/sim_env.zsh` first:

```bash
python3 tools/localization/sim_scan_relay.py
python3 tools/localization/scan_rate_limiter.py
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args --params-file tools/localization/rf2o_sim.yaml
ros2 run slam_toolbox localization_slam_toolbox_node --ros-args --params-file tools/localization/slam_localization_sim.yaml
python3 tools/localization/compare_sources_synced.py
```

Run each on its **own terminal**, not sequentially in a single shell. Do not run `command_model_odom.py` alongside RF2O: both would publish the same estimated TF edge. The initial pose in SLAM config assumes the car is at the beginning of the loop; restarting SLAM in the middle of a lap without relocalization invalidates results.

The simulator publishes privileged `/map`; SLAM's OccupancyGrid must be on `/localized_map` to avoid Foxglove topic collisions. For simulation visualization keep Foxglove fixed frame `map`; don't combine disconnected map-frame overlays without TF.

## What we learned

1. RF2O's default `init_pose_from_topic` waits on `/base_pose_ground_truth`; explicitly set the YAML parameter to `""` to prevent using truth pose as initialization.
2. Restricting SLAM scan input to ~8–10 Hz did not eliminate map-localization errors.
3. SLAM's `minimum_time_interval: 0.5` restricted accepted scans to 2 Hz; switching to `0.08` enabled ~8 Hz `/pose` updates, **but increased update frequency did not solve accuracy**.
4. Synchronized diagnostics found RF2O moving smoothly but drifting, while SLAM corrections jumped substantially. Neither is acceptable as the sole map pose for autonomous racing at present.
5. A command-integrated bicycle model is useful as a simulation experiment, **not** a replacement for measured VESC/encoder + IMU odometry.

## Recommended final architecture

```text
VESC motor telemetry -> calibrated forward wheel speed ---+
                                                        +-> EKF -> odom -> base_link
IMU gyro/acceleration (+ bias calibration) --------------+
                                                         |
LiDAR + saved map + EKF odometry -> particle-filter map localization
                                      |
                              map -> odom correction
                                      |
                 validated map-frame trajectory + Pure Pursuit
                                      |
                           independent collision/stop layer
```

This is a target design, **not an implemented subsystem**. ERPM must be converted with motor pole-pair count, transmission ratio and wheel circumference and calibrated. Servo commands are not measured steering angles. Local odometry and map correction must have distinct TF ownership. Global localization can still produce incorrect matches, so add jump limits, covariance/confidence checks and a stop watchdog.

## Near-term team ownership

- **Sensor fusion:** Moatasem — produce realistic simulated VESC-like speed and IMU measurements with noise/bias, integrate `robot_localization` EKF, report synchronized metrics with at least three seeds.
- **Planning:** Ahmad — retain map-derived centerline and clearance/speed validation; remove dependence on privileged map registration from the production algorithm.
- **Localization integration:** evaluate a particle filter on the saved occupancy map with fused odometry; measure absolute/relative pose errors and relocalization stability.
- **Controller/safety:** keep working oracle-pose baseline unchanged; add estimated-pose driving only after localization passes repeatable tests.

## Repository hygiene

Retain reproducible source, configuration, mapped track, all four saved map files, validated path/speed CSVs and a curated metrics set. Ignore generated ROS `build/`, `install/`, `log/`, `.venv/`, Python caches and editor temp files. Do not blindly delete source files or untracked results: first inspect `git status --short`, copy important data to backup, and verify each cleanup item. `config/slam_sim.yaml.bak` is an example of an obsolete backup candidate if the live YAML file is intact.

See [the detailed log](docs/2026-10-27-development-log.md) for exact metrics and known limitations.
