# UJ F1TENTH Simulation Workspace

**ROS 2 Humble | University of Jordan | Simulation, autonomy development, and reproducible experiments**

This repository develops and evaluates autonomous driving software **before deployment to the physical F1TENTH car**. It contains our ROS 2 control package, Levine reference trajectories, curvature-aware speed-profile tooling, and simulation results. Upstream `f1tenth_gym` and `f1tenth_gym_ros` are installed separately using `simulation_dependencies.repos` rather than committed into this repository.

> **Team priority (October 26 – November 1, 2026):** Moatasem implements sensor fusion and Ahmad implements map-based planning **in simulation**, with code, tests, plots and Pull Requests. See [Week sprint assignments](#week-sprint-assignments). Waiting for the real car is **not a dependency** for either assignment.

## Contents

1. [Project vision](#project-vision)
2. [What currently works](#what-currently-works)
3. [Repository structure](#repository-structure)
4. [System architecture and ROS topics](#system-architecture-and-ros-topics)
5. [Installation](#installation)
6. [Run the existing simulation](#run-the-existing-simulation)
7. [Understand the existing code](#understand-the-existing-code)
8. [Using a different track](#using-a-different-track)
9. [Week sprint assignments](#week-sprint-assignments)
10. [Contribution rules](#contribution-rules)
11. [Known gaps and roadmap](#known-gaps-and-roadmap)

## Project vision

Our **final physical vehicle** should start on an unfamiliar closed-loop track, explore and create an initial map, continue refining its map on later laps, localize reliably, generate a safe trajectory, drive autonomously using Pure Pursuit, and increase speed after *validated successful laps* up to a calibrated safety threshold. Safety faults or unacceptable tracking/localization must prevent speed increases.

This is **not yet implemented**. The simulation repository currently provides a racing-controller baseline; the physical repo [f1tenth_uj](https://github.com/EngZaq/f1tenth_uj) provides LiDAR, IMU, VESC, RF2O and SLAM bringup foundations. We will build reusable navigation components incrementally.

The simulator has access to true pose and a known track map. For realistic autonomy experiments, treat them as **evaluation ground truth only**: algorithms should consume realistic sensors and internally produced estimates. The current Pure Pursuit controller uses simulator odometry and is an **oracle-pose baseline**, not the end-to-end autonomous solution.

## What currently works

- F1TENTH Gym/ROS 2 Humble simulator integration and visualization (as documented in this repo).
- ROS 2 `f1tenth_control` package:
  - `path_publisher`: visualizes a loaded path on `/reference_path`.
  - `pure_pursuit`: constant-speed trajectory tracking.
  - `racing_pure_pursuit`: follows a waypoint-aligned speed profile and records lap statistics.
- A **safe Levine centerline** with **619 points** and length approximately **62.56 m**.
- `tools/generate_speed_profile.py`: calculates curvature-based speed caps plus acceleration and braking passes around a loop.
- Existing experiment CSVs in `results/`, including `pure_pursuit_metrics.csv` and `racing_profile_metrics_v2.csv`.

Representative profile-based Levine laps in the checked-in results are around **14.6–14.9 s**. These are *simulator measurements* and **not** physically verified performance.

Not implemented here yet: simulated sensor fusion with noise, automatic map-derived centerline, autonomous first-lap exploration, loop-closure decisions, map-frame localization, robust command arbitration and emergency-stop supervision, and lap-by-lap speed adaptation.

## Repository structure

```text
f1tenth_uj_sim/
├── README.md
├── simulation_dependencies.repos       # Upstream f1tenth_gym + gym_ros repos
├── src/
│   ├── f1tenth_control/                # Existing project ROS package
│   │   ├── f1tenth_control/
│   │   │   ├── path_publisher.py
│   │   │   ├── pure_pursuit.py
│   │   │   └── racing_pure_pursuit.py
│   │   ├── package.xml
│   │   └── setup.py
│   ├── f1tenth_localization/           # TO CREATE: Moatasem
│   └── f1tenth_planning/               # TO CREATE: Ahmad
├── reference/levine/
│   ├── levine_centerline_safe.csv
│   ├── levine_centerline_original.csv
│   ├── levine_raceline_original.csv
│   ├── levine_candidate.csv
│   ├── levine_speed_profile.csv
│   └── levine_planning_map.yaml
├── results/
│   ├── pure_pursuit_metrics.csv
│   ├── racing_profile_metrics.csv
│   └── racing_profile_metrics_v2.csv
└── tools/
    └── generate_speed_profile.py
```

**Note:** New package paths above are proposals, not currently committed directories. Upstream simulator source is also absent from the project Git tree by design. The Levine planning-map YAML references an upstream map image; check that it exists in your local simulator checkout.

## System architecture and ROS topics

```text
                  F1TENTH GYM (simulation)
                  ├── /scan       (LiDAR)
                  ├── /map        (known world map; privileged)
                  ├── /ego_racecar/odom (true simulator pose; oracle)
                  └── /ego_racecar/collision
                          |
             +------------+-----------------+
             |                              |
   Existing Pure Pursuit                New sensor emulator
   (oracle-pose baseline)                + fusion EKF
             |                              |
             |                       /odometry/filtered
             |                              |
             +--------- future map-frame localization
                          |
           Map-derived centerline + speed planner
                          |
                   Shared controller
                          |
                  Safety supervisor
                          |
                        /drive
                          |
                    F1TENTH Gym
```

| Topic | Type | Intended use |
|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | Sensor observations |
| `/map` | `nav_msgs/msg/OccupancyGrid` | **Planning test input only**; actual first-lap SLAM should build its own map |
| `/ego_racecar/odom` | `nav_msgs/msg/Odometry` | Existing controller baseline and evaluator; *truth input only to isolated sensor emulator* |
| `/ego_racecar/collision` | `std_msgs/msg/Bool` | Simulator collision status |
| `/drive` | `ackermann_msgs/msg/AckermannDriveStamped` | Speed and steering commands |
| `/reference_path` | `nav_msgs/msg/Path` | Existing reference trajectory visualization |
| `/sim/wheel_odom_noisy` | `nav_msgs/msg/Odometry` | **Proposed:** emulated wheel odometry |
| `/sim/imu_noisy` | `sensor_msgs/msg/Imu` | **Proposed:** emulated IMU |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | **Proposed:** fused odometry |
| `/planning/centerline` | `nav_msgs/msg/Path` | **Proposed:** collision-checked map-derived path |

**Important coordinate-frame distinction:** `odom` is a drifting local estimate; `map` is the corrected global map frame. A controller following map-frame waypoints needs a **map-frame vehicle pose**. Do not rename `/odometry/filtered` to make it appear map-relative without a valid `map → odom` transform/localization step. The simulator's TF child frame may be `ego_racecar/base_link` rather than `base_link`; inspect it before configuring fusion.

## Installation

### Requirements

- Ubuntu 22.04 (recommended development target)
- ROS 2 Humble, Python 3.10
- `git`, `python3-colcon-common-extensions`, `python3-vcstool`, `python3-rosdep`
- Dependencies required by the checked-out F1TENTH Gym and ROS bridge; these may require additional Python packages or an appropriately configured Python virtual environment.

### Clone repository and upstream dependencies (Bash)

```bash
sudo apt update
sudo apt install -y git python3-colcon-common-extensions python3-vcstool python3-rosdep

git clone https://github.com/EngZaq/f1tenth_uj_sim.git ~/f1tenth_sim
cd ~/f1tenth_sim

mkdir -p src
vcs import src < simulation_dependencies.repos
source /opt/ros/humble/setup.bash

# Initialize rosdep once if not already initialized on the machine:
# sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y

colcon build --symlink-install
source install/setup.bash
```

The `.repos` file pins upstream **branch names** (`dev-humble`), not immutable commit SHAs. If reproducibility matters, record the checked-out commit IDs in experiment notes. A Python venv that hides system ROS packages can break imports; follow the upstream simulator's environment instructions if this happens.

Build success has **not been independently verified on a clean machine** for this README update. If it fails, record the first error and the exact upstream dependency versions; do not skip failed packages and claim installation succeeded.

### Every new terminal

```bash
cd ~/f1tenth_sim
source /opt/ros/humble/setup.bash
source install/setup.bash
```

For `zsh`, replace `setup.bash` with `setup.zsh`.

## Run the existing simulation

### Terminal 1 — start simulator bridge

```bash
cd ~/f1tenth_sim
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py open_foxglove:=false
```

### Terminal 2 — verify data

```bash
source /opt/ros/humble/setup.bash
source ~/f1tenth_sim/install/setup.bash
ros2 node list
ros2 topic list
ros2 topic echo /ego_racecar/odom --once
ros2 topic echo /scan --once
ros2 topic echo /map --once
ros2 topic hz /scan
```

### Terminal 3 — optional path visualization

```bash
source /opt/ros/humble/setup.bash
source ~/f1tenth_sim/install/setup.bash
ros2 run f1tenth_control path_publisher --ros-args \
  -p waypoints_file:=$HOME/f1tenth_sim/reference/levine/levine_centerline_safe.csv
```

Visualize `/reference_path`, `/map`, `/scan` and TF in RViz/Foxglove. The path publisher is **visualization only**; it does not drive the car.

### Terminal 4 — run ONE baseline controller

```bash
source /opt/ros/humble/setup.bash
source ~/f1tenth_sim/install/setup.bash
ros2 run f1tenth_control pure_pursuit --ros-args \
  -p waypoints_file:=$HOME/f1tenth_sim/reference/levine/levine_centerline_safe.csv \
  -p results_file:=/tmp/f1tenth_pure_pursuit_baseline.csv \
  -p target_speed:=1.0
```

**Do not run** `pure_pursuit` and `racing_pure_pursuit` at the same time: both publish `/drive`. The simulator may retain a previous command; verify that a zero-speed command has been applied before ending manual tests.

### Run profile-based racing controller

Stop the previous controller, then:

```bash
ros2 run f1tenth_control racing_pure_pursuit --ros-args \
  -p waypoints_file:=$HOME/f1tenth_sim/reference/levine/levine_centerline_safe.csv \
  -p speed_profile_file:=$HOME/f1tenth_sim/reference/levine/levine_speed_profile.csv \
  -p results_file:=/tmp/f1tenth_racing_metrics.csv \
  -p use_speed_profile:=true
```

The current racing controller defaults to `lookahead_distance=1.0 m`, `wheelbase=0.3302 m`, `max_steering_angle=0.4189 rad`, and `target_speed=5.0 m/s`. **These speeds and gains are simulator baselines, not approved real-car values.**

### Quick troubleshooting

| Symptom | What to inspect |
|---|---|
| `Package not found` | Source ROS and the local `install/setup.bash`; check `colcon list` |
| Simulator bridge import error | Upstream `f1tenth_gym` Python dependencies and interpreter/venv |
| No `/scan` or `/map` | Bridge running, exact launch args, ROS domain, startup logs |
| No `/drive` output | Controller path file exists and odometry is arriving |
| No visible path in RViz | Fixed frame, map origin, path frame, late-subscriber QoS |
| Unexpected collision | Initial pose, track dimensions, controller speed, path alignment |
| Duplicate vehicle transforms | Only one node should own the same TF edge |

## Understand the existing code

### `path_publisher.py`

Reads a semicolon-separated waypoint file, creates a `nav_msgs/Path`, and publishes it in the `map` frame as `/reference_path` (transient-local QoS). This is useful for seeing whether a proposed trajectory is aligned with the map.

### `pure_pursuit.py`

Subscribes to `/ego_racecar/odom`, loads the reference CSV, finds the nearest waypoint, selects a lookahead target approximately `1.0 m` farther along the closed loop, and computes steering using bicycle geometry. It publishes `/drive` at 20 Hz and logs lap time and cross-track error. It assumes the input pose and waypoint coordinates agree.

### `racing_pure_pursuit.py`

Adds an optional speed profile indexed by the nearest waypoint. It logs minimum/maximum/mean **commanded** speed, lap time, and tracking errors. Commanded speed is **not** measured wheel speed. The simulator collision input is not an adequate real-world safety system; add a watchdog and independent stop supervisor before hardware use.

### `generate_speed_profile.py`

Calculates discrete curvature from triples of centerline points, limits speed by lateral acceleration, then applies repeated backward braking and forward acceleration passes. It currently contains an absolute path under `/home/zaids_laptop`; changing paths or refactoring to configurable CLI arguments is required on other computers.

Existing speed-profile constants are `MAX_SPEED=5.0 m/s`, `MIN_SPEED=2.0 m/s`, lateral acceleration `7.0 m/s²`, forward acceleration `4.0 m/s²`, braking `6.0 m/s²`. These are **not** appropriate defaults for physical commissioning.

### Waypoint CSV format

The existing controller requires **semicolon** columns:

```text
# s_m;x_m;y_m;psi_rad;kappa_radpm;vx_mps;ax_mps2
0.000;0.000;0.000;0.000;0.000;0.500;0.000
```

Comment lines start with `#`. The separate speed-profile CSV uses a header named `planned_speed_mps`, and its row count must match the waypoint count. These two CSV formats are not interchangeable.

## Using a different track

The current simulator uses Levine by default. To test a new closed loop, create a map image plus `.yaml` metadata using **real-world meters**, not arbitrary pixels; store assets in the appropriate upstream simulator maps folder or configure an absolute map path. In the checked-out simulator configuration, inspect `src/f1tenth_gym_ros/config/sim.yaml` for map selection and starting pose names (commonly `map_path`, `map_img_ext`, `sx`, `sy`, `stheta`). The exact upstream file location can differ with installation.

**Track design requirements**:
- A closed, continuous drivable corridor; no unavoidable intersections for first experiments.
- Declare centerline length (meters), passage width, radius of tightest turn, origin, map resolution, and start pose.
- Width must leave room for **full car width + margin + tracking error**. A `0.9 m` passage is a difficult target, not a guaranteed-safe width.
- In unknown-track tests, **hide** the simulator's perfect occupancy map and supplied centerline from the exploration/localization/planning nodes. The simulator can still internally use the true map for collision physics and evaluation.
- Begin with an oval, then a curved loop, then a narrow loop; compare on at least two geometrically different tracks.

Before running a controller on a generated map, verify path and simulated vehicle share the same map/world origin. `map_path` alone does not validate this.

## Week sprint assignments

**Schedule: Mon October 26 – Sun November 1, 2026.** Deliver *code, a running demonstration, metrics and a GitHub PR*. Research-only progress does not count. Work must not depend on gaining access to the assembled real car.

### Moatasem — Sensor fusion in simulation

**Owner:** `src/f1tenth_localization/` (new ROS 2 package, suggested name `f1tenth_localization`)  
**Branch:** `feat/fusion-sim`  
**Goal:** Produce noisy simulated wheel-odometry and IMU messages, combine them in an EKF, and compare the result with simulator ground truth.

**Implementation contract**

1. Create a valid `ament_python` package with `package.xml`, `setup.py`, launch file, parameters YAML, README, and tests.
2. Implement **isolated** `sim_sensor_adapter.py`, the only node in this package allowed to subscribe to simulator truth `/ego_racecar/odom`. It should publish:
   - `/sim/wheel_odom_noisy` (`nav_msgs/Odometry`). Start from **velocity + yaw-rate** measurements with configurable noise/scale drift; integrate an independent wheel odometry trajectory rather than copying perfect x/y pose.
   - `/sim/imu_noisy` (`sensor_msgs/Imu`). Generate gyro-rate readings with Gaussian noise and optional bias; define valid units/covariances and set unavailable IMU orientation accordingly. Do not invent accelerometer measurements you cannot justify.
3. Parameterize rates, noise, drift, and random seed so tests are reproducible (suggest wheel 50 Hz, IMU 100 Hz). Preserve ROS time stamps and document the world/vehicle frame conventions.
4. Integrate `robot_localization` EKF (`ros-humble-robot-localization` recommended) to publish `/odometry/filtered`. Avoid feeding correlated ground-truth copies as independent measurements. Define who publishes TF to avoid conflicting `odom → base_link` publishers.
5. Build an evaluator (`tools/evaluate_fusion.py` inside package or an executable) which compares **time-aligned** wheel-only and fused paths against truth. Report position RMSE, yaw error, and loop-end drift. Write a CSV and visualization.
6. Test at least **three random seeds** and at least one **sensor bias/error injection** scenario. Report any case where fusion performs worse, with an explanation.
7. Document why fused odometry is **not** map-based localization; odometry may drift even with a successful EKF.

**Daily output**

| Date | Commit-worthy deliverable |
|---|---|
| Mon 26 | Simulator running; topic/TF inventory; package skeleton + plan |
| Tue 27 | Noisy wheel-odometry publisher; deterministic seed test |
| Wed 28 | Noisy IMU publisher; units/covariance and turning tests |
| Thu 29 | EKF launch/config and live `/odometry/filtered` |
| Fri 30 | Time-aligned evaluator + first plots |
| Sat 31 | Three-seed tests, deliberate sensor-bias run, results table |
| Sun 1 | Reproducible README and PR with demo evidence |

**Definition of done:** Clean build; one-command launch; topic rate evidence; three-seed metric CSV with plots; comparison to wheel-only odometry; failure-mode discussion; unit/frame tests; open PR. Stretch goal: connect filtered odometry into an experimental localization pipeline without duplicate TF publishers.

### Ahmad — Centerline planning from an occupancy map

**Owner:** `src/f1tenth_planning/` (new ROS 2 package, suggested name `f1tenth_planning`)  
**Branch:** `feat/map-planner`  
**Goal:** Given a single-loop track as an `OccupancyGrid`/saved map, automatically generate a smooth, collision-checked closed-loop path and a conservative speed profile.

**Implementation contract**

1. Create a valid ROS 2 package with launch/config, tests and instructions.
2. Read `nav_msgs/OccupancyGrid` or the corresponding map YAML + image. Respect **resolution, origin, axis orientation, unknown (-1), free and occupied values**. Write a map-to-world coordinate unit test.
3. Inflate obstacles by `vehicle_half_width + safety_margin` (both configurable); treat unknown as blocked for the initial implementation.
4. Extract an *ordered closed loop* using a documented method (medial axis/skeleton with graph ordering is a good candidate). Explicitly detect no loop, ambiguous branching and collision-clearance failure: fail with diagnostics instead of returning an invalid path.
5. Resample at roughly constant arc length (~`0.05–0.10 m` starting suggestion). Smooth only with post-smoothing collision checks. Calculate yaw, curvature, closure gap and minimum obstacle clearance.
6. Publish `/planning/centerline` as a `nav_msgs/Path` in the `map` frame and export the same seven-column, semicolon-separated CSV expected by the current Pure Pursuit loader. Do **not** read `levine_centerline_safe.csv` to produce the path; that CSV is allowed only as a comparison reference after generation.
7. Generate a conservative, configurable speed profile using lateral-acceleration limits and forward/backward acceleration and braking passes. No machine-specific hard-coded paths or assumption that 2–5 m/s is appropriate on our hardware.
8. Add path evaluation output: track length, minimum clearance, waypoint spacing statistics, maximum curvature, continuity/closure gap, obstacle violation count and plot overlay.
9. Run on **two geometrically different closed-loop maps** (Levine if its image is installed plus a generated oval, or two generated tracks), plus one *invalid/blocked* test.

**Daily output**

| Date | Commit-worthy deliverable |
|---|---|
| Mon 26 | Inspect map structure; coordinate transforms and tests |
| Tue 27 | Map parsing, inflation, visualization |
| Wed 28 | Candidate centerline extraction and loop detection |
| Thu 29 | Ordered loop + smoothing + collision checks |
| Fri 30 | Path topic, CSV export, conservative speed profile |
| Sat 31 | Two-map experiment, negative case, metric table and plots |
| Sun 1 | Clean build, reproducible launch and PR |

**Definition of done:** Clean build; a command that turns a map into a path; publish + CSV export; two map overlays; quantified minimum clearance and closure quality; negative test; PR. Stretch goal: load the generated CSV into the existing racing controller after verifying world coordinates and simulator map match.

### Shared Saturday integration review

Each teammate demonstrates the package **live** in 10–15 minutes. Compare topics and frame IDs. Moatasem's `/odometry/filtered` is odom-relative; Ahmad's `/planning/centerline` is map-relative, so they are **not plug-and-play** without global localization / `map→odom`. Neither contributor should publish driving commands to `/drive` during independent tests.

### Evidence and grading rubric

| Criterion | Weight |
|---|---:|
| Runnable ROS 2 implementation and committed code | 40% |
| Repeatable tests and quantitative evidence | 30% |
| Clear ROS interfaces, parameterization and documentation | 20% |
| GitHub hygiene, collaboration and demonstration | 10% |

A report without runnable code is **not done**. A working baseline that clearly documents limitations is more valuable than an ambitious, incomplete algorithm.

## Contribution rules

1. Branch from the latest `main`; never commit directly to `main` for the sprint.
2. Use `feat/fusion-sim` (Moatasem) or `feat/map-planner` (Ahmad), or an equivalent unique branch name.
3. Make at least one **meaningful pushed commit daily** with a short demonstration/status note.
4. Keep modifications inside your owned package unless coordinated. Do not rewrite working baseline control code.
5. Do not commit generated `build/`, `install/`, `log/`, Python virtual environments, huge rosbags or environment-specific secrets.
6. PR description must include: goal, changed files, exact commands, topic/frame table, sample logs, metric table/plot, tests, known limitations and next steps.
7. The team lead should reproduce the demo from a fresh checkout before merging.

Suggested PR template:

```markdown
### Goal
### Files and ROS interfaces
### Install/build/launch commands
### Tests and quantitative results
### Screenshots or plots
### Known limitations
### Follow-up work
- [ ] Clean build
- [ ] Verified frame IDs and units
- [ ] No simulator-ground-truth shortcut
- [ ] Existing baseline controller unchanged
```

## Known gaps and roadmap

1. **This sprint:** Moatasem's simulated fusion; Ahmad's map-derived planner.
2. **Next:** safety supervisor/command watchdog; realistic simulated sensor timing, LiDAR geometry and vehicle parameters.
3. **Unknown-track first lap:** reactive corridor/Follow-the-Gap exploration, with live SLAM and independent collision avoidance. The planner sprint does **not** solve exploration.
4. **Loop handling:** loop-closure verification, map quality assessment and map-version management.
5. **Map-based pose:** localization and reliable transformations between `map`, `odom`, and the vehicle frame.
6. **First autonomous racing laps:** adapt current Pure Pursuit to a validated map-relative pose + generated path, at low speed.
7. **Progressive racing:** after each qualified successful lap, increase allowable speed cautiously while limiting curvature, stopping distance, clearances, localization uncertainty, and observed tracking error.
8. **Sim-to-real:** validate vehicle dimensions, VESC calibration, wheel/IMU measurement characteristics and safety before motion on the assembled vehicle.

**Principle:** Every feature must have a repeatable simulation test, measured result, documented ROS interface, and clear distinction between simulated ground truth and physically measurable information.
