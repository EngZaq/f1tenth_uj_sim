# UJ F1TENTH Simulation Workspace

ROS 2 Humble simulation and autonomy-development workspace for the University
of Jordan F1TENTH graduation project. The workspace is used to develop,
exercise, and measure autonomous-driving software before deploying it to the
physical F1TENTH vehicle.

The repository contains the project-owned ROS 2 packages, controller code,
Levine reference paths, speed-profile tooling, and recorded evaluation
results. The upstream F1TENTH Gym source is kept outside this repository and
is installed as a workspace dependency.

## Contents

- [System overview](#system-overview)
- [Requirements](#requirements)
- [Installation and build](#installation-and-build)
- [Running the simulator](#running-the-simulator)
- [ROS interfaces](#ros-interfaces)
- [Controllers](#controllers)
- [Levine reference data](#levine-reference-data)
- [Speed-profile generation](#speed-profile-generation)
- [Evaluation results](#evaluation-results)
- [Visualization and debugging](#visualization-and-debugging)
- [Safety behavior](#safety-behavior)
- [Testing](#testing)
- [Repository layout](#repository-layout)
- [Development roadmap](#development-roadmap)

## System overview

```text
                           ROS 2
                             |
                     f1tenth_gym_ros
                       simulator bridge
                             |
          +------------------+------------------+
          |                  |                  |
        /drive             /scan           /ego_racecar/odom
          |                  |                  |
          v                  v                  v
     simulated car      LiDAR data       vehicle state
          ^                                     |
          |                                     |
          +--------- autonomy nodes ------------+
                    |       |       |
             path_publisher |   controllers
                             |
                  pure_pursuit / racing_pure_pursuit
```

The simulator and physical vehicle expose the same high-level control concept:
an Ackermann command is published to `/drive`. This makes it possible to tune
path following, speed control, localization, LiDAR processing, collision
handling, and evaluation workflows without continuously using the real car.

## Requirements

The tested development environment is:

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- `zsh`
- A Python virtual environment at `~/f1tenth_sim/.venv`
- The `f1tenth_gym_ros` and `f1tenth_gym` dependencies on their
  `dev-humble` branches

The simulator dependency is described in
[`simulation_dependencies.repos`](simulation_dependencies.repos). The
upstream simulator source is intentionally not committed here.

## Installation and build

Clone the repository and enter the workspace:

```bash
git clone git@github.com:EngZaq/f1tenth_uj_sim.git ~/f1tenth_sim
cd ~/f1tenth_sim
```

Create or activate the Python environment used by the simulator:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
```

Install the upstream repositories described by the repos file using the
project's normal ROS 2 dependency workflow, then install any simulator Python
dependencies required by `f1tenth_gym_ros`.

Build the workspace:

```bash
source /opt/ros/humble/setup.zsh
colcon build --symlink-install
source install/setup.zsh
```

For every new terminal, source the environment before using ROS commands:

```bash
cd ~/f1tenth_sim
source .venv/bin/activate
source /opt/ros/humble/setup.zsh
source install/setup.zsh
```

## Running the simulator

Launch the bridge with Foxglove disabled:

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py \
  open_foxglove:=false
```

The default launch uses the Levine map. The bridge node is `/bridge`, and the
simulator publishes the vehicle state, LiDAR, TF, map, and collision topics
listed below.

Useful basic checks from another sourced terminal:

```bash
ros2 node list
ros2 topic list
ros2 topic echo --once /ego_racecar/odom
ros2 topic echo --once /scan
ros2 topic echo --once /ego_racecar/collision
ros2 topic echo --once /map
ros2 run tf2_tools view_frames
```

Manual Ackermann commands can be sent directly to the simulator:

```bash
ros2 topic pub -r 10 /drive \
  ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {speed: 1.0, steering_angle: 0.0}}"
```

Stop the vehicle explicitly when ending a manual test:

```bash
ros2 topic pub --once /drive \
  ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {speed: 0.0, steering_angle: 0.0}}"
```

## ROS interfaces

| Topic | Type | Purpose |
|---|---|---|
| `/drive` | `ackermann_msgs/msg/AckermannDriveStamped` | Ackermann speed and steering command |
| `/ego_racecar/odom` | `nav_msgs/msg/Odometry` | Simulated vehicle pose and velocity |
| `/scan` | `sensor_msgs/msg/LaserScan` | Simulated LiDAR scan |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Levine occupancy map |
| `/ego_racecar/collision` | `std_msgs/msg/Bool` | Collision state |
| `/reference_path` | `nav_msgs/msg/Path` | Published map-frame reference path |
| `/tf` | `tf2_msgs/msg/TFMessage` | Dynamic transforms |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Static transforms |
| `/joint_states` | `sensor_msgs/msg/JointState` | Vehicle joint states |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Simulator teleoperation interface |

The primary frame relationships are:

```text
map
└── ego_racecar/base_link
    ├── ego_racecar/laser
    └── vehicle links and wheels
```

The initial ego pose is approximately `x = -12.0 m`, `y = 0.0 m`,
`yaw = 0 rad`.

## Controllers

The `f1tenth_control` package provides three executables:

| Executable | Purpose | Default input |
|---|---|---|
| `path_publisher` | Publishes the Levine path once on `/reference_path` | `reference/levine/levine_centerline_safe.csv` |
| `pure_pursuit` | Constant-speed Pure Pursuit with lap and CTE metrics | Safe Levine centerline |
| `racing_pure_pursuit` | Pure Pursuit with optional curvature/braking-aware speed profile | Safe Levine centerline and speed profile |

### Publish the reference path

```bash
ros2 run f1tenth_control path_publisher
```

The path is published in the `map` frame with transient-local QoS, so a
subscriber that starts later can still receive it.

### Run constant-speed Pure Pursuit

```bash
ros2 run f1tenth_control pure_pursuit
```

Useful parameter overrides:

```bash
ros2 run f1tenth_control pure_pursuit --ros-args \
  -p target_speed:=1.5 \
  -p lookahead_distance:=1.0 \
  -p waypoints_file:=/absolute/path/to/levine_centerline_safe.csv \
  -p results_file:=/absolute/path/to/pure_pursuit_metrics.csv
```

The node subscribes to `/ego_racecar/odom` and
`/ego_racecar/collision`, publishes `/drive` at 20 Hz, clamps steering to
`0.4189 rad`, stops on collision, and sends several zero-speed commands during
shutdown.

### Run profile-based racing Pure Pursuit

```bash
ros2 run f1tenth_control racing_pure_pursuit
```

The default racing configuration is:

| Parameter | Default |
|---|---:|
| `waypoints_file` | `reference/levine/levine_centerline_safe.csv` |
| `speed_profile_file` | `reference/levine/levine_speed_profile.csv` |
| `results_file` | `results/racing_profile_metrics_v2.csv` |
| `use_speed_profile` | `true` |
| `target_speed` | `5.0 m/s` |
| `lookahead_distance` | `1.0 m` |
| `wheelbase` | `0.3302 m` |
| `max_steering_angle` | `0.4189 rad` |

To run at a constant target speed without the profile:

```bash
ros2 run f1tenth_control racing_pure_pursuit --ros-args \
  -p use_speed_profile:=false \
  -p target_speed:=3.0
```

The racing controller requires the speed profile and waypoint files to contain
the same number of points. It records commanded-speed statistics, lap time,
mean cross-track error, and maximum cross-track error after each completed
lap.

## Levine reference data

The current map is the Levine track. Its occupancy-grid metadata is:

```text
resolution = 0.05 m/cell
image      = levine_slam.pgm
origin     = [-25, -6.19, 0]
```

Reference files are stored in [`reference/levine`](reference/levine):

| File | Description |
|---|---|
| `levine_centerline_original.csv` | Original Levine centerline |
| `levine_centerline_safe.csv` | Safe semicolon-delimited centerline used by the controllers |
| `levine_candidate.csv` | Candidate path in `x,y,yaw,speed` format |
| `levine_raceline_original.csv` | Original raceline data |
| `levine_planning_map.yaml` | Planning-map metadata |
| `levine_speed_profile.csv` | Generated CSV speed profile |

The safe centerline has 619 waypoints and describes a closed loop of
approximately 62.56 m. Its controller format is:

```text
s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2
```

The first line is a comment/header, and controller loaders ignore blank lines
and lines beginning with `#`.

## Speed-profile generation

[`tools/generate_speed_profile.py`](tools/generate_speed_profile.py) derives a
speed profile from the safe centerline:

1. Estimate curvature from three neighboring waypoints.
2. Limit speed using the lateral-acceleration constraint.
3. Apply repeated backward braking passes.
4. Apply forward acceleration passes.
5. Write the resulting profile as CSV.

The configured limits are:

```text
maximum speed:             5.0 m/s
minimum speed:             2.0 m/s
maximum lateral accel.:    7.0 m/s²
maximum acceleration:      4.0 m/s²
maximum braking:           6.0 m/s²
```

Run it from the repository root:

```bash
python tools/generate_speed_profile.py
```

The script currently uses the workspace path
`~/f1tenth_sim/reference/levine/levine_centerline_safe.csv` and writes
`~/f1tenth_sim/reference/levine/levine_speed_profile.csv`. Change those
constants before using a different checkout location.

The checked-in profile contains 619 samples. Its planned speed ranges from
`2.625 m/s` to `5.000 m/s`, with a mean of approximately `4.770 m/s`.

## Evaluation results

Recorded CSV files are in [`results`](results):

| File | Meaning |
|---|---|
| `pure_pursuit_metrics.csv` | Constant-speed controller runs |
| `racing_profile_metrics.csv` | Earlier profile-controller runs |
| `racing_profile_metrics_v2.csv` | Current profile-controller runs with commanded-speed statistics |

The current result schema includes lap time and tracking error. The v2 racing
schema additionally records mean, minimum, and maximum commanded speed:

```text
timestamp,
lap,
configured_target_speed_mps,
mean_commanded_speed_mps,
min_commanded_speed_mps,
max_commanded_speed_mps,
lookahead_m,
lap_time_s,
mean_cte_m,
max_cte_m
```

Representative recorded results include:

- Constant-speed Pure Pursuit at `1.5 m/s`: approximately `41.3 s` per lap
  with mean CTE near `0.017 m`.
- Profile-based racing at a configured maximum of `5.0 m/s`: approximately
  `14.6–14.9 s` per lap, with mean CTE generally below `0.023 m`.

These are simulation measurements, not physical-vehicle guarantees. Results
depend on simulator state, starting conditions, controller parameters, and
collision behavior.

## Visualization and debugging

Foxglove Bridge is available at:

```text
ws://localhost:8765
```

Launch the simulator with Foxglove enabled when required by the local launch
configuration, then use `map` as the fixed frame. The most useful displays
are:

- Levine occupancy map
- `/reference_path`
- `/scan`
- vehicle TF
- `/ego_racecar/odom`
- `/ego_racecar/collision`

The RViz configuration is
[`src/f1tenth_gym_ros/config/rviz/gym_bridge.rviz`](src/f1tenth_gym_ros/config/rviz/gym_bridge.rviz).
The Foxglove layout is
[`src/f1tenth_gym_ros/config/foxglove/gym_bridge_foxglove.json`](src/f1tenth_gym_ros/config/foxglove/gym_bridge_foxglove.json).

## Safety behavior

The controllers:

- stop publishing motion commands when collision is reported;
- publish zero speed and zero steering on shutdown;
- clamp steering to the configured maximum;
- monitor collision state through `/ego_racecar/collision`;
- record tracking statistics only while the vehicle is not in collision.

The simulator can retain the last `/drive` command if a publisher disappears.
Always stop the vehicle explicitly after manual tests, and stop the simulator
when it is no longer needed.

## Testing

Build and run the package tests:

```bash
source /opt/ros/humble/setup.zsh
colcon test --packages-select f1tenth_control
colcon test-result --verbose
```

The package includes copyright, Flake8, and PEP 257 checks. A successful
controller run should also be checked through the ROS topics and the generated
metrics CSV.

## Repository layout

```text
.
├── reference/levine/       Levine maps, paths, and speed profile
├── results/                Recorded controller metrics
├── src/f1tenth_control/    Path publisher and Pure Pursuit nodes
├── src/f1tenth_gym_ros/    ROS 2 simulator bridge and configurations
├── tools/                  Offline profile-generation utilities
├── simulation_dependencies.repos
└── README.md
```

Generated `build/`, `install/`, and `log/` directories are ignored by Git.

## Development roadmap

Completed:

- [x] ROS 2 Humble simulator bringup
- [x] Levine map loading
- [x] `/drive`, odometry, LiDAR, TF, map, and collision verification
- [x] Foxglove/RViz visualization setup
- [x] Reference path publishing
- [x] Constant-speed Pure Pursuit
- [x] Lap-time and cross-track-error recording
- [x] Curvature- and acceleration-aware speed profile
- [x] Profile-based racing Pure Pursuit

Next:

- [ ] Add a command watchdog with stale-command timeout
- [ ] Add LiDAR emergency braking
- [ ] Add automated repeatable simulation experiments
- [ ] Compare simulation results with physical-vehicle runs
- [ ] Tune lookahead distance as a function of speed
- [ ] Expand collision and safety regression tests

## License and upstream attribution

This repository contains project-specific work for the University of Jordan
F1TENTH project. Upstream F1TENTH Gym and ROS bridge code retain their
respective licenses and attribution notices in their source trees.
