# UJ F1TENTH Simulation Workspace

ROS 2 Humble simulation and autonomy-development workspace for the University of Jordan F1TENTH graduation project.

The purpose of this workspace is to develop, test, and evaluate the high-level autonomous-driving software before transferring it to the physical F1TENTH car.

---

## Why We Use Simulation

The real vehicle and the simulator should expose similar high-level interfaces.

```text
                    AUTONOMY SOFTWARE
                           |
                    Ackermann command
                           |
                         /drive
                           |
              +------------+------------+
              |                         |
        F1TENTH Gym                 Real F1TENTH
              |                         |
      simulated vehicle          VESC + steering
              |
       +------+------+
       |             |
     /scan     /ego_racecar/odom
```

This allows us to develop algorithms without continuously needing the physical vehicle.

The simulator will be used for:

- Path following
- Pure Pursuit control
- Speed-profile development
- Localization experiments
- LiDAR-based algorithms
- Collision and safety testing
- Controller tuning
- Lap-time evaluation
- Cross-track-error evaluation
- Sim-to-real preparation

---

# Current Environment

Development machine:

- Ubuntu 22.04
- ROS 2 Humble
- zsh
- Python virtual environment

Workspace:

```text
~/f1tenth_sim
```

Python environment:

```text
~/f1tenth_sim/.venv
```

Simulator:

- `f1tenth_gym_ros`
- branch: `dev-humble`
- `f1tenth_gym`
- branch: `dev-humble`

The upstream simulator source is intentionally not committed to this repository.

This repository is intended to contain our own:

- controllers
- planning code
- configuration
- launch files
- evaluation tools
- documentation

---

# Current Simulator Architecture

```text
                        ROS 2
                          |
                 +--------+--------+
                 |                 |
              /drive            /cmd_vel
                 |
                 v
          +--------------+
          |  Gym Bridge  |
          +--------------+
             |   |   |
             |   |   +----------------+
             |   |                    |
             v   v                    v
           /odom /scan            collision
             |    |                   |
             +----+---------+---------+
                            |
                            v
                       autonomy
                       software
```

Main simulator node:

```text
/bridge
```

Other active nodes include:

```text
/ego_robot_state_publisher
/map_server
/lifecycle_manager_localization
/foxglove_bridge
```

---

# Verified ROS Interfaces

The following interfaces have been verified working.

| Topic | Message purpose |
|---|---|
| `/drive` | Ackermann steering and speed commands |
| `/ego_racecar/odom` | Simulated vehicle pose and velocity |
| `/scan` | Simulated LiDAR |
| `/map` | Levine occupancy map |
| `/ego_racecar/collision` | Collision state |
| `/tf` | Dynamic coordinate transforms |
| `/tf_static` | Static coordinate transforms |
| `/joint_states` | Vehicle joint states |
| `/cmd_vel` | Teleoperation interface |

The `/drive` topic uses:

```text
ackermann_msgs/msg/AckermannDriveStamped
```

The `/scan` topic uses:

```text
sensor_msgs/msg/LaserScan
```

The odometry topic uses:

```text
nav_msgs/msg/Odometry
```

---

# Levine Map

The simulator currently uses the Levine track.

Verified map information:

```text
resolution = 0.05 m/cell
width      = 2048 cells
height     = 2048 cells

origin:
x = -51.225 m
y = -51.225 m
```

This means one occupancy-grid cell represents approximately:

```text
5 cm × 5 cm
```

The initial ego vehicle pose is approximately:

```text
x = -12.0 m
y = 0.0 m
yaw = 0 rad
```

---

# Virtual LiDAR

The simulated LiDAR is publishing successfully on:

```text
/scan
```

The scan contains one distance measurement for each laser beam.

Conceptually:

```text
beam angle + measured range
             |
             v
        obstacle point
```

ROS combines the LaserScan data with TF to place the measurements correctly in the map coordinate frame.

The relationship is approximately:

```text
map
 |
 +-- ego_racecar/base_link
          |
          +-- ego_racecar/laser
```

---

# TF Coordinate Frames

TF has been verified working.

The main relationship is:

```text
map
 |
 +-- ego_racecar/base_link
          |
          +-- wheels
          |
          +-- laser
          |
          +-- vehicle links
```

This allows sensor measurements expressed in the LiDAR frame to be transformed into the vehicle or map frame.

This will later be essential for:

- localization
- path following
- LiDAR processing
- real-vehicle integration

---

# Manual Vehicle Tests Completed

## Straight Driving

A command was successfully sent using:

```zsh
ros2 topic pub -r 10 \
/drive ackermann_msgs/msg/AckermannDriveStamped \
"{drive: {speed: 1.0, steering_angle: 0.0}}"
```

The simulated vehicle moved in the positive X direction.

Odometry changed correctly while the heading remained approximately constant.

---

## Steering Test

Steering commands were also successfully tested.

Example:

```zsh
ros2 topic pub -r 10 \
/drive ackermann_msgs/msg/AckermannDriveStamped \
"{drive: {speed: 0.3, steering_angle: 0.05}}"
```

The vehicle changed:

```text
X position
Y position
Yaw
```

confirming that Ackermann steering and simulated vehicle dynamics are working.

---

# Collision Testing

Collision detection was verified using:

```text
/ego_racecar/collision
```

Normal driving produces:

```text
data: false
```

A useful simulator behavior was discovered during testing:

when the vehicle collides with a wall, the current simulator implementation resets several dynamic states.

Therefore collision status should always be checked when analyzing strange post-collision odometry.

---

# Foxglove Visualization

Foxglove Bridge is installed and working.

WebSocket:

```text
ws://localhost:8765
```

The following have been visualized successfully:

- Levine occupancy map
- vehicle TF tree
- vehicle frames
- LiDAR scan
- map coordinate system

Recommended Foxglove fixed frame:

```text
map
```

For debugging, the most useful frames are:

```text
map
ego_racecar/base_link
ego_racecar/laser
```

Displaying every wheel and hinge TF at the same time can make the visualization unnecessarily cluttered.

---

# Starting the Simulator

Terminal 1:

```zsh
cd ~/f1tenth_sim

source ~/f1tenth_sim/.venv/bin/activate
source /opt/ros/humble/setup.zsh
source ~/f1tenth_sim/install/setup.zsh

ros2 launch f1tenth_gym_ros gym_bridge_launch.py \
  open_foxglove:=false
```

The virtual environment must be activated before launching.

The generated ROS Python executable uses:

```text
#!/usr/bin/env python3
```

so the active Python environment determines which Python interpreter runs the Gym bridge.

---

# Development Roadmap

## Phase 1 — Simulator Bringup

Completed:

- [x] Install F1TENTH Gym
- [x] Install `f1tenth_gym_ros`
- [x] Configure Python virtual environment
- [x] Build ROS 2 workspace
- [x] Launch simulator
- [x] Load Levine map
- [x] Verify `/drive`
- [x] Verify `/ego_racecar/odom`
- [x] Verify `/scan`
- [x] Verify `/map`
- [x] Verify TF
- [x] Verify collision detection
- [x] Visualize simulation using Foxglove
- [x] Perform straight-drive test
- [x] Perform steering test

## Phase 2 — Reference Path

Next:

- [ ] Inspect Levine centerline/raceline files
- [ ] Understand waypoint format
- [ ] Load waypoints into ROS
- [ ] Visualize the reference path in Foxglove
- [ ] Determine nearest waypoint
- [ ] Determine lookahead point

The desired structure will be:

```text
Levine map
     |
centerline / raceline
     |
     v
waypoints
     |
     v
Pure Pursuit
```

## Phase 3 — Pure Pursuit Controller

We will implement our own ROS 2 controller.

Inputs:

```text
/ego_racecar/odom
reference path
```

Controller:

```text
current vehicle pose
        |
find nearest path point
        |
select lookahead point
        |
transform target into vehicle frame
        |
calculate curvature
        |
calculate steering angle
```

Output:

```text
/drive
```

First goal:

> Complete one autonomous lap of Levine at a low constant speed.

---

## Phase 4 — Speed Control

After reliable path following:

- [ ] Increase speed gradually
- [ ] Create curvature-dependent speed profile
- [ ] Reduce speed before sharp corners
- [ ] Increase speed on straights
- [ ] Tune lookahead distance as a function of speed

Possible relationship:

```text
low curvature  -> high speed
high curvature -> low speed
```

---

## Phase 5 — Safety

Add safety mechanisms such as:

- [ ] command watchdog
- [ ] automatic stop on stale commands
- [ ] collision monitoring
- [ ] LiDAR emergency braking
- [ ] steering saturation
- [ ] speed limits

A command watchdog is important because the simulator demonstrated that a vehicle may continue executing the last `/drive` command when the publisher disappears.

---

## Phase 6 — Performance Evaluation

We will measure:

- lap time
- average speed
- maximum speed
- cross-track error
- maximum path error
- steering behavior
- collision count
- controller stability

These measurements will allow controller parameters and algorithms to be compared objectively.

---

## Phase 7 — Localization Experiments

Initially the simulator provides ground-truth odometry.

Later we will replace this assumption with localization closer to the real vehicle:

```text
LiDAR
  +
known map
  +
localization algorithm
  |
  v
estimated pose
  |
  v
Pure Pursuit
```

This follows the project's real-car strategy:

> Build a map first, then localize and race on the known map.

---

## Phase 8 — Sim-to-Real Transfer

The high-level controller should remain independent of the hardware.

Simulation:

```text
Pure Pursuit
     |
   /drive
     |
F1TENTH Gym
```

Real vehicle:

```text
Pure Pursuit
     |
   /drive
     |
VESC / steering interface
     |
motor + servo
```

The objective is to reuse as much high-level autonomy software as possible between simulation and the physical F1TENTH car.

---

# Immediate Next Goal

The next development milestone is:

> Load and visualize the Levine reference path, then build the first Pure Pursuit controller.

Target pipeline:

```text
Levine reference path
          +
/ego_racecar/odom
          |
          v
    Pure Pursuit
          |
          v
       /drive
          |
          v
   F1TENTH Gym
          |
          v
Autonomous Levine lap
```

---

# Project Goal

The simulation milestone is complete when the vehicle can:

1. start from its initial position,
2. determine its position on the known track,
3. follow the reference path,
4. complete a full Levine lap without collision,
5. maintain a controlled speed profile,
6. report quantitative performance metrics.

After this pipeline is reliable, the same architecture will be adapted to the physical University of Jordan F1TENTH vehicle.
