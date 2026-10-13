# UJ F1TENTH Simulation Workspace

ROS 2 Humble simulation workspace for the University of Jordan F1TENTH project.

## Current status

- Ubuntu 22.04
- ROS 2 Humble
- F1TENTH Gym installed in Python virtual environment
- f1tenth_gym_ros dev-humble built successfully
- Levine map available
- Simulator launches successfully
- Verified ROS topics:
  - /scan
  - /ego_racecar/odom
  - /drive
  - /map
  - /tf
  - /tf_static

## Setup

```zsh
source .venv/bin/activate
source /opt/ros/humble/setup.zsh
source install/setup.zsh
