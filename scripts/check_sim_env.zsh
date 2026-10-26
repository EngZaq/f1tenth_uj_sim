#!/usr/bin/env zsh
set -eu
HERE="${0:A:h}"
source "$HERE/sim_env.zsh"
BRIDGE="$F1TENTH_SIM_WS/install/f1tenth_gym_ros/lib/f1tenth_gym_ros/gym_bridge"
echo "Launcher first line: $(head -n 1 "$BRIDGE")"
if [[ $(head -c 2 "$BRIDGE") != '#!' ]]; then
  echo 'ERROR: invalid bridge launcher header. Rebuild/reinstall the ROS bridge; do not modify this generated file.' >&2
  exit 1
fi
/usr/bin/python3 -c 'import f1tenth_gym; print("system Python finds Gym:", f1tenth_gym.__file__)'
/usr/bin/python3 -c 'import rclpy; print("system Python finds ROS rclpy")'
$F1TENTH_SIM_WS/.venv/bin/python3 -c 'import f1tenth_gym; print("venv finds Gym:", f1tenth_gym.__file__)'
ros2 pkg prefix f1tenth_gym_ros
echo 'Environment checks passed. Next: launch simulator in this same environment.'
