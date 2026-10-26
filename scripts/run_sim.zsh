#!/usr/bin/env zsh

set -e

HERE="${0:A:h}"

source "$HERE/sim_env.zsh"

if ! ros2 pkg prefix f1tenth_gym_ros >/dev/null 2>&1; then
    echo "ERROR: f1tenth_gym_ros is not available."
    echo "Check your workspace setup."
    exit 1
fi

exec ros2 launch f1tenth_gym_ros gym_bridge_launch.py "$@"
