#!/usr/bin/env bash
# Reproduce the simulator dependency setup in an isolated CI workspace.
set -euo pipefail
REPO_ROOT="$(pwd)"
CI_SIM_WS="$REPO_ROOT/.ci_sim_ws"
export CI_SIM_WS
mkdir -p "$CI_SIM_WS/src/f1tenth_gym_ros"
BRIDGE_SOURCE="$CI_SIM_WS/src/f1tenth_gym_ros"
git init "$BRIDGE_SOURCE"
git -C "$BRIDGE_SOURCE" remote add origin https://github.com/f1tenth/f1tenth_gym_ros.git
git -C "$BRIDGE_SOURCE" fetch --depth 1 origin 08395766c4d9dc5a763381f1dd6fa4a3d68df66e
git -C "$BRIDGE_SOURCE" checkout --detach FETCH_HEAD
git -C "$BRIDGE_SOURCE" apply "$REPO_ROOT/patches/simulator/uj_simulator_changes.patch"
GYM_SOURCE="$BRIDGE_SOURCE/f1tenth_gym"
git init "$GYM_SOURCE"
git -C "$GYM_SOURCE" remote add origin https://github.com/f1tenth/f1tenth_gym.git
git -C "$GYM_SOURCE" fetch --depth 1 origin bdaec1420c3b0f103858d289866d0d4e2e597c30
git -C "$GYM_SOURCE" checkout --detach FETCH_HEAD
python3 -m venv --system-site-packages "$CI_SIM_WS/.venv"
source "$CI_SIM_WS/.venv/bin/activate"
python3 -m pip install 'numpy<2' 'opencv-python<4.12' 'matplotlib<4' 'setuptools<80' 'scikit-image<0.26'
python3 -m pip install "$GYM_SOURCE"
# Fail dependency setup before spending a mapping lap if planning cannot import.
python3 "$REPO_ROOT/tools/planning/extract_robust_centerline.py" --help >/dev/null
python3 "$REPO_ROOT/tools/planning/prepare_vehicle_route.py" --help >/dev/null
python3 - <<'PY'
import os
from pathlib import Path
import yaml
root=Path.cwd()
config=Path(os.environ['CI_SIM_WS'])/'src/f1tenth_gym_ros/config/sim.yaml'
data=yaml.safe_load(config.read_text())
data['bridge']['ros__parameters']['map_path']=str(root/'maps/uj_technical_loop_v1')
data['bridge']['ros__parameters']['kb_teleop']=False
data['foxglove']['ros__parameters']['open_foxglove']=False
config.write_text(yaml.safe_dump(data,sort_keys=False))
PY
# ROS setup scripts can read unset variables, so source without nounset.
set +u
source /opt/ros/humble/setup.bash
cd "$CI_SIM_WS"
colcon build --symlink-install --packages-select f1tenth_gym_ros
