# Source this script from zsh: source scripts/sim_env.zsh
# Keep ROS startup in one place; do not edit generated entry points.
if [[ -z "${ZSH_VERSION:-}" ]]; then
  echo 'ERROR: sim_env.zsh must be sourced from zsh.' >&2
  return 1 2>/dev/null || exit 1
fi
SIM_WS="${F1TENTH_SIM_WS:-$HOME/f1tenth_sim}"
if [[ ! -f "$SIM_WS/.venv/bin/activate" || ! -f "$SIM_WS/install/setup.zsh" ]]; then
  echo "ERROR: Missing .venv or install/setup.zsh in $SIM_WS" >&2
  return 1
fi
source "$SIM_WS/.venv/bin/activate"
source /opt/ros/humble/setup.zsh
source "$SIM_WS/install/setup.zsh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
# ROS executable may have #!/usr/bin/python3 even when .venv is activated.
# Extend import path for that interpreter, without editing generated scripts.
VENV_SITE="$($SIM_WS/.venv/bin/python3 -c 'import site; print(next(p for p in site.getsitepackages() if p.endswith("site-packages")))')" || return 1
GYM_SOURCE="$SIM_WS/src/f1tenth_gym_ros/f1tenth_gym"
if [[ ! -d "$GYM_SOURCE/f1tenth_gym" ]]; then
  echo "ERROR: Gym source missing at $GYM_SOURCE" >&2
  return 1
fi
export PYTHONPATH="$VENV_SITE:$GYM_SOURCE:${PYTHONPATH:-}"
export F1TENTH_SIM_WS="$SIM_WS"
echo "F1TENTH environment: $SIM_WS (domain=$ROS_DOMAIN_ID; RMW=$RMW_IMPLEMENTATION)"
