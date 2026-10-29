#!/usr/bin/env zsh
emulate -L zsh
cd "${F1TENTH_SIM_WS:-$HOME/f1tenth_sim}" || exit 1
source scripts/sim_env.zsh || exit 1
python3 -u tools/mission/mission_runner.py
