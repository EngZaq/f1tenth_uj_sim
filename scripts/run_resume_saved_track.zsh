#!/usr/bin/env zsh
emulate -L zsh
setopt ERR_EXIT
cd "${F1TENTH_SIM_WS:-$HOME/f1tenth_sim}"
source scripts/sim_env.zsh
python3 -u tools/mission/resume_saved_map_mission.py
