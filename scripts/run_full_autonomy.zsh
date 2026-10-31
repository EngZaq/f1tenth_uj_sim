#!/usr/bin/env zsh
emulate -L zsh
set -e
AUTONOMY_ROOT="${0:A:h:h}"
source "$AUTONOMY_ROOT/scripts/sim_env.zsh"
cd "$AUTONOMY_ROOT"
exec python3 -u tools/mission/full_autonomy.py --autonomous --start-simulator "$@"
