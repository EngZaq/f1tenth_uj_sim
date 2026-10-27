#!/usr/bin/env bash
set -euo pipefail
cd "$HOME/f1tenth_sim"
if [[ ! -d .git ]]; then echo 'Not a git repository: ~/f1tenth_sim' >&2; exit 1; fi
printf '\n== Branch and remote ==\n'
git branch --show-current
git remote -v
printf '\n== Current changes ==\n'
git status --short
printf '\n== Working tree diff summary ==\n'
git diff --stat
printf '\n== Tracked backup files (review) ==\n'
git ls-files '*bak' '*tmp' || true
printf '\n== Local scratch candidates (review only; nothing deleted) ==\n'
find tools config results -type f \( -name '*.pyc' -o -name '*.bak' -o -name '*.tmp' \) -print 2>/dev/null || true
printf '\n== Local map artifacts ==\n'
ls -lh results/mapping/uj_technical_slam_01.{yaml,pgm,posegraph,data} 2>/dev/null || true
printf '\n== Referenced generated paths ==\n'
ls -lh reference/generated/uj_technical_slam_01_{centerline,registered,speed_profile}.csv 2>/dev/null || true
printf '\nNothing was deleted, staged, committed or pushed.\n'
printf 'Copy README.md + docs/ into the repo, then review git status and run the commit commands in the answer.\n'
