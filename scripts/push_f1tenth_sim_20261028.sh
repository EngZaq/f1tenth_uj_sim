#!/usr/bin/env bash
# User-run, review-before-push local synchronization for 28 Oct 2026.
# It cannot run inside ChatGPT; it must be executed on the development laptop.
set -euo pipefail
WS="${F1TENTH_SIM_WS:-$HOME/f1tenth_sim}"
cd "$WS"
[[ -d .git ]] || { echo 'ERROR: not a git repository'; exit 1; }
[[ "$(git branch --show-current)" == "main" ]] || { echo 'ERROR: switch to/verify main before using this script'; exit 1; }
ORIGIN="$(git remote get-url origin)"
[[ "$ORIGIN" == *EngZaq/f1tenth_uj_sim* ]] || { echo "ERROR: origin unexpected: $ORIGIN"; exit 1; }

printf '\n=== PRE-PUSH STATUS ===\n'
git status --short
echo "Origin: $ORIGIN"

# Untracked `.env`, secrets and temporary dependencies are NOT to be uploaded.
# Preserve the local workspace before we stage anything. Archive lives OUTSIDE git.
BACKUP="$HOME/f1tenth_sim_before_push_20261028.tar.gz"
if [[ ! -f "$BACKUP" ]]; then
  echo "Creating source backup at $BACKUP ..."
  tar -czf "$BACKUP" --exclude='./.git' --exclude='./.venv' \
    --exclude='./build' --exclude='./install' --exclude='./log' \
    --exclude='./src/f1tenth_gym_ros' \
    --exclude='__pycache__' --exclude='*.pyc' .
fi

# Stage code, configs, runbooks and docs; dependency source is ignored.
for path in README.md docs scripts tools config reference maps src; do
  if [[ -e "$path" ]]; then git add -A -- "$path"; fi
done
# Stage historical map snapshots (SLAM posegraph/data only if each file <100MB).
if [[ -d results/mapping ]]; then git add -A -- results/mapping; fi

# Stage mission artifacts needed to reproduce the recovered map and final route.
M=results/missions/20261028_112407
for name in \
  new_track.yaml new_track.pgm lap_pose.json \
  new_track_centerline.csv new_track_mission_forward.csv \
  new_track_mission_forward_feasible.csv \
  new_track_speed_profile.csv new_track_speed_profile_feasible.csv \
  new_track_preview.png new_track_diagnostic.png \
  new_track_feasible_preview.png \
  explorer.log lap_watcher.log slam.log; do
  if [[ -f "$M/$name" ]]; then git add -- "$M/$name"; fi
done
# Keep representative localization CSVs; test log folders remain on the laptop.
if [[ -d results/localization ]]; then
  while IFS= read -r -d '' f; do git add -- "$f"; done < <(find results/localization -maxdepth 2 -type f -name '*.csv' -print0)
fi
if [[ -d "$M" ]]; then
  while IFS= read -r -d '' f; do git add -- "$f"; done < <(find "$M" -mindepth 2 -maxdepth 2 -type f \( -name 'estimated_drive.csv' -o -name 'live_scan_seed.json' -o -name 'amcl.log' \) -print0)
fi

printf '\n=== STAGED SUMMARY ===\n'
git diff --cached --stat
git diff --cached --check
printf '\n=== STAGED FILES ===\n'
git diff --cached --name-status

# Stop if any staged object exceeds GitHub hard file limit.
while IFS=$'\t' read -r mode type sha path; do
  [[ -z "${sha:-}" ]] && continue
  size="$(git cat-file -s "$sha")"
  if (( size > 95000000 )); then
    echo "ERROR: staged $path is $size bytes; too large for normal GitHub push."
    echo "Use git restore --staged '$path'; consider Git LFS or a release archive."
    exit 2
  fi
done < <(git ls-files --stage)

# Be explicit about risky names. Not a substitute for reviewing file contents.
if git diff --cached --name-only | grep -Ei '(^|/)(\.env($|\.)|id_rsa|id_ed25519|.*(password|private[_-]?key|token|secret).*)' >/dev/null; then
  echo 'ERROR: staged files have secret-looking names; inspect and unstage before pushing.'
  exit 2
fi

echo
printf 'Review the staged file list and verify no secrets or unintended files.\n'
read -r -p 'Type PUSH to COMMIT AND PUSH to origin/main, or anything else to cancel: ' answer
[[ "$answer" == PUSH ]] || { echo 'Canceled: nothing committed or pushed.'; exit 0; }

# Update origin/main before committing, never force-push or overwrite remote history.
git fetch origin main
if ! git merge-base --is-ancestor origin/main HEAD; then
  echo 'ERROR: local main does not include remote main. Resolve divergence manually; no commit/push attempted.'
  exit 3
fi
if ! git diff --cached --quiet; then
  git commit -m 'feat: document and integrate F1TENTH SLAM, AMCL and recovery results'
else
  echo 'Nothing new staged; skipping commit.'
fi
git push origin main
printf '\n=== REMOTE CHECK ===\n'
git status -sb
git log -1 --oneline
git ls-remote origin refs/heads/main
