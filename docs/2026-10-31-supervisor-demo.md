# Supervisor demo checkpoint — 31 October 2026

Current scope: **mapping, localization and planning**. Racing is paused. The
recorded mapping-only runs completed; moving AMCL accuracy and a repeatable
combined three-stage workflow remain unfinished. No physical-car changes are
included in this checkpoint.

## What is included

- The existing engineering history through `3dd8232`, including frame corrections,
  route-relative readiness, tracking-speed reduction and recorded failed full runs.
- The local demo's mapping-only exit after successful route preparation.
- The previously supplied 1 m/s exploration-ceiling patch, now integrated into
  the mission, explorer and watchdog. The default ceiling remains 0.18 m/s.
- Regression checks for stage-limited completion, explicit watchdog speed limits
  and exploration reaction/braking/turning bounds.
- The complete project log, editable supervisor slides, result images and
  compressed recorded mission/localization evidence.

The six source/test edits left in the older analysis checkout were already
present upstream by `2f60ebb`; they were not duplicated or used to overwrite
the newer documentation. This checkpoint is based on the newer remote history.

## Run the demonstrated mapping and planning stages

```zsh
cd ~/f1tenth_supervisor_demo
source ~/f1tenth_sim/scripts/sim_env.zsh
python3 tools/mission/full_autonomy.py --help
setopt PIPE_FAIL
zsh scripts/run_full_autonomy.zsh --mapping-only \
  2>&1 | tee supervisor_demo_console.log
```

The script starts the simulator and mapping stack. The stage sequence ends at
`COMPLETED_MAPPING_AND_PLANNING`. It starts no saved-map AMCL stage or autonomous
route follower in this mode. Stop/cleanup and the mission ZIP are still handled
by the launcher. Check that another simulator/controller stack is not running.

For the previously tested faster setting, append `--explore-speed 1.0` to the
command. This sets a maximum requested simulator speed, not a constant measured
vehicle speed. The explorer still reduces speed for clearance and steering and
limits upward acceleration. Braking/lateral assumptions are simulation-specific.

The wrapper without `--mapping-only` still requests the historical full mission.
Do not use that invocation to describe this stage-limited demonstration.

## Results from supplied recordings

| Metric | Baseline `124702` | Faster `131817` |
|---|---:|---:|
| Requested exploration ceiling | 0.18 m/s | 1.0 m/s |
| Total mapping/planning stage time | 260.601 s | 86.957 s |
| Exploration stage time | 230.179 s | 55.072 s |
| Closed optimized route length | 33.8245 m | 33.8588 m |
| Waypoints | 339 | 339 |
| Nominal minimum body clearance | 0.354 m | 0.358 m |
| Required peak steering | 21.62° | 23.88° |
| Mission status | Completed mapping/planning | Completed mapping/planning |

These are one run per setting, not a repeatability or equal-accuracy benchmark.
The fast-run steering limit is approximately 24°, leaving little reserve. The
reported clearance is nominal saved-map geometry, not a bound after localization
and tracking error. The fast run also logged a dropped scan and a SLAM exception
during shutdown after map saving; the outputs do not establish clean shutdown.

The optimized route contains a generated speed envelope, but it was not executed
by the mapping-only mission. The separate `new_track_speed_profile.csv` contains
a different 0.4 m/s profile; always identify the file and cap actually used.

## Localize separately while stationary

Start the normal simulator separately if the mapping launch has cleaned it up.
Use the existing saved mission explicitly; no exploration rerun is needed:

```zsh
python3 -u tools/mission/run_profiled_saved_map.py \
  --mission-dir results/missions/autonomous_20261031_131817_595257 \
  --localization-only --observe-seconds 45
```

Use a map with matching geometry metadata and no conflicting localization or
drive stack. This option does not start or arm the autonomous route controller.
The result folder may retain the historical `race_*` naming in observation mode.

Four fresh AMCL observations passed startup in the supplied later test. Its
412-observation passive audit nevertheless recorded AMCL relative-position RMS
0.44021 m and maximum 0.84802 m. These are all-sample, first-pose-aligned drift
statistics, not absolute map accuracy or a controlled AMCL/EKF ranking. The
historical controller stopped with zero laps and net progress 104/339 when its
local stopping tube rejected all evaluated speed proposals.

## Work still required

1. Diagnose scan/map agreement, timing, geometry and AMCL model consistency from
   existing recordings before changing more confidence thresholds.
2. Quantify repeatable map quality and closure, and fix dropped scans/shutdown.
3. Increase planning steering reserve and budget measured estimation uncertainty.
4. Validate short controlled localization motion, then combine the three stages
   without adding racing.
5. Commission real sensor/actuator geometry and calibration separately.

Validation of this checkpoint is recorded in the adjacent evidence directory.
The local packaging checks passed 113 unittest cases, the standalone trajectory
safety checks, changed-Python syntax parsing, archive integrity and the staged
diff whitespace check. The checked-in JSON lists the test files and base commit.
No new live ROS/Gym mission was run while packaging it. The original laptop
worktree may contain additional edits not included in the supplied files; this
checkpoint cannot capture unseen laptop-only changes.
