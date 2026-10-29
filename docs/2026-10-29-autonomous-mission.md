# Automatic simulation mission and the measured laser correction

The complete mission is now connected behind one command. This is an
experimental simulation workflow; a successful end-to-end ROS/Gym run has not
yet been demonstrated with these changes. Earlier tests remain evidence for
their individual stages, not proof of the new integrated mission.

## The latest recording found a concrete mismatch

The uploaded `stationary_probe.json` reports these actual transforms:

| Transform / consumer | Laser x from body origin |
| --- | --- |
| Gym `ego_racecar/base_link -> ego_racecar/laser` | **0.100 m** |
| Old `fusion_base -> fusion_laser` | **0.275 m** |
| Old scan-to-map initializer and swept obstacle guard | **0.275 m** |

We carried the stock 0.275 m assumption into this custom simulator. It disagreed
with the measured transform by **0.175 m**. Frame renaming does not move laser
rays, so AMCL, SLAM and collision calculations were being told the wrong
measurement origin. This is a confirmed geometry-contract defect and a
credible contributor to the estimator behavior. It is not proof that every
localization or tracking issue is resolved.

An independent offline replay strengthens the finding. On the simulator's
technical-loop map, ray marching from the recorded stationary truth pose gives
range RMSE **0.01697 m** with the measured 0.100 m laser origin, versus
**0.36438 m** with 0.275 m. This uses 271 recorded beams and a 0.00625 m ray-march
step. This is an approximate raster-model consistency check, not an online
correction or a localization-accuracy certificate. The recorded pose is used
only in the offline regression fixture. No controller, initializer or planner
loads that truth pose.

Source checksum and measured values:
[geometry evidence](evidence/sim_laser_geometry_findings.json).

## What changed

`config/sim_laser_geometry.yaml` supplies x/y/yaw to the scan relay, scan matcher
and swept obstacle guard. Before relaying scans, the relay checks the actual
body-to-laser TF against that calibration and rejects disagreement or a
nonplanar mount. It never reads global truth pose to steer or initialize AMCL.

Maps generated with the old geometry may contain geometry-dependent errors.
Automatic racing therefore requires a matching `laser_geometry.json` beside
the map. The full mission builds a fresh map and route and writes that record.
Old maps and logs are retained; the automatic runner does not silently reuse
the October 28 map with the new mounting assumption.

The 150-request stationary probe now works at approximately 0.3 s AMCL updates.
Its wheel/gyro statistics agree with the intended noise scale and show no
sampled true motion. Those are useful checks; they do not validate estimator
performance during motion.

## Start the complete automatic sequence

From the existing review worktree:

```bash
cd ~/f1tenth_sim_seed_check
git fetch origin engineering/autonomous-mission-20261029
git merge --ff-only origin/engineering/autonomous-mission-20261029
zsh scripts/run_full_autonomy.zsh
```

The wrapper reuses the existing built workspace/venv through `sim_env.zsh`, then
runs source from this review checkout. It starts Gym when it is not already
running. An already healthy simulator is reused, and no simulator reset pose
is published. Existing autonomy publishers or conflicting AMCL/SLAM nodes cause
preflight failure rather than being killed or overwritten.

Defaults: exploratory speed 0.18 m/s, racing cap 0.40 m/s, three estimated-pose
laps. All intermediate prompts are removed by the explicit `--autonomous`
flag in this wrapper. Optional arguments:

```bash
zsh scripts/run_full_autonomy.zsh --laps 3 --speed-cap 0.40 --explore-speed 0.18
```

## What it does without manual handoffs

| Stage | Automatic behavior |
| --- | --- |
| Preflight | Check existing nodes, actuation ownership and simulator; start the command watchdog and sensor/EKF/SLAM stack |
| Exploration | Use the LiDAR explorer; detect an estimated return to the start, stop exploring and hold zero speed |
| Map save | Save a new YAML/PGM, capture the current SLAM-frame pose and stop SLAM before AMCL starts |
| Planning | Extract and orient a closed centerline, try periodic smoothing candidates, optimize and check full-body clearance/steering; reject all-invalid candidates |
| Localization | Restart local sensor/EKF chain, fit current LiDAR against this new map and send an explicit covariance seed on `/fusion/initialpose` |
| Readiness | Wait for four distinct fresh AMCL estimates within the existing position/yaw uncertainty gates; time out after 45 seconds if readiness fails |
| Racing | Start the profiled controller on the new route, automatically arm, count the requested estimated-pose laps and retain its fault gates |
| Completion/stop | Save stage status and an offline truth-audit summary when available, clean up owned processes and create one ZIP with run outputs |

The return detector is still an odometry/heading heuristic. A closed extracted
route and an estimated return are not proof of a verified SLAM pose-graph loop
closure. Repeated-map/start/fault acceptance remains future verification work.

## Command ownership and stopping

The new mission uses `/mission/drive` for the explorer/controller command input.
`drive_watchdog.py` is the single `/drive` publisher, operating at 20 Hz. It
outputs zero on missing commands after 0.25 s, stale scans, invalid/out-of-range
commands, collision, or competing `/drive` ownership. Collision and severe
input/scan/ownership faults latch and create `watchdog_fault.json`. The parent
monitors that file and child exits. If the watchdog itself exits, parent cleanup
uses a direct zero-speed fallback. The watchdog stays alive while other owned
processes are shut down.

This is a heartbeat/collision/command arbiter. It is not an independent obstacle
planner or physical emergency stop. The controller's swept endpoint guard and
the explorer's obstacle checks remain responsible for their proposed motions.
The complete chain still needs real Gym fault-injection tests, including node
crashes; the pure policy tests are not substitutes for those live tests.

No covariance, tracking-error or swept-margin threshold was increased to force
completion. If a stage cannot satisfy its current checks, the run stops and
records the failed stage. A failed or interrupted racing process cannot report
success merely by exiting with code zero: the requested lap-completion record
is required.

## Outputs and meaning of success

Each run uses a new `results/missions/autonomous_<timestamp>/` folder with stage
status, subprocess logs, command results, fresh map/route, calibration record,
planner attempts and a nested racing folder. The parent prints an uploadable
sibling ZIP path after completion or failure. Upload that single ZIP to review
the integrated result; no separate stationary test is requested.

`COMPLETED_ESTIMATED_LAPS` means the controller's estimated-pose lap checks
completed. It does not assert independently verified absolute localization,
true route CTE, minimum true wall clearance, loop-closure graph quality or
hardware readiness. Passive truth is retained for evaluating those claims.

## Verification performed here

- Offline unittest suite, including automatic no-prompt handoffs, readiness
  refusal, planning failure, missing lap-completion record, log packaging,
  watchdog heartbeat/collision policies and calibration consistency.
- A recorded-scan regression independently rejects the old 0.275 m origin.
- The real automatic route-preparation command accepts the saved feasible
  route fixture and writes its audited output; rejection handling is tested.
- Standalone swept collision check, source compilation and zsh syntax checks.

ROS 2/Gym are not installed in this editing environment. No new mapping lap,
live watchdog stop or end-to-end race has been executed here. The next laptop
run exercises the complete workflow rather than another stationary diagnostic.
