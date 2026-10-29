# Automatic simulation mission and the measured laser correction

The complete mission is now connected behind one command. This is an
experimental simulation workflow. The first uploaded integrated run completed
exploration, map saving and route preparation, then stopped before localization
on a stale SLAM-node graph check. The correction is described below. A complete
end-to-end ROS/Gym race has not yet been demonstrated with these changes.

## Current runtime uncertainty policy

The later full runs show that a single world-axis position sigma is not a
sufficient driving policy. With the revised laser model, lateral spread fell
to about 0.01-0.02 m but along-corridor spread sometimes remained near 0.10 m.
The car could repeatedly pause in a straight corridor even while its recorded
tracking error was small. The earlier scalar-gate experiments below are
historical; the current runtime policy explicitly changes that criterion.

The controller validates the full planar `(x,y,yaw)` covariance, including
symmetry and positive semidefiniteness. It projects translation uncertainty
onto the local route tangent and normal. Default pause bounds are 0.08 m
across track, 0.12 m along track and 10 degrees yaw. The largest position
ellipse axis still faults at 0.16 m, now independently of map rotation; the
20-degree yaw hard stop remains. Four fresh stationary observations within
the current bounds are required after a pause, with the same four-second
timeout. Initial readiness still requires four fresh estimates inside the
original 0.08 m world-axis bound before the controller starts.

Above 0.08 m longitudinal sigma, the commanded speed cap decreases smoothly
from its nominal value to one quarter at 0.12 m (0.40 to 0.10 m/s at defaults).
This provides additional measurements per metre without demanding that the
car become perfectly localized along a visually uniform corridor at rest.
Longitudinal spread above 0.12 m pauses; it is not ignored.

The runtime collision guard rolls out the requested Ackermann steering in
body coordinates. It no longer treats future map-route rectangles as the
commanded vehicle trajectory. That earlier assumption produced a false
obstruction at the first tight turn: the car and local walls share global
pose uncertainty, whereas fixed map targets do not. The earlier covariance
projection function remains available for offline reference-route analysis;
it is not the runtime braking decision.

Each scan is transformed from its acquisition time into the current body
frame through `fusion_odom`, using TF's advanced time API. The newest scan with
a complete local transform is selected from a bounded eight-scan buffer and
must still pass the 0.45 s freshness limit. The map frame does not enter this
transform. Full AMCL covariance still governs route confidence and speed.

The commanded motion is sampled at <=0.02 m arc intervals through reaction
travel plus `v^2/(2*0.7)` and a 0.10 m buffer, with the complete body footprint.
Reaction time is at least 0.35 s and increases to `scan_age + 0.25 s` for older
accepted scans. Guard speed is the largest of measured, previous commanded and
planned speed. A rigid-body interpolation bound covers gaps between samples.

The 0.07 m base clearance remains. The tube additionally covers a steering
transition between measured curvature (EKF yaw rate divided by speed) and the
requested curvature. For curvature difference `dk`, the centre/heading bound
is `(s^2/2 + body_corner_radius*s)*abs(dk)`. Three-sigma local speed and yaw-rate
allowances use 0.01 m/s and 0.01 rad/s respectively over the stopping interval.
These are explicit simulation assumptions; the model assumes steering stays
within the initial/requested interval and deceleration is at least 0.7 m/s².
They require separate actuator/noise measurements before hardware use.
Occluded/missed obstacles and sensor-model faults remain limitations.

The first live commanded-motion run then exposed a missing speed-planning
step: the route/AMCL cap rose from about 0.29 to 0.40 m/s just before a tight
turn, and the local braking tube reached the hard clearance bound. The
controller now tests descending speed proposals before publishing, each with
its own lookahead/steering and a guard speed no lower than measured/previous
commanded speed. It prefers the highest sampled proposal with 0.15 m adjusted
clearance (the 0.07 m hard bound plus 0.08 m planning reserve). If only hard
clearance is available, it selects the slowest hard-clear proposal, down to
0.05 m/s. If no proposal is hard-clear it still latches a stop. Speed reduction
cannot erase the stopping distance of an already moving car. This makes
clearance an input to speed planning rather than only a terminal rejection.
The chosen speed is still acceleration/deceleration limited. The CSV records
the pre-local route cap and whether local clearance reduced it. A stopped run
also retains scan ranges, the local transform and evaluated proposals in
`guard_stop_context.json` for a direct geometry replay.

The existing `predicted_body_gap_m` CSV field now reports that conservative,
local-motion-tube-adjusted gap. New fields include `amcl_major_std_m` and
`uncertainty_speed_cap_mps`. The controller parameters are
`max_across_track_std_m`, `max_along_track_std_m` and
`max_major_position_std_m`; the former profiled-controller
`max_position_std_m` scalar parameter is replaced by these explicit bounds.

Tests check covariance projection, invalid/correlated matrices and recorded
corridor uncertainty, plus commanded turns toward/away from walls, braking
distance, scan motion compensation, steering transition and local noise. Small backward nearest-waypoint corrections now subtract from
net progress instead of looking like an almost-full-lap jump. Large progress
and pose discontinuities still stop the controller; estimator jitter cannot
artificially add laps.

## Second integrated run: confidence recovery correction

`autonomous_20261029_182122_744147` passed both fresh graph checks, initialized
AMCL and drove approximately 2.35 m before pausing. During moving samples the
estimated route CTE peaked at 0.051 m and the predicted scan/body gap stayed
above 0.465 m. The passive, first-pose-aligned audit reports moving relative
position RMS error about 0.020 m, maximum 0.039 m. Those short-run relative
measurements do not establish absolute map accuracy or whole-track clearance.

The terminal fault was `AMCL did not relocalize in 4.0s: pos_std=0.070m`.
The entry condition allowed 0.080 m position standard deviation, but the old
pause exit required `0.8 * 0.080 = 0.064 m` and four stationary fresh estimates.
Recorded uncertainty returned to 0.068-0.078 m, so the tighter exit never
counted a good sample. Lateral uncertainty was around 0.02-0.03 m; most spread
was along the corridor. This supports correcting recovery before changing the
estimator or raising the admission bound.

Recovery now uses the same position/yaw limits as entry and retains four
distinct increasing pose stamps while measured EKF speed is <=0.08 m/s. This
provides a time dwell before resuming. Above-limit or moving samples reset the
recovery count; out-of-order timestamps cannot contribute. The 0.080 m entry,
0.160 m hard position limit, 10/20 degree yaw limits, four-second pause timeout,
tracking error limit and collision checks remain in force. The recovery exit
threshold changed explicitly; this is not a claim that tighter covariance was
obtained from the same measurements.

Replaying measured covariance/stamps from the uploaded pause reaches four
acceptable observations about 1.50 seconds into that stop, instead of timing
out. That replay assumes zero measured speed after stopping because the old
controller CSV omitted EKF speed. The new CSV includes `measured_speed_mps`
and exact `amcl_stamp_s` for future checks. Stationary no-motion update requests
continue at the bounded refresh rate throughout a confidence pause.

The offline suite includes this recorded regression plus moving/unsafe sample
resets and out-of-order estimates. A new independent CI workflow builds pinned
Gym dependencies, applies the recorded simulator patch, and runs the complete
unknown-map mission on the technical track under ROS Humble/CycloneDDS. Its
artifacts include console output, maps, routes and passive audit logs on success
or failure. This full test is separate from the existing ROS graph-only test.
See [confidence evidence](evidence/confidence_recovery_findings.json) and the
[full Gym workflow](../.github/workflows/autonomy-gym.yml) for assumptions and
current verification status.

## Coverage completion discovered by full Gym CI

The full ROS/Gym run also exposed a distinct mapping handoff problem: the EKF
return detector stopped after about 34.1 m even though the saved map contained
an unknown wedge that broke its 0.30 m clearance corridor. The extractor
correctly rejected that map instead of filling the gap or reducing clearance.

`mission_first_lap.py` now also subscribes to the observed SLAM occupancy grid.
It requires a fresh map with a closed 0.30 m corridor around a substantial
interior, using the same free/unknown interpretation as the route extractor.
If the odometry return happens while coverage is still open, exploration
continues within the existing timeout. Only a return with observed closed
coverage ends mapping. The later full-body and steering audit is still needed;
this topology check alone does not establish route feasibility or a verified
SLAM pose-graph loop closure.

The actual rejected CI mask and the prior passing laptop mask are retained in
`tests/fixtures/map_coverage_recordings.json`. Tests verify the difference and
that unknown cells cannot silently become free. The map-coverage decision is
included in `lap_pose.json` for each new run.
If coverage requires another mapping lap, total travel remains in
`distance_odom_m`, while `estimated_lap_length_m` preserves the first return's
single-lap length. Route extraction uses the latter so extra exploration does
not incorrectly double the expected circuit length.

## Simulation AMCL measurement model under full-run evaluation

The coverage-gated full CI run passed mapping, planning, seeding and readiness,
then stopped on sustained 0.081-0.086 m world-axis position uncertainty. Its
lateral spread stayed about 0.023-0.026 m while longitudinal spread approached
0.10 m. The admission threshold correctly remained in force: changing the
recovery exit alone cannot solve a posterior that stays outside admission.

The simulation likelihood-field model now tests `sigma_hit=0.10 m` instead of
0.20 m and 120 sampled beams instead of 60. The simulator range noise is
0.01 m, saved-map cells are 0.05 m, and the latest startup scan-match squared
hit score corresponds approximately to 0.04 m residual scale. The 0.10 m
setting retains slack for map error; it is a testable simulation-model choice,
not a formally calibrated sensor-confidence claim. The random-hit mixture,
motion noise, initial 0.10 m/5 degree prior and controller admission/hard limits
remain unchanged. Full-run passive truth errors must be inspected alongside
covariance; artificial confidence is not an acceptance result.

An intermittent stationary initialization deadlock also appeared in CI: AMCL
logged that it accepted the seed, but the initializer missed its pose reply.
The old helper waited for another pose while all no-motion refresh callers
started only after that acknowledgement. It now publishes the seed exactly
once, then requests bounded asynchronous no-motion updates while awaiting a
fresh reply. This avoids repeatedly resetting the particle cloud. A live ROS
fixture deliberately withholds the initial reply and confirms that a service-
driven estimate completes the handshake without drive or Gym-reset publishing.

## First integrated run: SLAM-to-AMCL handoff correction

The uploaded run `autonomous_20261029_164139_586206` explored 34.212 m over
227.07 seconds, saved a 220 x 296 map at 0.05 m resolution and generated a
374-waypoint route. The route passed the existing nominal body/steering checks:
0.250 m clearance at waypoints, approximately 0.224 m between waypoints and
23.74 degrees peak required steering. These are map-geometry measurements.

`slam.log` reports that the SLAM child process finished cleanly. Nevertheless,
the racing launcher's immediate `ros2 node list` check still returned
`/slam_toolbox`, producing `RESUME STOPPED: /slam_toolbox already running`.
No AMCL or racing controller was started in this attempt. This evidence points
to stale CLI-daemon/discovery state during the stage handoff, not a demonstrated
failure of the newly generated route.

The parent now enters `WAITING_FOR_LOCALIZATION_HANDOFF` and verifies departure
with `ros2 node list --no-daemon --spin-time 1.0`, allowing up to 30 seconds and
requiring two consecutive clear observations. The child performs its own fresh
check before starting localization. Topic-ownership queries also bypass the
daemon. A running conflicting stack still blocks startup, and a failed query
cannot count as an empty graph. The watchdog keeps commanding zero while no
controller is active. Nothing resets or kills an unrelated process.

The parent also drains the final race log after the child exits and includes
its error in the terminal message. Fast child exits previously hid that error
behind a generic filename. `mapping_handoff.json` and
`race_*/localization_handoff.json` retain the discovery observations.

The offline regression suite covers delayed departure, transient empty graph
observations, a genuinely active SLAM node, discovery errors/timeouts and fast
child failure reporting. A separate CI job uses ROS Humble with CycloneDDS to
warm daemon discovery with a real `slam_toolbox`-named node, reject it while
alive, stop it and verify the fresh handoff succeeds. This checks ROS graph
ownership; it does not run SLAM, AMCL, Gym or the complete driving mission.

An additional ideal kinematic replay on this uploaded route completed three
laps with peak CTE 0.04394 m using the current policy and 0.40 m/s cap. That
replay assumes perfect pose and omits sensor/actuator dynamics, so it is not
evidence of actual localization/tracking performance. Source hashes and
measurements are saved in [handoff evidence](evidence/slam_amcl_handoff_findings.json).

ROS Humble CLI implementation references:
[fresh/daemon node strategy](https://github.com/ros2/ros2cli/blob/humble/ros2cli/ros2cli/node/strategy.py)
and [topic-info strategy](https://github.com/ros2/ros2cli/blob/humble/ros2topic/ros2topic/verb/info.py).

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
| Exploration | Use the LiDAR explorer; require an estimated return plus a fresh observed closed corridor before stopping and holding zero speed |
| Map save | Save a new YAML/PGM, capture the current SLAM-frame pose and stop SLAM before AMCL starts |
| Planning | Extract and orient a closed centerline, try periodic smoothing candidates, optimize and check full-body clearance/steering; reject all-invalid candidates |
| Graph handoff | Verify departure of mapping nodes through bounded fresh discovery before starting localization |
| Localization | Restart local sensor/EKF chain, fit current LiDAR against this new map and send an explicit covariance seed on `/fusion/initialpose` |
| Readiness | Wait for four distinct fresh AMCL estimates within the existing position/yaw uncertainty gates; time out after 45 seconds if readiness fails |
| Racing | Start the profiled controller on the new route, automatically arm, count the requested estimated-pose laps and retain its fault gates |
| Completion/stop | Save stage status and an offline truth-audit summary when available, clean up owned processes and create one ZIP with run outputs |

The return detector still uses an odometry/heading heuristic plus a necessary
observed-coverage gate. A closed extracted route and an estimated return are
not proof of a verified SLAM pose-graph loop
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

The current route-relative admission and commanded local swept check are
specified above. Tracking-error, hard uncertainty and base swept-margin limits
remain in force. If a stage cannot satisfy its checks, the run stops and
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

ROS 2/Gym are not installed in the local editing runtime. Uploaded laptop runs
supply exploration/mapping/planning and short AMCL driving evidence. ROS and
full Gym execution use separate CI jobs, with their results and logs retained
in GitHub Actions. A graph-only pass and an offline covariance replay do not
substitute for successful full-mission execution.
