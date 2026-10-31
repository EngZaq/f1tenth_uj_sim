# F1TENTH University of Jordan — complete project progress and handoff log

**Status date:** 31 October 2026, Jordan time (UTC+3).
**Coverage:** Project background and significant work recorded from 26–31 October, with emphasis on the latest simulator tests.
**Purpose:** Preserve what we built, why we changed it, what the evidence actually shows, what prevents completion, and what to do next. This file is suitable for the team, supervisor, or another engineer continuing the work.
**Current scope chosen by the project owner:** Mapping, localization and planning. Racing is paused and is included here only as development history and diagnostic evidence.
**Changes made while preparing this log:** Documentation only. No project source files, branches or remote repositories were changed or pushed.

## 1. The current situation in plain language

We have made substantial progress. The simulator can explore the custom track, build and save a map, extract a centreline, and generate a route that passes nominal vehicle-body and steering checks. Two 31 October mapping-only runs reached `COMPLETED_MAPPING_AND_PLANNING`.

We also have an implemented localization pipeline: synthetic motion sensors → EKF local odometry → AMCL using live LiDAR and the saved map. Automatic scan-based initialization and fresh-pose readiness checks work in the supplied tests.

**The main unfinished problem is reliable localization while moving.** Passing AMCL's initial confidence check does not establish accurate global position throughout a run. In the latest uploaded moving test, the passive audit recorded approximately 0.44 m AMCL relative-position RMS error and 0.85 m maximum relative error. These are first-pose-aligned drift measurements, not absolute saved-map accuracy.

The historical full driving mission also remains unfinished. Different runs stopped for different reasons: stale ROS graph observations, covariance recovery timeouts, estimated cross-track error, and a predicted local stopping-tube obstruction. Some underlying integration defects were corrected; a complete repeated-lap acceptance result has not been demonstrated in the evidence reviewed for this handoff.

**The next milestone is a repeatable, measured mapping–localization–planning workflow.** We should improve and validate each stage, then combine those three stages. Higher-speed racing and physical autonomous driving remain later milestones.

### Status ledger

| Area | What exists / what worked | Remaining limitation |
|---|---|---|
| Simulator and custom track | F1TENTH Gym ROS, ROS 2 Humble and Foxglove were used successfully | Reproducibility across clean installations needs continued documentation |
| Unknown-track exploration | Reactive LiDAR exploration completed recorded mapping missions | Only limited tracks/start conditions tested; no general robustness claim |
| Faster exploration | A 1.0 m/s command ceiling completed one mapping-and-planning run | It is a ceiling, not constant actual speed; repeatability and map accuracy unproven |
| SLAM and map saving | Generated PGM/YAML maps; recent resolution 0.025 m/cell | Absolute map accuracy and independent loop-closure validation remain open |
| Centreline / route generation | Automatic route generation, smoothing and nominal body audit | Fast-run steering demand is close to the limit; uncertainty margins need integration |
| EKF sensor fusion | Synthetic wheel-speed and gyro inputs produce local odometry | Simulator measurements are emulated; physical calibration/slip remain unvalidated |
| AMCL initialization | Live-scan seed and four fresh startup observations passed | A plausible seed and low covariance do not prove accurate moving localization |
| AMCL during motion | Runs produce poses and covariances; passive audit records errors | Significant relative drift in the latest test; root cause not established |
| Stage orchestration | Automatic explore → save → plan; experimental localization handoff | Need a dedicated, validated three-stage workflow without racing |
| Fault handling | Command/scan watchdogs, confidence checks, body/steering checks and latched stops | These are engineering safeguards, not a complete physical safety validation |
| Estimated-pose full mission | Partial progress in several tests | No demonstrated complete repeated-lap mission on the newly generated map |
| Physical car | Several hardware components and manual driving reported operational | Full autonomous stack, geometry, timing, steering and braking validation incomplete |

## 2. Project objective, team and constraints

This is a Mechatronics Engineering graduation project at the University of Jordan. The broader original objective is a physical Ackermann-steered F1TENTH car that can start on an unknown closed track, explore it, build a map, localize, generate a route, and eventually drive repeated autonomous laps. Progressive speed improvement is a later research goal.

The simulation-first approach allows software problems to be found before testing a custom physical vehicle. It does not remove the need to discover hardware-specific faults later.

Previously discussed team responsibilities:

- **Zaid / project owner:** Overall integration, testing, system understanding and supervisor coordination.
- **Moatasem:** Sensor fusion and localization work.
- **Ahmad:** Planning work.
- Previously discussed availability: approximately 15–25 hours per week per team member; physical-car access reported available daily. These are planning assumptions, not a new commitment.
- Previously discussed project window: 26 October–26 December 2026. The schedule should be revised around measured readiness.

Working preferences that must be retained:

1. Explain changes step by step so the owner understands the graduation project.
2. Do not change the physical-car main repository directly. Provide proposed commands and explanations for review.
3. The owner said not to push changes during the supervisor-demo work. Do not infer permission for a new push or merge from this log.
4. Preserve generated results and successful baselines. Do not delete logs or overwrite evidence to make the workspace look clean.
5. Do not call a partial run, a planned route or an offline model test a completed autonomous mission.
6. Current active scope excludes racing. Historical racing failures are useful diagnostics, not instructions to resume racing.

## 3. Repositories, workspaces and environment

### Repositories

- Simulation: https://github.com/EngZaq/f1tenth_uj_sim
- Physical car: https://github.com/EngZaq/f1tenth_uj

Historical engineering branches mentioned in the work:

- `engineering/swept-path-safety-20261029`
- `engineering/localization-evidence-20261029`
- `engineering/autonomous-mission-20261029`
- Local supervisor demonstration branch: `demo/supervisor-20261031`

Important revisions from supplied terminal records:

| Revision | Meaning in the recorded work |
|---|---|
| `6c01448` | Earlier swept-path / localization-evidence worktree state |
| `4f8f774` | Isolated AMCL initial pose from the simulator reset topic |
| `c9519a8` | Added connected experimental mission, watchdog/readiness/geometry-related work |
| `65244ba` / `0e8d0c7` | Composed CoG/rear-axle geometry; corrected pinned Gym class import for the live model contract check |
| `70f85ee` | Shorter low-speed Pure Pursuit lookahead for technical bends |
| `10e60b0` | Stopping-buffer/sampling correction and CI evidence work |
| `3aa3bd5` | Finer occupancy cells and denser mapping constraints in an engineering checkout |
| `3dd8232` | Last explicit user-reported demo base: documentation of stage validation and unresolved AMCL recovery |

These are historical states, not a declaration of the current GitHub branch tip. No fresh remote synchronization was performed for this documentation task. The user's laptop demo worktree contains local changes beyond its base commit.

The engineering checkout available for analysis also differs from the user's demo checkout: it is at `3aa3bd5` with six modified source/test files. It does not contain every local demo change, including the same mapping-only / faster-exploration interface. **Do not transfer or reset a checkout based only on a commit number in this document.** First inspect the actual files, branch and diff on the intended machine.

### Laptop environment used in the tests

- Ubuntu 22.04; ROS 2 Humble; zsh.
- Main dependency workspace: `~/f1tenth_sim`.
- Python virtual environment: `~/f1tenth_sim/.venv`.
- Environment setup: `source ~/f1tenth_sim/scripts/sim_env.zsh`.
- `ROS_DOMAIN_ID=42`.
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.
- Foxglove WebSocket: `ws://localhost:8765`.
- Review/demo workspaces used: `~/f1tenth_sim_autonomy_audit`, `~/f1tenth_sim_seed_check`, `~/f1tenth_supervisor_demo`.

Detached Git HEAD in a review worktree is not itself an error. Generated results are often untracked and therefore absent from GitHub. A clean remote repository does not mean all local experiments are archived.

### Maps

- `uj_oval_debug`: simpler simulator track for debugging.
- `uj_technical_loop_v1`: more demanding custom closed track used in the work.
- `new_track.yaml` / `new_track.pgm`: maps generated during individual missions.

The simulator's preloaded environment map and the map constructed by SLAM are separate data products. Showing the preloaded map does not prove mapping quality.

## 4. How the current architecture works

### A. Unknown-track mapping

1. Gym simulates the car and LiDAR.
2. The explorer uses live LiDAR to select a local steering direction and speed without a pre-supplied route.
3. The sensor emulator produces wheel-speed/VESC-like and gyro measurements from simulator motion quantities.
4. The EKF combines motion measurements into local odometry.
5. The scan relay provides scans in a consistent fusion laser frame at a controlled processing rate.
6. SLAM Toolbox combines scans and local odometry to build the map and estimate the map-to-odometry transform.
7. The lap/coverage logic recognizes an estimated return with an observed closed corridor, stops exploration, and saves the map.

The estimated return is not the same as an independently verified SLAM graph loop closure. For example, the faster mapping run's recorded return distance was about 0.837 m with approximately 14.67° yaw discrepancy.

### B. Planning

1. Read occupancy and map metadata.
2. Extract a centreline in observed free space.
3. Smooth and optimize a candidate route.
4. Check the full vehicle body against occupied and unknown map cells.
5. Check required steering, including geometry between route samples.
6. Generate route arc length, heading, curvature and a speed profile.
7. Accept or reject the candidate and save the audit report.

Unknown map space must not silently become driveable free space. A centreline is an intermediate geometric object; the body-audited route is the candidate intended for subsequent use.

### C. Saved-map localization

1. Use the saved map and a live scan to find an initial pose hypothesis.
2. Send the pose to AMCL through the dedicated initialization topic.
3. AMCL combines live scans with odometry to estimate the pose in the saved-map frame.
4. Require fresh, finite, sufficiently confident observations before reporting readiness.
5. Evaluate errors through an independent passive audit.

The latest startup result was `fresh confident AMCL observations 4/4`. This confirms the startup contract, not ground-truth position accuracy.

### D. Historical path execution, currently paused

The estimated-pose controller uses the map-frame route and estimated pose. It selects lookahead, steering and speed while applying confidence, stale-data and collision guards. A separate watchdog owns the final drive topic in the orchestrated mission. Recorded failures from this mode revealed important localization and integration problems.

This implementation exists, but it is outside the current demonstration milestone. Its existence is not evidence of completed racing.

### Simulator truth boundary

- Global simulator pose is used by passive evaluation, not as AMCL's pose input or the estimated controller's steering state.
- The synthetic sensor adapter uses simulator motion quantities to emulate measurements. That is an unavoidable simulator interface, and must not be described as real-sensor validation.
- Earlier oracle baselines deliberately used privileged simulator pose. Their successful laps are controller baselines, not proof of estimated-pose autonomy.

## 5. Topics, frames and ownership

| Topic / frame | Purpose | Caution |
|---|---|---|
| `/scan` | Raw simulator LiDAR, approximately 30 Hz | Use its actual acquisition frame and timestamp |
| `/fusion/scan` | Relayed scan for mapping/localization | Target rate and measured delivered rate can differ |
| `/ego_racecar/odom` | Simulator pose/twist source | Privileged pose: passive audit / explicitly labeled oracle experiments only |
| `/sim/vesc/wheel_odom`, `/sim/vesc/erpm` | Synthetic wheel/VESC-like measurements | Simulator conversion is not physical VESC calibration |
| `/sim/imu/data_raw` | Synthetic gyro-like measurement | Do not invent reliable absolute heading |
| `/odometry/filtered` | EKF local estimate | Drifting local reference, not independently globally corrected position |
| `fusion_odom → fusion_base` | EKF motion transform | One publisher/owner |
| `fusion_base → fusion_laser` | Calibrated sensor transform | `fusion_base` is the rear axle in the corrected engineering convention |
| `slam_map → fusion_odom` | Global correction | SLAM during mapping OR AMCL during saved-map localization, not both |
| `/mission/slam_map` | Map constructed by SLAM | Distinct from Gym's preloaded `/map` |
| `/fusion/localization_map` | Saved map served for AMCL | Must match the map and geometry used for planning |
| `/amcl_pose` | AMCL estimated pose and covariance | Small covariance can coexist with a wrong estimate |
| `/fusion/initialpose` | Dedicated AMCL seed | Isolated from Gym reset handling |
| `/initialpose` | Existing simulator reset-sensitive topic | Do not reuse it casually for saved-map AMCL seeding |
| `/mission/drive` | Internal requested command in watchdog-controlled mode | Final ownership remains with the watchdog |
| `/drive` | Simulator actuation command | Exactly one authorized final publisher |
| `/ego_racecar/collision` | Simulator collision indication | Collision should latch a stop |

For Foxglove, use `slam_map` as the fixed frame when inspecting the generated/localization map. Display `/fusion/localization_map`, `/fusion/scan` and `/amcl_pose` together during a live localization session. Missing topics after the test's cleanup are not proof that they failed while the test was running. Changing the display frame is a visualization fix, not an estimator correction.

## 6. The geometry correction that must not be forgotten

There were two different physical reference points in use:

- The pinned Gym single-track state / raw `ego_racecar/base_link` represents the **centre of gravity (CoG)**.
- The engineering EKF kinematics, route footprint and steering calculations use the **rear axle** as `fusion_base`.

Measured raw LiDAR mounting and the rear-axle conversion must be composed. Renaming the raw scan frame without translating its parent reference point was incorrect.

| Quantity | Correct engineering simulation value |
|---|---:|
| Raw CoG → laser x | 0.10000 m |
| Rear axle → CoG x | 0.17145 m |
| Rear axle (`fusion_base`) → laser x | 0.27145 m |
| Rear axle → collision-body centre x | 0.16510 m |
| Body length × width | 0.58 × 0.31 m |
| Front / rear body extent relative to rear axle | +0.45510 / −0.12490 m |
| Wheelbase used in the model | Approximately 0.3302 m |

The early report mentioned 0.275 m, and the first raw scan investigation identified 0.100 m. These are historical stages of the investigation. **The final fusion-frame value is 0.27145 m, not simply 0.100 m.**

An offline raw-scan consistency check recorded approximately 0.017 m range RMSE using the measured raw 0.100 m mount, compared with approximately 0.364 m under the old raw 0.275 m assumption. This was an offline geometry check, not proof of subsequent AMCL accuracy.

The passive truth audit was also corrected to compare the same rear-axle point. Comparing a CoG truth trajectory with a rear-axle estimate produces apparent error during turns even with perfect motion integration. Older error figures that mixed reference points must not be interpreted as corrected rear-axle accuracy.

The actual pinned Gym implementation was checked through a model-contract test. Historical pins recorded in the engineering work:

- Gym: `bdaec1420c3b0f103858d289866d0d4e2e597c30`.
- ROS bridge: `08395766c4d9dc5a763381f1dd6fa4a3d68df66e`.

Changing the model/version requires rechecking the geometry contract. Old generated maps may be incompatible with new geometry metadata; do not bypass the matching check or relabel an old map as newly calibrated.

## 7. Development timeline: 26–28 October

These earlier results are preserved from the engineering handoff and development reports. They were not all rerun for this log, and their maps, estimators and reference conventions differ.

### 26–27 October — establish baselines and discover frame/estimator problems

- Set up the simulator, custom tracks and ROS bridge.
- Generated early SLAM maps and extracted candidate routes.
- Found that using a SLAM-frame route directly in the simulator's map frame could cause a collision.
- Established an oracle-only rigid registration for controlled baseline tests.
- Ran Pure Pursuit using privileged pose: a recorded five-lap series had lap times around 26.53–26.58 s and mean CTE around 0.026 m. This showed the controller could follow the registered route when given accurate pose.
- Compared longer lookahead settings; they changed tracking error and corner behavior.
- Investigated RF2O, command-integrated odometry and SLAM localization. These showed drift, jumps or both; simply changing scan rate did not resolve localization.

Selected historical measurements, kept with their original limitations:

| Experiment | Reported historical outcome | Interpretation |
|---|---|---|
| Oracle Pure Pursuit, five laps | 26.584, 26.537, 26.551, 26.540, 26.528 s | Privileged-pose controller baseline only |
| Timestamp-aligned RF2O | Mean relative error ~0.285 m, max ~0.593 m | Local laser odometry drift remained |
| Timestamp-aligned SLAM localization | Mean ~0.649 m, max ~1.806 m; pose jumps | Unsuitable as an unqualified fast-control pose source |
| Earlier RF2O-only run | Mean ~1.639 m, max ~10.367 m | A failed historical configuration, not the current EKF pipeline |

These figures are not a controlled benchmark between the present algorithms. Sensor/reference definitions evolved later.

### 28 October — mapping and saved-map navigation developed further

- Built/used synthetic wheel-speed and gyro inputs with EKF local odometry.
- Used AMCL with a known saved map; one earlier known-map estimated-pose lap was reported complete at approximately 0.35 m/s on a 344-point reference.
- This known-map success was not a complete unknown-map → freshly generated map → repeated-lap mission.
- Completed an unknown-track exploration with approximately 34.422 m estimated travel over 229.2 s.
- Saved a 225×296-cell SLAM map at 0.05 m/cell for that earlier run.
- Iterated centreline extraction after earlier failed candidates. A V3 centreline had 377 points and approximately 37.67 m length.
- Generated steering-feasible routes and added full-body audits. One original route had only about 0.112 m nominal minimum body clearance.
- Added optimization that produced a candidate with approximately 0.150 m nominal body clearance.

Key lesson: a geometrically valid centreline is not automatically a feasible body trajectory, and a body-feasible route is not automatically safe under pose/tracking uncertainty.

## 8. Development timeline: 29 October

### A. Earlier moving saved-map test: `profiled_20261029_142531`

- Reported net progress reached 41 of 377 route samples.
- Commanded speed reached approximately 0.40 m/s.
- Estimated CTE was small in reported drive messages, but this was computed from the same pose estimate being evaluated.
- AMCL covariance rose; the confidence pause timed out after 4 s.
- The older gate used a world-axis position standard deviation around 0.08 m and a hidden stricter recovery threshold around 0.064 m.

We learned that low estimated CTE does not independently establish true path tracking. A passive timestamp-paired audit was necessary. Later CoG/rear-axle corrections also mean some early drift figures cannot be directly compared with the latest corrected audits.

### B. AMCL seed isolation and stationary observation: `profiled_20261029_153559`

- Revision: `4f8f774`.
- Started sensor emulator, EKF and scan relay; matched a live scan to the recovered map.
- Sent AMCL seed with approximately 0.10 m XY standard deviation and 5° yaw standard deviation.
- First AMCL pose agreed closely with the seed: about 5 mm / 0.5° in the console.
- No autonomous controller was started and no arm request was sent.
- The 45.636 s recording contained 28 paired observations. Recorded truth endpoints stayed fixed.
- AMCL moved about 0.07764 m from its first estimate, mostly along its initial heading; heading changed about −1.603°.
- Final uncertainty projected into vehicle axes was approximately 0.115 m longitudinal and 0.029 m lateral.

This confirmed the seed/initialization behavior and absence of a sampled reset to the map-frame seed. It did not prove absolute map-frame position. Stationary AMCL movement may be refinement or bias; first-pose alignment cannot decide which.

The observer was improved from repeatedly starting `ros2 service call` processes to one asynchronous ROS probe. The old sparse cadence prevented reliable continuous-motion classification. The new instrumentation collected scan snapshots, transforms, measurement statistics and no-motion service behavior; instrumentation itself was not an accuracy fix.

### C. Raw LiDAR geometry investigation

- A stationary recording measured raw scan rate near 30 Hz and relayed rate around 8.36 Hz for that specific run.
- Wheel-speed noise standard deviation was about 0.00655 m/s; gyro noise about 0.01005 rad/s in that recording.
- The raw sensor offset investigation led to the later complete CoG/rear-axle correction in Section 6.

### D. First integrated mission: `autonomous_20261029_164139_586206`

Completed exploration, map saving and route preparation, then stopped before driving because the new stage saw `/slam_toolbox` as already running.

- Estimated exploration travel: 34.212 m.
- Lap-watcher duration: 227.07 s.
- Failure: `RESUME STOPPED: /slam_toolbox already running; stop previous stack first`.
- Logs showed the previous SLAM child had already exited cleanly. A stale daemon-backed graph observation was the likely integration explanation.
- Correction: bounded fresh ROS graph discovery and two successful absence checks before handing ownership to localization. An actually running conflicting stack must still block startup.

An ideal kinematic replay of this route completed three modeled laps, but it used ideal pose and no real ROS/Gym sensor/actuator chain. That replay was not a successful integrated mission.

### E. Second integrated mission: `autonomous_20261029_182122_744147`

- Fresh graph handoff passed.
- Route had 374 waypoints and passed nominal body/steering preflight.
- Live scan seed and AMCL readiness passed.
- Vehicle commanded approximately 0.40 m/s briefly, then stopped at roughly 23/374 net progress on covariance recovery.
- Recorded reason: AMCL did not relocalize within 4 s while position standard deviation was around 0.070 m.

The admission gate allowed 0.080 m but the old recovery required 0.064 m. The recovery logic was changed to use admissible bounds plus four distinct fresh observations while stationary, rather than an unachievable hidden tighter bound. Offline replay supported recovery under the stated speed assumption. This repaired a policy inconsistency, not AMCL's physical accuracy.

## 9. Development timeline: 30 October

Further engineering work addressed several problems exposed by integrated tests:

1. **Coverage checks:** Added evidence of an observed closed corridor instead of treating approximate odometric return alone as proof of a completed mapped circuit.
2. **Finer mapping:** Moved engineering mapping toward 0.025 m cells and denser constraints to better represent tight bends.
3. **Reference geometry:** Corrected the complete CoG/rear-axle/laser/body chain and passive truth reference.
4. **Covariance policy:** Used the full planar covariance, checked symmetry/positive semidefiniteness, projected position uncertainty along/across the local route, and retained hard uncertainty bounds.
5. **Local collision prediction:** Changed the runtime guard to predict commanded Ackermann motion in the vehicle frame rather than treating future map-route rectangles as the commanded motion.
6. **Time compensation:** Transformed scans from acquisition time into the current body frame through local odometry.
7. **Speed candidates:** Evaluated descending candidate speeds before publishing, keeping guard velocity no lower than measured/previous commanded speed.
8. **Technical-bend tracking:** Shortened low-speed Pure Pursuit lookahead after an S-bend case showed the old distant target could request steering in the wrong direction for correcting local error.
9. **Stopping buffer:** Replaced an excessive fixed low-speed travel extension with a documented sampling buffer while retaining reaction/braking/body allowances and the hard clearance bound.
10. **Diagnostics:** Saved the scan, transform and evaluated proposals when a guard latched. Diagnostic serialization failures were isolated so they could not prevent a stop.

Historical default values in the reviewed engineering code included: 0.08 m route-normal soft uncertainty, 0.12 m along-route soft uncertainty, 0.16 m major-axis hard uncertainty, 10° soft/20° hard yaw bounds, four fresh stationary recovery observations and a 4 s recovery timeout. The user's later local timeout experiment used 10 s. These are version-specific settings, not universal recommended limits or validated hardware parameters.

The historical local stopping model used at least 0.35 s reaction, braking assumption 0.7 m/s², 0.02 m arc sampling/buffer and 0.07 m hard clearance, with additional local motion/steering allowances. These were explicit simulator assumptions. A parameter existing in code does not establish measured braking performance.

### Recorded run `autonomous_20261030_081638_708195`

- Exploration, map saving, planning and handoff executed.
- Estimated exploration travel: 34.159 m; lap-watcher duration: 226.49 s.
- Saved corridor check reported closed.
- Later execution stopped when estimated CTE exceeded 0.090 m; a logged hold reported about 0.091 m.
- Mission status was `STOPPED`, not completed autonomy.

An archived full ROS/Gym CI run (`40999826215`) also advanced into the route and stopped at estimated CTE about 0.092 m, with logged progress reaching approximately 117/339. It did not complete a lap. This demonstrates why successful offline/controller checks cannot replace the integrated acceptance test.

## 10. 31 October: supervisor demo and mapping-only milestone

The owner narrowed the demonstration to the strongest demonstrated work, avoided a new push, and used a local demo worktree.

The mapping-only option was introduced so the mission can finish after route preparation and report `COMPLETED_MAPPING_AND_PLANNING`. This is an intentional successful stage-limited result, not a completed localization or driving mission.

### A. Baseline run: `autonomous_20261031_124702_440857`

Command supplied by the user:

```zsh
setopt PIPE_FAIL
zsh scripts/run_full_autonomy.zsh --mapping-only \
  2>&1 | tee supervisor_demo_console.log
```

Recorded stages: preflight → exploring → saving map → planning → `COMPLETED_MAPPING_AND_PLANNING`.

| Measurement | Result |
|---|---:|
| Exploration command ceiling | 0.18 m/s |
| Total mission-stage duration | 260.601 s |
| Exploration stage duration | 230.179 s |
| Lap-watcher duration | 227.41 s |
| Estimated odometry travel | 34.287 m |
| Optimized closed-route length | 33.8245 m |
| Route waypoints | 339 |
| Minimum nominal body clearance | Approximately 0.354 m |
| Accepted required peak steering | 21.62° |
| Approximate steering limit | 24.00° |
| Optimized route speed envelope generated | 0.607–0.800 m/s; not executed |

The first route candidate required about 24.85° and was rejected. Smoothing produced an accepted candidate at about 21.62° while preserving nominal clearance.

### B. Faster exploration patch

The user asked for exploration up to 1.0 m/s. The local patch changed three files:

- `tools/mission/full_autonomy.py`: allowed the larger exploration ceiling and passed the matching ceiling to the watchdog.
- `tools/navigation/drive_watchdog.py`: accepted an explicit maximum command speed rather than rejecting every request above the old default.
- `tools/exploration/exploration_controller.py`: limited speed by clearance/braking and turning demand, with an acceleration ramp.

The patch used simulator hypotheses of 0.60 s reaction, 0.70 m/s² braking and 0.60 m/s² lateral acceleration; upward acceleration was limited to about 0.4 m/s². Corner and clearance behavior still reduced speed below the requested ceiling.

The first `git apply --check` failed because local file context differed. Subsequent source checks and a reverse-apply check established that the complete patch was already applied. Do not apply it again simply because the earlier console contains a failed patch command.

Affected offline checks were reported passing in the prior work, but this is not the same as rerunning every base test on the user's complete modified demo tree.

### C. Faster run: `autonomous_20261031_131817_595257`

| Measurement | Result |
|---|---:|
| Exploration command ceiling | 1.0 m/s |
| Highest sampled logged command | Approximately 0.97 m/s |
| Total mission-stage duration | 86.957 s |
| Exploration stage duration | 55.072 s |
| Lap-watcher duration | 52.48 s |
| Estimated odometry travel | 34.679 m |
| Optimized closed-route length | 33.8588 m |
| Route waypoints | 339 |
| Map resolution | 0.025 m/cell |
| Minimum nominal body clearance | Approximately 0.358 m |
| Accepted peak required steering | 23.88° |
| Approximate steering limit | 24.00° |
| Optimized route speed envelope generated | 0.616–0.800 m/s; not executed |
| Mission result | `COMPLETED_MAPPING_AND_PLANNING` |

The unsmoothed route demanded about 28.19° steering and was rejected. Smoothing produced the accepted 23.88° candidate. This leaves only about 0.12° of steering reserve, so the plan is geometrically admissible but should be improved before demanding execution.

Compared with the baseline, total mission time fell by about **66.6%**, from 261 s to 87 s, approximately **3× faster**. The exploration stage itself fell from 230 s to 55 s. These are one run per setting, not a robustness study or proof of equal mapping accuracy.

The faster-run SLAM log contains a dropped-scan / queue-full warning and a `karto::Exception` during deliberate shutdown after map saving. Mapping and planning artifacts were produced, but shutdown was not clean. Do not hide the exception or call the run warning-free.

### Important speed-file distinction

The optimized route CSV contains the approximately 0.616–0.800 m/s generated envelope in the faster run. The separate `new_track_speed_profile.csv` currently contains a 0.400 m/s profile. They are different artifacts with different contents; the generated envelope was not executed during mapping-only mode.

Any presentation or later launcher must state which file it uses and which cap applies. None of these planned values is evidence of actual racing speed.

## 11. Latest localization/execution diagnostics: 31 October

Racing is currently paused, but the following tests supplied important localization evidence that must be retained.

### A. First short saved-map attempt: `race_20261031_133408`

- Startup handoff and four fresh AMCL readiness observations passed.
- Commanded speed cap was about 0.15 m/s.
- Controller recorded net progress 26/339; zero complete laps.
- Stopped after 4 s of unsuccessful confidence recovery.
- Stop context: lateral sigma about 0.013 m, along-route/major sigma about 0.119 m, yaw sigma about 2.8°.
- The recorded local proposal was not blocked: adjusted gap about 0.426 m. This stop was a confidence-recovery failure, not the later collision-guard failure.

The passive audit had 166 paired observations. All-observation relative-position RMS was about 0.0706 m for AMCL and 0.00320 m for EKF; AMCL maximum about 0.153 m. These statistics include stationary samples and use first-pose alignment.

A later local timeout-only change from 4 to 10 s was explored; replay of the earlier data indicated recovery would require longer than 4 s, around 5.6 s under the stated replay conditions. Increasing allowed recovery time changes orchestration policy; it does not repair estimator drift.

### B. Later test: `race_20261031_134058`

The uploaded console and files show:

- Initialization/readiness again passed.
- The vehicle moved longer, at commands up to about 0.15 m/s.
- Recorded net progress reached **104/339**; complete laps remained **0**.
- The final nearest-route index was **108**, while net progress was **104**. These are different fields, not contradictory counts.
- Maximum finite estimated CTE in the supplied controller CSV was approximately **0.0613 m**.
- Final reason: `SAFETY HOLD: no safe local speed: measured/proposed stopping tube obstructed`.

The stop snapshot evaluated several speeds. Every proposal was blocked under the current local stopping/body model:

| Candidate speed | Adjusted local gap | Result |
|---:|---:|---|
| 0.1232 m/s | 0.0386 m | Blocked |
| 0.1047 m/s | 0.0486 m | Blocked |
| 0.0862 m/s | 0.0552 m | Blocked |
| 0.0677 m/s | 0.0651 m | Blocked |
| 0.0500 m/s | 0.06947 m | Blocked against 0.070 m hard margin |

Measured speed was approximately 0.0486 m/s; reaction interval about 0.355 s. The lowest proposal was only slightly below the guard's clearance threshold, but that alone does not prove a false stop. The gap includes model allowances and depends on scan geometry, current position, steering and timing. The proper next step is replay and diagnosis, not subtracting a few millimetres from the guard until it allows motion.

### C. Latest passive localization audit

Source: `localization_truth_audit(5).csv`, 412 paired observations spanning approximately 107.3 s. The rows declare the corrected truth reference as `rear_axle` and retain raw CoG fields.

| Quantity | RMS over all valid paired observations | Maximum absolute value |
|---|---:|---:|
| AMCL relative position error | 0.44021 m | 0.84802 m |
| EKF relative position error | 0.02220 m | 0.04074 m |
| AMCL relative yaw error | 7.036° | 12.179° |
| EKF relative yaw error | 0.326° | 0.706° |

**How to interpret this correctly:**

- RMS summarizes error magnitude over the selected samples; these figures include stopped and moving portions.
- Each estimate is aligned with simulator reference at its first observation. The numbers measure subsequent relative drift, not absolute saved-map registration error.
- AMCL and EKF use different frames/reference alignments. This short test is not a fair general algorithm ranking and does not establish long-term EKF global accuracy.
- Nevertheless, the large AMCL relative drift, compared with the short-test local odometry drift, makes scan-to-map consistency, global correction and map geometry high-priority investigations.
- Small estimated CTE does not contradict this drift: estimated CTE is computed from the estimated pose and therefore is not an independent accuracy measurement.
- Passing the four-observation startup gate did not prevent later drift. Covariance-only admission cannot be treated as a complete localization health test.

The observed symptoms are established. The single root cause is **not** established. Do not claim that a particular AMCL model, timeout or geometry parameter alone explains the whole latest failure.

## 12. What specifically prevents project completion

### Blocker 1 — moving global localization is not validated

We need to know that the reported position is accurate enough for the mapped corridor. The latest audit shows substantial AMCL drift, and there is no independently verified absolute registration result for the saved map and moving car.

Investigate saved-map distortion, scan likelihood/matching, transform consistency, timestamps, odometry calibration/noise, particles and motion-model assumptions. Evaluate these using recorded data before another parameter sweep.

### Blocker 2 — planning margins do not yet include the full estimation problem

The route passes nominal body geometry, but its safety budget must also cover localization error, tracking error, map uncertainty and actuator/dynamic effects. The faster route's steering demand is within about 0.12° of the limit.

A larger nominal body clearance does not by itself establish a safe plan when global localization drifts. We need a route with useful steering reserve and explicitly justified uncertainty margins.

### Blocker 3 — local guard stops need grounded classification

There have been both policy defects and plausible local obstructions. The latest stopping-tube rejection must be checked against the recorded scan and transform. We have not established whether it represents actual wall proximity, pose-induced route misalignment, map error, a conservative prediction, or a combination.

This issue concerns future execution. It should not force us to resume racing while mapping/localization/planning are the chosen milestone.

### Blocker 4 — map quality and closure are not quantified independently

The robot saves recognizable track maps and the planner sees a closed observed corridor. We still need repeated-run boundary consistency, scan-to-map agreement and closure assessment. Approximate odometric return is not enough to certify graph closure or accurate absolute shape.

### Blocker 5 — integration reliability and version reproducibility

Fixes have accumulated across engineering branches and local demo patches. The complete exact laptop state is not represented by one freshly verified clean release in this handoff. Some historical scripts/options differ between checkouts. SLAM shutdown still has an exception in the faster run.

We need a recorded base commit plus local diff, environment/dependency versions, map/geometry checksums and repeatable stage outcomes. A generic statement that “all tests passed” is insufficient without the tested revision and test type.

### Blocker 6 — physical-car commissioning remains separate

Even perfect simulation would not validate real steering calibration, VESC speed conversion, sensor mounting, timing, electrical behavior, slip or braking. The physical full stack has not been commissioned through an autonomous end-to-end acceptance test in the available evidence.

## 13. Completed improvements and their actual value

| Improvement | Why we needed it | What it establishes / does not establish |
|---|---|---|
| Isolated AMCL initialization | Initial-pose topic could reset Gym | Correct seeding behavior; not accurate global pose |
| Measured raw laser transform | Assumed mount disagreed with raw scan geometry | Better sensor geometry; not a complete localization fix |
| Composed rear-axle geometry | CoG and rear-axle references were mixed | Consistent physical reference for mapping/guard/audit |
| Corrected passive audit reference | Turning created artificial drift between different body points | More meaningful error measurement, not estimator improvement by itself |
| Fresh ROS graph handoff | Stale discovery blocked localization after SLAM exited | Cleaner stage ownership; not completed mission performance |
| Full covariance projection/validation | World-axis scalar gate missed directional structure | Better uncertainty interpretation; calibration still required |
| Fresh stationary recovery observations | Hidden tighter recovery threshold stranded valid states | More consistent recovery policy; not AMCL accuracy repair |
| Body/steering route audit | Centreline geometry alone missed body/curvature constraints | Rejects nominally infeasible candidates |
| Finer map / smoothing iterations | Tight bends produced excessive curvature | Better candidates; margins/repeatability remain open |
| Commanded-motion stopping tube | Fixed map-route sweep was not the actual command | More relevant local prediction under explicit assumptions |
| Candidate-speed evaluation | Reject-only logic did not plan around local clearance | Can reduce command demand before a guard hold |
| Watchdog and topic ownership | Multiple/stale commands could undermine stopping | A separate command layer; hardware safety still separate |
| Mission bundles and stop snapshots | Console snippets were insufficient for diagnosis | Reproducible evidence and offline replay |
| Mapping-only exit | Demo should show proven stages cleanly | Honest stage-limited completion |
| Adaptive 1 m/s exploration ceiling | Very slow exploration limited demo efficiency | One substantially faster completed mapping/planning run |

## 14. Tests: what passed and what those tests mean

Work across the engineering revisions included:

- Pure-Python tests for route progress, lookahead, speed policies and confidence recovery.
- Covariance validity/projection and sparse/timestamped audit analysis tests.
- Full-body footprint, swept motion, route rejection and optimizer CLI tests.
- Initialization and no-motion probe tests.
- ROS graph handoff/ownership checks in ROS 2 Humble with CycloneDDS.
- A model-contract check against the actual pinned Gym geometry.
- Ideal-pose kinematic route replay.
- Full ROS/Gym mission runs, including failures whose logs were retained.
- Local checks of the faster-exploration patch.

Earlier workflow stages reported test totals such as 40, 105 and 110 as suites expanded; targeted faster-exploration checks were also reported. These counts belong to their respective revisions. **No new full regression suite or live ROS/Gym run was executed while writing this log, and this log does not certify the user's current modified demo tree.**

The distinction is essential:

1. Offline tests validate functions and selected modeled cases.
2. ROS graph tests validate discovery/ownership behavior.
3. Ideal bicycle replay validates modeled tracking under ideal assumptions.
4. Full Gym/ROS tests exercise the integrated stack.
5. Physical tests validate actual sensors, actuators and vehicle response.

Passing an earlier level cannot substitute for a later level. The full mission has not passed merely because a route replay or CI unit-test job passed.

## 15. Current code/file guide

Paths below are relative to the simulation repository. Files and options vary by checkout; confirm them locally before using a command.

| File / area | Role |
|---|---|
| `scripts/sim_env.zsh` | Select ROS/Python environment and domain |
| `scripts/run_sim.zsh` | Start simulator stack |
| `scripts/run_full_autonomy.zsh` | Environment wrapper for mission orchestration |
| `tools/mission/full_autonomy.py` | Stage supervisor, subprocess ownership, status and packaging |
| `tools/mission/mission_first_lap.py` | First-lap mapping workflow |
| `tools/mission/mission_slam.launch.py` | SLAM launch/configuration |
| `tools/mission/mission_explore.py` | Exploration launch/integration |
| `tools/exploration/exploration_controller.py` | Reactive LiDAR exploration and local speed adaptation |
| `tools/fusion/sim_vesc_imu.py` | Synthetic motion-sensor adapter |
| `config/fusion_sim_sensors_unbiased.yaml` | Synthetic measurement configuration |
| `config/slam_sim.yaml` | Mapping parameters |
| `config/sim_laser_geometry.yaml` | Simulator sensor/reference geometry contract |
| `tools/sim_geometry.py` | Shared geometry and model conventions |
| `tools/localization/fusion_scan_relay.py` | Scan relay, rate and frame handling |
| `tools/localization/initialize_amcl_pose.py` | Dedicated one-shot AMCL initialization |
| `tools/localization/stationary_localization_probe.py` | Stationary instrumentation / no-motion updates |
| `tools/localization/wait_amcl_ready.py` | Fresh-pose readiness gate |
| `tools/mission/map_scan_seed.py` | Live scan matching for pose hypothesis |
| `tools/mission/mission_amcl.launch.py` | Saved-map AMCL launch |
| `config/fusion_amcl.yaml` | Base AMCL/localization configuration |
| `tools/mission/ros_graph.py` | Fresh graph discovery and absence checks |
| `tools/mission/map_coverage.py` | Observed-corridor/map coverage checks |
| `tools/planning/extract_robust_centerline.py` | Centreline extraction |
| `tools/planning/prepare_vehicle_route.py` | Candidate preparation and acceptance attempts |
| `tools/planning/optimize_vehicle_route.py` | Route optimization/smoothing with constraints |
| `tools/planning/vehicle_footprint_audit.py` | Nominal full-body/steering audit |
| `tools/navigation/drive_watchdog.py` | Command limits, freshness and final command ownership |
| `tools/navigation/pose_uncertainty.py` | Covariance validation/projection |
| `tools/navigation/confidence_policy.py` | Pause/recovery/fault policy |
| `tools/navigation/trajectory_safety.py` | Local predicted body-motion collision geometry |
| `tools/navigation/racing_policy.py` | Historical estimated-route tracking policy |
| `tools/navigation/profiled_amcl_drive.py` | Historical ROS controller integration, currently out of scope |
| `tools/mission/run_profiled_saved_map.py` | Saved-map preparation/localization-only observation or historical drive launch |
| `tools/localization/sim_localization_audit.py` | Passive paired reference/estimate recorder |
| `tools/localization/summarize_sim_localization_audit.py` | Analyze already-recorded controller/audit data |
| `tests/` and `.github/workflows/` | Offline, graph, model and integration checks |
| `docs/` and `docs/evidence/` | Reports, previous findings and selected reproducibility evidence |

### Generated mission outputs

- `mission_status.json`: stage history and result.
- `commands.jsonl`: launched process commands/configuration evidence, not a complete high-rate vehicle-command recording.
- `new_track.yaml`, `new_track.pgm`: map metadata and occupancy image.
- `lap_pose.json`: estimated end pose, odometric return and coverage observations.
- `laser_geometry.json`: geometry compatibility metadata.
- `new_track_centerline.csv`: extracted centreline.
- `new_track_body_optimized.csv`: final body-audited route candidate, semicolon-separated route fields.
- `new_track_speed_profile.csv`: separate profile artifact; verify it against the selected route/cap.
- `new_track_preview.png`: centreline/map illustration.
- `route_preparation.json`: attempted candidates, rejected/accepted audits.
- Mapping/localization handoff JSON: process/discovery checks.
- `live_scan_seed.json`, `amcl_initial_pose.json`: pose-hypothesis and initialization evidence.
- `recovered_map_amcl.yaml`: exact per-run localization parameters.
- `localization_ready.json`: readiness outcome.
- `localization_truth_audit.csv`: passive paired reference/estimate observations.
- Historical `estimated_drive.csv`: commands, estimated tracking, covariance and guard metrics.
- `guard_stop_context.json`: recorded scan/transform/proposals at a stop.
- Node `.log` files and the mission ZIP: failure diagnosis and preservation.

The per-run YAML and metadata are often more useful than a screenshot because they preserve what actually ran.

## 16. Next work: ordered, concrete and limited to the chosen scope

### Priority 0 — preserve a reproducible baseline

Owner: Zaid/integration.

- Keep both completed 31 October mapping mission directories and ZIPs.
- Keep the latest AMCL/guard/CSV files, not only the console screenshot.
- Save the demo base commit, local diff and untracked evidence inventory outside the repository or in a deliberate backup folder.
- Record environment, geometry version, map hash and route hash with every future run.
- Do not reset the demo worktree or reapply the already-present speed patch.

Acceptance: another engineer can identify the exact map, route, configuration and local source changes that produced each result.

### Priority 1 — diagnose localization from existing recordings

Owner: Moatasem, with integration support.

1. Separate moving and stationary portions and check time pairing/lag.
2. Verify all comparisons use the rear axle and the same timestamp convention.
3. Replay available scan snapshots/stop scans against the exact saved map and transform.
4. Inspect scan endpoints near walls and compare AMCL motion with EKF-relative motion and passive reference motion.
5. Test whether scan-to-map fit worsens while AMCL still reports low uncertainty.
6. Investigate map distortion, beam model, particle behavior and odometry/noise assumptions based on that evidence.
7. Check AMCL motion-model assumptions for Ackermann motion as a hypothesis; do not replace models blindly.

Acceptance: document a supported explanation or narrow set of explanations for drift, with plots and parameter provenance. A lower covariance or longer timeout alone is not acceptance.

### Priority 2 — validate mapping quality and clean shutdown

Owner: Zaid/integration, with localization support.

- Resolve the queue-full/dropped-scan symptom and the shutdown exception without hiding failures.
- Repeat from multiple initial poses; include a simpler track and the technical track.
- Compare generated boundaries and scan-to-map residuals after independent offline alignment.
- Distinguish observed corridor closure, estimated return and SLAM graph closure in the status output.
- Measure whether the faster exploration setting preserves useful map quality.

Proposed acceptance: at least three documented mapping runs on the main track plus a second track, with saved outputs, clean shutdown and explicit map-quality checks. This is a proposed next gate, not a result already achieved.

### Priority 3 — improve planning reserve

Owner: Ahmad.

- Keep unknown cells blocked and preserve full-body checks between samples.
- Reduce peak steering demand enough to leave a useful measured reserve rather than merely passing by 0.12°.
- Ensure smoothing does not create wall/unknown-space incursions or a bad closure seam.
- Define one authoritative planned profile artifact, and clearly log later command caps.
- Add localization/map uncertainty to the clearance budget once those errors are measured.
- Return explicit rejection reasons when no candidate meets the budget.

Acceptance: route audit reports length, continuity, minimum body clearance, peak curvature/steering and the uncertainty assumptions. It must be understandable why the route was accepted.

### Priority 4 — validate localization in controlled motion

Owner: Moatasem + Zaid.

- After recorded-data corrections, first confirm stationary initialization and scan/map agreement.
- Then use a short controlled low-speed motion test with full recording, rather than attempting a complete race.
- Measure position/yaw error, along/across-route components, scan-to-map residuals and covariance consistency.
- Keep simulator truth evaluation-only.

Acceptance: errors remain within a predeclared budget derived from available clearance and intended motion. The exact numeric budget should be selected after map and sensor consistency are established; this log does not invent a universal safe error threshold.

### Priority 5 — combine mapping, localization and planning without racing

Owner: integration/team.

- Produce an explicit workflow whose final result is validated map + accepted localization + accepted plan.
- Preserve single TF/actuation ownership through stage changes.
- Make success stage-limited and explicit; do not reuse a racing-success name for this milestone.
- Package the map, route, localization report and logs together.

Acceptance: repeatable three-stage completion, accurate localization evidence, accepted planning audit, and no autonomous route-following stage started.

### Later milestone — physical-car commissioning

Only after the relevant simulation stage is validated, bring the equivalent module onto the physical car with measured hardware geometry and a controlled test plan. The owner must receive commands and explanations before main-repository changes. Path execution/racing should return to the plan only after these foundations are reliable.

## 17. Physical car: what is known and what still needs measurement

The following comes from the owner's hardware reports, not a new inspection:

| Hardware / subsystem | Last reported state |
|---|---|
| Jetson Orin Nano 8 GB, Ubuntu 22.04, ROS 2 Humble | Working |
| LakiBeam L1 LiDAR, nominal 30 Hz / 25 m | Working |
| BMI160 IMU | Working at driver/device level |
| VESC and motor | Working |
| Steering servo | Partial |
| Manual/remote driving | Working |
| SLAM Toolbox + RF2O | Partial |
| Complete ROS vehicle stack | Partial |
| Autonomous physical mapping/localization/route execution | Not demonstrated in the evidence |

Previously discussed component/specification candidates include VESC MINI 6.7 Pro, DYS D3542 1450 KV motor and FT5330M steering servo. Approximate wheelbase is 0.33 m. Earlier notes contain differing wheel-diameter estimates, including approximately 65 mm physical wheels versus a simulator adapter assumption of 0.1016 m. Gear ratio, wheel dimensions, steering angles and all conversion constants need direct measurement on the actual car. Do not copy simulator geometry or ERPM conversions into hardware without calibration.

Physical work still required:

1. Document wiring, power supplies, grounds, servo supply and mounting.
2. Verify remote takeover and a working stop mechanism independently of autonomy.
3. Measure chassis/body dimensions and LiDAR/IMU transforms relative to the chosen body reference.
4. Calibrate steering centre, range, sign, angle conversion, response and rate limit.
5. Calibrate VESC-to-wheel-speed scale, direction and drivetrain ratio.
6. Measure braking/deceleration and command/communication delay.
7. Calibrate IMU axes, bias, units, timestamps and covariance.
8. Verify real LiDAR frames, scan timing/rate, invalid returns and alignment.
9. Validate physical local odometry before relying on it for mapping.
10. Start with supervised slow mapping and offline planning, then stationary/controlled-motion saved-map localization.

Simulation's 1.0 m/s exploration ceiling and 0.7 m/s² braking assumption are not authorized physical-car settings.

## 18. Reference projects and research direction

The project owner asked to learn from Penn/F1TENTH, ETH and other strong university teams, and improve step by step.

Reference repositories recorded in the earlier engineering review:

- F1TENTH ROS vehicle system / bringup: https://github.com/f1tenth/f1tenth_system
- F1TENTH particle-filter localization: https://github.com/f1tenth/particle_filter
- F1TENTH Gym ROS bridge: https://github.com/f1tenth/f1tenth_gym_ros
- University of Waterloo workspace example: https://github.com/CL2-UWaterloo/f1tenth_ws

Useful engineering lessons were single command ownership, priority/manual override, motion/sensor-model consistency, speed-aware tracking, localization likelihood and independent stopping behavior.

This log does not claim an exhaustive ETH/university-code audit, a benchmark win, or a system better than those teams. Establishing that would require a defined task, matched tracks/sensors/limits, reproducible metrics and repeated trials. Advanced control should follow validated estimation and planning rather than mask their errors.

## 19. Safe reference commands for the current stage-limited work

These are reference commands for the user's laptop, not commands executed while creating this log. They do not push or modify the physical repository.

### Inspect the demo checkout

```zsh
cd ~/f1tenth_supervisor_demo
source ~/f1tenth_sim/scripts/sim_env.zsh
git status --short --branch
git log -1 --oneline
git --no-pager diff --stat
python3 tools/mission/full_autonomy.py --help
ros2 node list --no-daemon --spin-time 2
```

Confirm that `--mapping-only` exists in this checkout before launching. A different engineering checkout may not have the same local option. If Git opens a pager showing `(END)`, press `q` to return to the shell.

### Reproduce the demonstrated mapping-only workflow

After confirming no conflicting stack is running:

```zsh
cd ~/f1tenth_supervisor_demo
setopt PIPE_FAIL
zsh scripts/run_full_autonomy.zsh --mapping-only \
  2>&1 | tee supervisor_demo_console.log
```

The already-tested local faster-exploration version additionally supports `--explore-speed 1.0`. Confirm the patch is present before using it. Treat it as a simulator command ceiling and retain all existing guards.

### Stationary saved-map localization only

The standalone observer needs the normal simulator already running. Use the saved mission directory explicitly so that it does not fall back to an older map/route path:

```zsh
cd ~/f1tenth_supervisor_demo
python3 -u tools/mission/run_profiled_saved_map.py \
  --mission-dir results/missions/autonomous_20261031_131817_595257 \
  --localization-only --observe-seconds 45
```

This option is intended not to start or arm the autonomous route controller. Verify the chosen map has the matching geometry metadata. Do not run a second conflicting watchdog/drive stack beside it. The launcher may label its result subdirectory `race_*` even in observation mode; the name alone does not mean driving occurred.

### Analyze existing data without driving

```zsh
python3 tools/localization/summarize_sim_localization_audit.py \
  /path/to/localization_truth_audit.csv \
  --controller-csv /path/to/estimated_drive.csv
```

Replace the paths with the exact recorded pair. The summary script applies motion/timing filters; its moving-only statistics can differ from the all-observation statistics in Section 11.

No command in this section claims the blockers are fixed. Do not add `--autonomous` to the standalone saved-map observer when the intended task is stationary observation only.

## 20. Claims we can and cannot make to the supervisor

**Supported statements:**

- We implemented and connected autonomous exploration, SLAM map saving and route generation.
- Recorded mapping-only runs completed on the custom simulation track.
- A faster exploration setting reduced one recorded total workflow from 261 s to 87 s.
- We generate and check full-body/steering-feasible route candidates in the saved map.
- We implemented sensor fusion, automatic AMCL initialization, readiness checks and passive error measurement.
- We found and corrected significant frame, initialization and process-handoff issues.
- We know that moving localization still requires improvement, and have recorded evidence for diagnosis.

**Unsupported statements:**

- The complete unknown-map racing mission is finished.
- The latest AMCL localization is accurate because it reports low covariance.
- A 1 m/s ceiling means the car drove at exactly 1 m/s throughout exploration.
- An accepted nominal route proves collision-free execution with localization error.
- Approximate return to the start proves accurate SLAM loop closure.
- The physical car is ready because the simulator runs.
- We outperform ETH/Penn or all university teams.

## 21. Deliverables and evidence register

### Main project deliverables already produced

- Earlier engineering development logs for 27 October.
- Technical report for 28 October.
- Comprehensive engineering handoff for 29 October.
- Engineering runbooks and diagnostic findings for AMCL seed behavior, confidence, geometry and handoff.
- Completed mapping mission bundles and result images on 31 October.
- Local faster-exploration patch.
- Editable 11-slide supervisor presentation with speaker notes: **F1TENTH_Project_Progress.pptx**.

Presentation: https://chatgpt.com/library?file_id=libfile_ea08b2bb918c81919f43c358fa61c9a5&account_id=personal

Earlier detailed handoff: https://chatgpt.com/library?file_id=libfile_9921a5854e5881918bd186e4912f002b&account_id=personal

The older handoff is historical. Its earlier geometry assumptions and unresolved items must be interpreted together with the corrections in this log.

### Key mission directories on the user's laptop

Under `~/f1tenth_sim_seed_check/results/missions/`:

- `profiled_20261029_153559`
- `autonomous_20261029_164139_586206`
- `autonomous_20261029_182122_744147`
- `autonomous_20261030_081638_708195`

Under `~/f1tenth_supervisor_demo/results/missions/`:

- `autonomous_20261031_124702_440857`
- `autonomous_20261031_131817_595257`
- Faster mission's saved-map attempts: `race_20261031_133408` and `race_20261031_134058`.

The latest uploaded file suffixes such as `(5)` are chat-upload filenames. The original run directories normally use unsuffixed names. Preserve the run association when renaming.

### Latest key source files

| File supplied | Used for |
|---|---|
| `Pasted text(20261031-104343).txt` | Later saved-map attempt console and final reason |
| `localization_ready.json` | Four fresh AMCL readiness observations passed |
| `localization_handoff.json` | Previous-stack absence / ownership evidence |
| `live_scan_seed(3).json` | Live scan pose hypothesis |
| `amcl_initial_pose(1).json` | AMCL initialization parameters |
| `recovered_map_amcl(4).yaml` | Exact run localization configuration |
| `localization_truth_audit(5).csv` | Latest 412-observation passive audit |
| `estimated_drive(3).csv` | Latest controller record; net progress 104, nearest index 108, zero laps |
| `autonomous_drive(4).log` | Controller messages and stopping-tube hold |
| `guard_stop_context.json` | Five blocked speed proposals and scan/transform data |
| `amcl(5).log`, `ekf(4).log`, `scan_relay(3).log`, `sensor_emulator(2).log` | Node-level diagnosis |

### How the conclusions were obtained

This log combines visible owner instructions and terminal outputs, the current contents of the older handoff, local copies of supplied mission artifacts, engineering source/docs and recalculated CSV statistics. The run times come from stage event timestamps; route length includes the closing segment; RMS/max errors come from recorded valid CSV observations. It is not a new live test or fresh hardware inspection.

## 22. Handoff instructions for the next engineer or assistant

1. Read Sections 1, 6, 11, 12 and 16 first.
2. Preserve the current user instruction: mapping, localization and planning; no racing for now.
3. Do not mistake this documentation for permission to push, merge or modify the physical-car main branch.
4. Do not repeat a mapping lap merely to obtain data already present in the saved bundles.
5. Do not confuse the preloaded simulator map with the generated SLAM map.
6. Use the rear-axle convention consistently; do not revert fusion laser x to the raw CoG value.
7. Do not combine statistics from different geometry conventions, maps or alignment methods without labeling them.
8. Diagnose confidence timeouts, true drift, estimated CTE and local obstruction separately.
9. Retain fault guards and explain any proposed policy change with recorded evidence and assumptions.
10. Complete the stage-specific acceptance gates before making a new claim of end-to-end completion.

**Current handoff conclusion:** Mapping and nominal planning are demonstrated on the custom simulator track, including one faster exploration run. AMCL initialization is demonstrated. Reliable moving localization, quantified map/plan margins, a repeatable combined three-stage workflow and physical commissioning remain unfinished. We have evidence and instrumentation to investigate these issues; we do not yet have evidence that the complete original project objective is achieved.


## Appendix — evidence fingerprints

SHA-256 fingerprints of the analyzed local evidence copies. These identify the exact artifacts underlying this log; upload filename suffixes are not version-control revisions.

| Evidence | SHA-256 |
|---|---|
| Faster mapping stage result | `bdb1b1ceaac23732a97f3479f10d634fef0a0cbd0900e2e89cff5473945f6b4c` |
| Faster map | `35440181e444a5f1b196075322ceb0b82c699746852dcfc2a43051d46e67f49f` |
| Faster map metadata | `a21d409d4bf65bd35232f2dab8ad80e0dae932f2cf01681b16572278eb14b0e0` |
| Optimized route | `6c2d1260930bffe58eface703a2c61869eb76cbd1faecc3454fb583ead20eaaf` |
| Route acceptance | `99888b3d1d0efe71e81ffe7d9962317efaaf9cebc7e713ad6c694de4bf308c0a` |
| Latest readiness | `de3c621f136562069376d4756c22a2e07fc61bc1a662707afd50c7b32a9f05f2` |
| Latest passive audit | `1267da69ca034b813d15e1c04a98b9430e80361dce2e7ad62c56dfc3772cd15f` |
| Latest controller data | `c2556393bd247229fdd025e6487f0cddc3f23f20d5c26572b33baba28c40b8dc` |
| Latest stop snapshot | `bbce3d628f4baf66001e6394639cdc79220a3993928b3de46a12f8c850eb624f` |
