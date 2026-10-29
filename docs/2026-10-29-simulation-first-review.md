# Simulation-first engineering review and first change

Reviewed 29 October 2026. This is a representative source-code comparison and
the first diagnostic step, not a claim to have evaluated every university or
completed the autonomy system.

Follow-up: the missing run recordings have since been supplied. See
[recorded localization findings and the startup fix](2026-10-29-recorded-localization-findings.md)
for the measured results, explicit-covariance initialization and next stationary
observation. Statements below about unavailable moving CSVs describe the first
review, before that upload.

## Exact starting point

| Source | Revision / evidence |
| --- | --- |
| UJ simulation main | `6e676622c49f5fb3e6c399e74c265881efb147ea` |
| UJ experimental branch / draft PR #1 | `6c014484d35ddcf2abdec889386a43ea1fbaa5d9` |
| UJ physical repository | `4ed65ea683a641efccf8273481b7e5131d02965b` |
| Latest integrated run | `profiled_20261029_142531`, described in the supplied handoff; full moving CSV unavailable in this checkout |

The known-map estimated-pose lap and the newly explored-map lap are different
experiments. The latest handoff records 41/377 progress on the latter before a
localization pause. It does not establish actual moving localization accuracy.

The uploaded `uj_technical_slam_01(2).pgm` is byte-identical to
`results/mapping/uj_technical_slam_01.pgm` (287 x 183 pixels). The newly explored
map is `results/missions/20261028_112407/new_track.pgm` (225 x 296 pixels).
Use each map with its own YAML and map-frame route.

## What the reference code teaches us

| Reference and inspected source | Relevant implementation | UJ decision |
| --- | --- | --- |
| [ETH ForzaETH, ROS 2 Humble](https://github.com/ForzaETH/race_stack/tree/948f65d82389a1e61e2e8af86988bb4b98d13244), `controller/controller/pp.py`, `controller/README.md`, `state_estimation/state_estimation/ekf_node/models.py`, global planner node | PP uses velocity-dependent lookahead and adjustments for delay, acceleration and tracking error; controller manager also supports MAP and Follow the Gap. Estimation source provides several vehicle models. Global planner extracts a line and boundaries before trajectory optimization. | Keep PP as the interpretable baseline. Separate estimation, route planning, behavior and control. Compare MAP/MPC only after measured state and actuator models are trustworthy. |
| [Official F1TENTH localization course](https://f1tenth-coursekit.readthedocs.io/en/latest/lectures/ModuleC/lecture08.html) and [Humble particle-filter implementation](https://github.com/f1tenth/particle_filter/blob/672ea4343dc30c981b08338103cb05b794eec606/particle_filter/particle_filter.py) | The official teaching material covers particle filters and AMCL. The F1TENTH filter is MIT-RACECAR-derived MCL with configurable odometry, ray casting and RangeLibc. Its inspected TF publication directly connects map and laser. | AMCL is already a particle filter. A faster ray-casting filter is an alternative to benchmark, not an additional global TF owner. Adapt frames and timestamps explicitly if replacing AMCL. These community sources are not proof of Penn's current competition configuration. |
| [University of Waterloo CL2](https://github.com/CL2-UWaterloo/f1tenth_ws/tree/c20cf63d04b9841ffdb6b2f963bd737d78074136), `src/pure_pursuit/src/pure_pursuit.cpp`, `src/safety_node/scripts/safety_node.py` | PP computes speed-dependent lookahead and reads waypoint velocities. The safety example uses measured odometry speed and LiDAR TTC. Its simulation PP defaults to `/ego_racecar/odom`. | Borrow the separation and geometric ideas; route our autonomous controller through estimated pose. A simulator-truth baseline is a distinct experiment. The safety example's competing drive publishers are not a substitute for an authoritative command arbiter. |
| [TUM trajectory optimizer](https://github.com/TUMFTM/global_racetrajectory_optimization/tree/a9995e2f5407f22eb7fb9dceac2b71a35276bb41), `Readme.md`, `main_globaltraj.py` | Supports shortest-path, minimum-curvature and minimum-time objectives, with vehicle/friction/acceleration information for speed planning. | A centerline is a useful baseline, not generally the fastest route. Establish body clearance and model validity before optimizing lap time. This is a planning library, not a complete F1TENTH stack. |

ETH's Humble README explicitly says its paper's results used ROS 1 and that the
ROS 2 port is less tested, with some functionality missing. Folder names or a
published winner's lap time are not evidence that a port or our adaptation works.

We can aim to improve a specific, measurable property relative to these
baselines. A fair claim requires the same map, dynamics, sensor errors, starting
conditions, pose source, computing budget and repeated trials. No superiority
claim is established by this review.

## Understand the current pipeline

1. **Sensor emulator:** creates wheel-speed and gyro measurements from simulator
   motion, with configured noise. This is the simulated hardware boundary.
2. **EKF:** integrates measured forward speed and yaw rate into locally smooth
   odometry. It does not obtain an absolute map position from those inputs.
3. **Scan relay:** rate-limits the scan and changes its frame label. Relabeling is
   valid only if the stated laser extrinsic matches the simulated rays; it does
   not geometrically transform the scan.
4. **SLAM:** builds a map and supplies global correction while mapping.
5. **AMCL:** localizes in the saved map while racing. SLAM and AMCL must not
   simultaneously own the same map-to-odometry transform.
6. **Route and controller:** generate a steerable path with speeds, then use the
   estimated pose to compute steering and speed commands.
7. **Independent evaluation:** observes simulator truth to measure errors. Truth
   must not feed online pose initialization, steering, or trajectory correction.

The physical repository currently contains sensor/teleop bringup rather than a
validated deployment of this pipeline. For example, the BMI160 node is commented
out in `sensors.launch.py`; lidar TF defaults to x=0.195 m while simulation uses
0.275 m. These are distinct mounts to verify, not values to copy between cars.

## First problem: can we trust the localization diagnosis?

The controller's CTE uses AMCL/TF. If that pose is wrong, a small reported CTE can
coexist with real deviation. The passive recorder is therefore useful, but its
old summary mixed stationary preparation samples with driving and allowed pairs
up to 0.18 s apart. At 0.4 m/s, the vehicle travels 0.072 m in that interval; this
is an illustrative timing scale, not a measured error in the latest run.

The first change is restricted to `summarize_sim_localization_audit.py` and its
tests. It reads existing files and has no ROS or actuation dependency:

- Classify interval endpoints using displacement of recorded true XY, not speed
  commands. Report moving and stationary error statistics separately.
- Default to a 0.05 s pairing filter and a 0.5 s maximum motion interval; show all
  excluded/unclassified counts. These are adjustable analysis choices, not new
  driving limits or a safety acceptance threshold.
- Do not bridge late/malformed samples, duplicate timestamps or clock resets.
- Reconstruct the recorder's fixed first-pose alignment and report signed error
  in true vehicle forward/left axes. These are **not** route tangent/normal axes.
- Preserve original unfiltered statistics and compatibility with old logs.
- Distinguish maximum waypoint index from accumulated progress and completed
  laps. In the archived run, index 371 occurs at startup, then wraps through 376
  to zero; a high index does not demonstrate a completed circuit.
- Print the first STOP after driving and make stationary-only evidence explicit.

For error fields, `rms` is root-mean-square error. For speed, lag or standard
deviation fields, it is simply the root mean square of that measured quantity.
The signed error fields also report a signed median and maximum; RMS gives their
unsigned scale.

Important limits: first-pose alignment removes the initial registration error.
The recorder did not retain exact paired truth timestamps or full XY covariance.
Consequently this analyzer cannot certify absolute localization, covariance
calibration, true route-normal error, wall clearance or collision-free laps.
Motion speed is an interval average using recorded AMCL timestamps; transitions
and sparse scans can still bias it. A filtered-out initial alignment remains the
anchor and produces a warning rather than being silently replaced.

## Read the existing run before any new driving

From a checkout containing this change, run:

```bash
RUN="$HOME/f1tenth_sim_autonomy_audit/results/missions/profiled_20261029_142531"
python3 tools/localization/summarize_sim_localization_audit.py \
  "$RUN/localization_truth_audit.csv" \
  --controller-csv "$RUN/estimated_drive.csv"
```

Supply the two original CSVs plus `amcl.log`, `ekf.log`,
`recovered_map_amcl.yaml` and `live_scan_seed.json` for diagnosis. These latest
moving files are not committed in the reviewed GitHub tree. Existing logs should
be read before scheduling another exploration or driving experiment.

Decision after inspecting that evidence:

| Observation | Next investigation |
| --- | --- |
| True relative error grows while estimated CTE stays small | Scan geometry/timing, odometry model, map consistency and AMCL measurement model |
| True lateral error remains small but covariance grows longitudinally | Route-relative covariance projection, ambiguous geometry and a calibrated clearance budget |
| Pose/scan timestamps lag or jump | Clock source, QoS, callback timing and TF availability at measurement time |
| Pose remains accurate but predicted body clearance becomes tight | Actual swept trajectory, footprint, braking and route clearance |

These causes can coexist. Raising a threshold alone does not distinguish them.

## Remaining simulation milestones

| Order | Concrete result | Evidence needed |
| --- | --- | --- |
| 1 | Reproducible localization diagnosis | Moving truth-relative errors, timing, frame checks; independently grounded registration for absolute-error claims |
| 2 | Authoritative stopping and clearance policy | Command arbiter, measured-speed braking, watchdog, full-body/unknown-space checks; controller-crash and stale-sensor tests |
| 3 | Saved-map estimated-pose driving | One full lap, then at least three consecutive laps, with real progress and collisions checked independently |
| 4 | Repeatability | Multiple starts, seeds, corner radii and widths; gyro bias, wheel-scale errors, delayed/dropped scans and wrong pose hypotheses |
| 5 | Complete unknown-map mission | Exploration, verified loop closure, map validation, feasible route/speed profile and repeated estimated-pose laps in one logged sequence |
| 6 | Performance comparison | Matched PP baseline versus justified MAP/Stanley/MPC alternatives; lap time, tracking error, stop/failure rate and CPU time |

Define and record the supported scenario envelope; no finite simulator test can
prove the car handles every possible situation. Hardware commissioning follows
the simulation milestones and requires measured actuator and sensor parameters.

## Validation performed for this change

- The original offline unittest suite passed before editing (13 tests).
- The expanded suite passed (21 tests), including moving/stationary separation,
  rotated coordinate axes, late data, gaps, timestamp resets, invalid parameters
  and old/new CSV layouts.
- The standalone trajectory collision check passed separately; it is a plain
  test function and is not collected by unittest discovery.
- An actual archived 1,667-row controller CSV was read successfully: zero laps,
  first STOP after DRIVE due to estimated cross-track error. This is historical
  controller evidence, not the latest truth audit.
- This editing environment has neither ROS 2/rclpy nor F1TENTH Gym installed.
  No ROS graph, new exploration or simulated lap was executed here.

After each next change, explain the input/output contract, governing equation,
parameter meaning and validation result. Keep changes small enough to attribute
their effects and defend the design in the graduation presentation.
