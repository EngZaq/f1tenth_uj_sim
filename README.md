# UJ F1TENTH — Autonomous Mapping, Localization & Planning Simulator

**University of Jordan | Mechatronics graduation project | ROS 2 Humble / Ubuntu 22.04 / F1TENTH Gym**  
**Last engineering evidence review:** 1 November 2026  
**Repository:** https://github.com/EngZaq/f1tenth_uj_sim  
**Physical-car repository:** https://github.com/EngZaq/f1tenth_uj

> **Current verified result:** An unknown-track, **mapping-only** mission explored a technical closed circuit, saved a new SLAM occupancy map, verified the observed corridor's topology, and generated a nominal body-audited trajectory and speed profile. The most accurate independently evaluated mapping baseline uses `do_loop_closing: false` as a **temporary containment measure**. It does **not** prove reliable AMCL autonomous laps, physical vehicle safety, or a correct loop-closure algorithm.

## 1. Start here

Read [the full engineering history and team handoff](docs/F1TENTH_SIMULATION_MASTER_LOG_2026-11-01.md) for chronological experiments, performance measurements, mission archive IDs, file-by-file guide, map geometry, failure analysis and hardware handoff. Older reports below that document October failures are **historical**, not the current release verdict.

### Current capability ledger

| Component | Evidence-supported state | Limitation |
|---|---|---|
| Unknown-track exploration | Completed on simulator technical loop | Exploratory policy, speeds and recovery are simulator-specific |
| VESC/IMU-like sensor fusion | Running EKF from synthetic wheel-speed + yaw-rate measurements | The emulator obtains vehicle twist from simulator truth; **not hardware-validated** |
| SLAM Toolbox mapping | Closed observed corridor and saved 2.5 cm occupancy grid | Loop closing OFF for validated mapping baseline; physical map accuracy untested |
| Geometry and route planning | Centerline, route optimization, body/steering audit, speed envelope produced | Nominal geometric clearance is not a tracking or collision guarantee |
| AMCL localization | Integrated and exercised in past simulator tests | Moving global localization and multiple fully estimated-pose laps **not validated** |
| Pure Pursuit | Five repeated laps in early oracle-pose experiment | **Historical oracle-pose demo**, not proof of estimated-pose autonomy |
| Mission controller / watchdog | Mapping-only automatic stage transitions; single `/drive` owner and stop guards | Simulator-only; physical independent E-stop must be separately engineered |

## 2. Verified latest result and why loop closure is temporarily OFF

On **1 November 2026**, two matched simulation runs were checked against the original technical track *offline* (the reference track never entered navigation). The live ROS parameter was checked in each run.

| Run ID | Runtime loop closing | Largest adjacent `slam_map → fusion_odom` translation / yaw step | Final map wall RMSE, after rigid alignment | Outcome |
|---|---|---|---|---|
| `autonomous_20261101_164525_862826` | `false` (verified) | 0.0801 m / 0.400° | **0.04687 m** | Mapping + nominal planning complete; preferred evidence baseline |
| `autonomous_20261101_170420_346072` | `true` (verified) | **1.6085 m / 25.333°** | **0.22545 m** | Mapping + nominal planning complete but geometry degraded; **do not use its map** |

In the enabled run, a Ceres graph solve occurs concurrently with a large late global correction. Intermediate map accuracy changes dramatically after that event. Disabling the loop-closure path removes that catastrophic correction under the tested conditions. This is **strong subsystem-level causal evidence**, **not identification of the faulty scan-pair/graph edge**. Loop closing must be repaired and tested separately before restoring it. Do **not** randomly relax clearance, AMCL confidence, or collision gates.

The provisional offline TF regression detects one-step global corrections over **0.25 m or 5°** at at most 0.1 s sample gaps. A flagged correction requires map review; a large *legitimate* loop closure can also trigger it. Passing this check is **not** a map accuracy certificate.

**Current baseline config:** `config/slam_sim.yaml` contains `do_loop_closing: false` and `debug_logging: false`. `tools/mission/mission_slam.launch.py` must **not override** the YAML loop parameter. Always verify with `ros2 param get /slam_toolbox do_loop_closing` during runtime when performing an ablation.

## 3. Environment and initial setup

Laptop configuration used for the demonstrated experiments:

- Ubuntu 22.04, ROS 2 Humble, zsh; F1TENTH Gym + Gym ROS 2 bridge; Python virtual environment and ROS workspace installed in `~/f1tenth_sim`.
- `ROS_DOMAIN_ID=42`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.
- A dedicated repository checkout, e.g. `~/f1tenth_supervisor_demo`, can reuse the built simulation workspace. **It does not replace** the ROS build workspace by itself.
- Required sources and pinned simulator adaptations are described in `simulation_dependencies.repos`, `patches/simulator/`, `scripts/sim_env.zsh` and `RUN_SIMULATOR.md` (where available locally).

```zsh
# In zsh, from a code checkout:
cd ~/f1tenth_supervisor_demo
source scripts/sim_env.zsh
python3 tools/mission/full_autonomy.py --help
```

Environment setup fails with a helpful message if `~/f1tenth_sim/.venv`, `~/f1tenth_sim/install/setup.zsh` or the Gym source tree is missing. For a new laptop, first build/install the separate simulator dependencies; do not expect `git clone` alone to provide ROS, the virtual environment or installed Gym.

**No competing simulator or controller:** `scripts/run_full_autonomy.zsh` already starts the simulator. Do not simultaneously launch `scripts/run_sim.zsh`, a second autonomy mission, another `/drive` publisher, RF2O odom TF, and EKF TF for the same edge.

## 4. Reproduce the demonstrated mapping-only mission

```zsh
cd ~/f1tenth_supervisor_demo
zsh scripts/run_full_autonomy.zsh --mapping-only --explore-speed 0.18
```

This launcher automatically supplies `--autonomous --start-simulator`, and performs the simulator preflight, watchdog, simulated sensors, EKF, scan relay, SLAM, lap watcher and explorer, map saving, nominal route preparation, then stops **before** starting AMCL driving/racing. It creates a new directory such as `results/missions/autonomous_YYYYMMDD_HHMMSS_ffffff` and records a final status `COMPLETED_MAPPING_AND_PLANNING` on success. This is **not** a `COMPLETED_ESTIMATED_LAPS` result.

**Optional ROS bag** (start *before* the mission in another zsh terminal):

```zsh
cd ~/f1tenth_supervisor_demo
source scripts/sim_env.zsh
OUT="$HOME/f1tenth_engineering_evidence/mapping_$(date +%Y%m%d_%H%M%S)"
ros2 bag record -o "$OUT" \
  /tf /tf_static /scan /fusion/scan /odometry/filtered \
  /ego_racecar/odom /mission/slam_map /drive \
  /ego_racecar/collision
```

Stop bag recording with Ctrl+C **after** the mission exits. Simulator truth topics are included **only for passive offline measurement**, never for mapping, localization, or controller input.

Check a completed mission:

```zsh
RUN="$(ls -dt results/missions/autonomous_20* | head -1)"
cat "$RUN/mission_status.json"
cat "$RUN/lap_pose.json"
cat "$RUN/route_preparation.json"
ls -lh "$RUN"/new_track*
```

**Do not drive simply because the route audit prints `accepted: true`.** The map might still be distorted, localization untrusted, or commanded tracking infeasible.

## 5. End-to-end data flow and frame ownership

```text
F1TENTH Gym (/scan, /ego_racecar/odom truth)
    |       |                         simulator truth: evaluation only
    |       +--> SIM-ONLY sim_vesc_imu.py --> /sim/vesc/wheel_odom (forward vx)
    |                                      +--> /sim/imu/data_raw (yaw rate)
    |                                      +--> robot_localization EKF
    |                                               |
    |                                   fusion_odom -> fusion_base
    |
    +--> /fusion/scan (identical LiDAR ranges, corrected frame)
                    |
                SLAM Toolbox (mapping mode)
                    |
         slam_map -> fusion_odom, /mission/slam_map
                    |
             save new_track.yaml + new_track.pgm
                    |
             observed-corridor closure gate
                    |
           centerline + oriented vehicle trajectory
                    |
        body clearance + steering/curvature audit
                    |
        new_track_body_optimized.csv + speed profile

Alternative historical saved-map mode (not current pass claim):
 saved map + scan + EKF -> AMCL map correction -> estimated-pose controller
                        -> /mission/drive -> simulator watchdog -> /drive
```

Important geometry contract (source: `docs/2026-10-29-autonomous-mission.md`): Gym publishes the vehicle **centre of gravity** as its truth body. The EKF's `fusion_base` represents the **rear axle**, 0.17145 m behind the CoG. The simulated laser is 0.10000 m forward of CoG; accordingly `fusion_base → fusion_laser = +0.27145 m` in this model. The collision body is 0.58 m × 0.31 m and is centred 0.16510 m ahead of rear axle. These are **simulator model conventions**, not measurements of the physical car. Never mix `map` (simulator reference world) with `slam_map` (estimated map) without a validated transform.

**TF ownership:** One node publishes each odometry TF. Mapping publishes `slam_map → fusion_odom`, EKF publishes `fusion_odom → fusion_base`; map-localizing AMCL can also publish `slam_map → fusion_odom`, so **do not run SLAM mapping and AMCL simultaneously**. `tools/navigation/drive_watchdog.py` is the only final `/drive` publisher during the managed mission; exploration/controllers write `/mission/drive`.

## 6. Maps, output files and what belongs in Git

Original simulator map: `maps/uj_technical_loop_v1.yaml` and its PNG, **720 × 480 cells, 0.025 m/cell, origin (-9,-6,0)**. `maps/uj_oval_debug` is a simpler synthetic test circuit. The original simulator map is permitted only for simulation rendering and **offline evaluation**, not as the supposedly unknown map for the explorer/planner.

**Known-good generated map** (1 Nov): `results/missions/autonomous_20261101_164525_862826/`, from a local mission recording. Its origin and raster dimensions are determined by the saved `new_track.yaml`/PGM; they are *not* the original simulator's map frame. Expected contents:

| Path | Meaning |
|---|---|
| `mission_status.json`, `lap_pose.json` | Stage outcome, travelled distance, return pose and observed-corridor check |
| `new_track.yaml`, `new_track.pgm` | Saved occupancy map; keep **together** |
| `new_track_centerline.csv` | Centerline from the generated map |
| `new_track_mission_forward.csv` | Centerline oriented from the estimated finishing pose |
| `new_track_body_optimized.csv` | Candidate body-audited route (not certified for actual tracking) |
| `new_track_speed_profile.csv` | Planned per-waypoint speed envelope, not measured velocity |
| `route_preparation.json`, `new_track_preview.png` | Planning acceptance report and visual preview |
| `commands.jsonl`, `*.log` | Reproducibility evidence; log text may contain workstation paths |

For the preferred no-loop run, estimated lap distance was **34.329 m** over **228.74 s**; EKF return distance **0.848 m**, return heading difference **13.59°**; the observed corridor check used **0.30 m erosion**, while the accepted *planned* route had **0.354 m minimum conservative body clearance** in its nominal map audit. Planned speed envelope in the accepted route was about **0.616–0.800 m/s**; exploration was requested at 0.18 m/s. These are separate quantities.

The large local `results/missions/` history and ROS `*.db3` recordings are **not automatically part of GitHub `main`**. They require an intentional artifact curation/upload; **never run `git add -A`** on the raw results tree. Preserve bags offline. Teammates must receive the matching validated map and route bundle, not just a documentation link.

## 7. Audit and test results

Use the standalone validation tool when available (`tools/validation/mapping_correction_audit.py`, or the distributed `f1tenth_mapping_correction_audit.py`):

```bash
python3 tools/validation/mapping_correction_audit.py \
  /absolute/path/to/mapping_YYYYMMDD_HHMMSS_0.db3
```

This performs a **read-only** `/tf` jump regression (no ROS runtime needed). The no-loop recording showed **0 violations**, while the enabled-loop recording showed **1** with the 1.6085 m / 25.333° jump. Historical unit/ROS integration/CI runs were reported as passing **110 offline tests** by 30 October, but a green offline suite did not imply the AMCL driving mission completed; see the dated CI and handoff documents for exact revisions.

Reference-track wall accuracy is evaluated *offline* by independently rigid-aligning observed SLAM wall boundaries against the simulator reference. It is a geometric diagnostic, **not** a physically surveyed-map guarantee or proof of absolute localization accuracy. Keep the original reference separate from navigation data.

## 8. Current limitations and next engineering work

1. **Physical readiness**: replace the simulated velocity-derived sensor emulator with real VESC wheel-speed and IMU readings; calibrate wheel diameter, ERPM gain, gyro bias, lever arms and laser extrinsics. Never feed `/ego_racecar/odom` into physical estimators.
2. **Localization**: repeat AMCL or an alternative on the validated no-loop map, with independent truth-relative motion audit, stationary and moving tests, reliable initial-pose handoff, and route-relative uncertainty metrics.
3. **Loop closure**: isolate the candidate constraint and covariance, verify graph consistency, then restore loop closing *only after* correcting the cause and passing repeated maps including a holdout track.
4. **Safety**: physical independent E-stop, command arbitration, timeout and steering calibration must be commissioned before hardware autonomous operation. Simulator watchdog is **not** a hardware safety case.
5. **Release evidence**: keep exact source commit, runtime parameters, track image/YAML, saved map/route bundle, ROS bag, mission logs and validation output together for each accepted map.

**Teammate ownership:** Moatasem — EKF/sensor fusion and real sensor characterization; Ahmad — mapping-to-route planning, footprint/steering audits; integration lead — TF ownership, launch coordination, safe handoff and validation. Verify actual assignments with the team.

## 9. Historical reports and technical references

- [Master engineering log — through 1 Nov 2026](docs/F1TENTH_SIMULATION_MASTER_LOG_2026-11-01.md)
- [29 October automatic mission, frames and controller design](docs/2026-10-29-autonomous-mission.md)
- [30 October stopped full-mission/CI review](docs/2026-10-30-simulation-handoff.md)
- [27 October early development and oracle Pure Pursuit laps](docs/2026-10-27-development-log.md)
- [29 October recorded AMCL findings](docs/2026-10-29-recorded-localization-findings.md)
- [29 October AMCL confidence investigation](docs/2026-10-29-amcl-confidence-root-cause.md)
- [28 October detailed technical report](docs/F1TENTH_UJ_Simulator_Technical_Report_2026-10-28.md)

**Evidence policy:** Distinguish simulator truth, estimated odometry, mapping, accepted route geometry, measured lap completion and physical validation. No one of those proves all the others.
