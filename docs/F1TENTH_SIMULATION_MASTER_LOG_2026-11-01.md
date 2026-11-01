# F1TENTH UJ — Complete Simulation Engineering Log and Teammate Handoff

**Coverage:** 26 October–1 November 2026 (with earlier simulation milestones referenced); **latest review:** 1 November 2026.  
**Repository:** `EngZaq/f1tenth_uj_sim`, `main`. **Physical vehicle:** `EngZaq/f1tenth_uj`.  
**Reference simulation code commit immediately after the user’s code push:** `6d82b4ea2854fb0baacf575800467f142fcff1d7` (documentation commits may advance `main` afterward).  
**Scope:** simulator design, map building, localization studies, path planning, tracking tests, SLAM diagnosis and transition to hardware. This is not a physical-system safety certificate.

> **HOW TO READ THIS LOG:** Sections A–O below are the **current authoritative summary** as of 1 November. The extensive **Historical Appendix** at the bottom reproduces the complete 31 October handoff so no earlier troubleshooting detail is lost. Older sections deliberately contain historical, superseded conclusions and are not current-release status. The current result is **mapping + nominal planning completed**, **loop closing temporarily OFF**, **estimated-pose autonomous racing NOT validated end-to-end**.

---

# A. Executive engineering verdict

## A1. Demonstrated, partially demonstrated, and blocked

| Area | Verified work | Remaining validation / constraint |
|---|---|---|
| Full ROS/Gym environment | Run in Ubuntu 22.04 / ROS 2 Humble, domain 42, CycloneDDS, Python virtual environment | Teammates need compiled workspace and simulator dependency checkouts |
| Simulator technical track | `uj_technical_loop_v1`, real simulator collision and LiDAR ray geometry used | Simulator original map must remain privileged reference, not navigation input |
| Unknown-track exploration | Autonomous exploration circuit traversal and observed-occupancy closure check passed | Behavior must be retested with real LiDAR/vehicle and alternate tracks |
| Local sensor fusion | `robot_localization` EKF consumes emulated VESC-style forward speed and IMU yaw rate | Synthetic sensor source is derived from simulator twist; real bias/slip/scale must be measured |
| SLAM occupancy mapping | 2.5 cm grid, new map saved, route extracted | Loop-closure-enabled runs distort geometry; only no-loop baseline has verified wall RMSE |
| Safe route preparation | Centerline, orientation, body/steering optimization, curvature-based speed envelope; nominal route accepted | Does not prove closed-loop tracking/localization, swept trajectory under uncertainty, or physical collision safety |
| Saved-map AMCL | Initialization, stationary/moving observations, covariance-aware controller, stop guards exercised | Failed full estimated-pose laps and confidence stops; don't claim full mission success |
| Pure Pursuit | Historical oracle-pose simulation achieved five repeat laps | Oracle-pose racing is separate from SLAM/AMCL-based driving |
| Watchdog | Simulation `/drive` owner checks scan staleness, command age, range and collisions | Real hardware needs independent E-stop, explicit actuator calibration and physical watchdog |

## A2. Specific final release decision

- `config/slam_sim.yaml`: `do_loop_closing: false` and `debug_logging: false` for the **known-good mapping baseline**.
- `tools/mission/mission_slam.launch.py`: source-of-truth parameter fix; do **not** override `do_loop_closing` in the launch dictionary after loading YAML. Experiment 09 exposed this real configuration bug.
- `scripts/run_full_autonomy.zsh --mapping-only --explore-speed 0.18`: current validated **mapping/nominal-planning** demo path, not a race claim.
- **Preferred map evidence:** `autonomous_20261101_164525_862826` (no-loop, passed observed-corridor and nominal route gates). **Rejected map:** `autonomous_20261101_170420_346072` (loop-enabled, major mapping distortion), even though its own route audit printed accepted.
- **Code ownership:** only simulation code; physical vehicle has a distinct repository and motor/servo/sensor calibration tasks. Do not run the simulator mission launcher on the Jetson with a live VESC.

---

# B. Physical and mathematical system contract

## B1. Purpose and mission sequence

The car starts on an unknown bounded closed circuit. On the first lap it explores based on LiDAR and local odometry, constructs an occupancy map, verifies a closed observed free-space corridor, saves the map, calculates a centreline and a body-feasible route, and prepares a speed profile. A later mode is designed to localize on the saved map and follow its waypoints. Multiple autonomous estimated-pose laps and speed progression are research goals, **not verified completion for this final mapping-only handoff**.

The mission stage sequence in `tools/mission/full_autonomy.py` is:

```text
PREFLIGHT
  -> EXPLORING
  -> SAVING_MAP
  -> PLANNING
  -> COMPLETED_MAPPING_AND_PLANNING (only with --mapping-only)

Else: -> WAITING_FOR_LOCALIZATION_HANDOFF
      -> LOCALIZING_AND_RACING
      -> COMPLETED_ESTIMATED_LAPS only if requested lap record exists
```

Failures are captured by `STOPPED`, with a stage and reason. The status is in `mission_status.json`; stdout banners are not sufficient evidence. There must be one `/drive` publisher, managed by `drive_watchdog.py` while this launcher is active. A live ROS scan timeout, stale command, simulator collision or node failure should stop the mission.

## B2. Geometry and frames — critical error corrected 29 October

Gym's `ego_racecar/base_link` truth is at the **centre of gravity**. The rear-axle odometry and Pure Pursuit reference `fusion_base` is 0.17145 m behind CoG. Raw simulator LiDAR is 0.10000 m ahead of CoG, so `fusion_base → fusion_laser` is +0.27145 m along X. The simulated collision body is 0.58 m long, 0.31 m wide, with its center 0.16510 m forward of the rear axle. These constants are encoded in `tools/sim_geometry.py` and should be checked against the pinned Gym model. Mixing CoG and axle origins creates turn-dependent localization error that cannot be fixed by tuning AMCL.

The correct estimated TF chain **during mapping** is:

```text
slam_map  --(SLAM Toolbox)--> fusion_odom --(robot_localization EKF)--> fusion_base
                                                                   |
                                                               fusion_laser (static)
```

The **simulator reference** frame `map` is not the SLAM map frame `slam_map`. Never simply compare XY values or overlay their maps without separate rigid registration. Mapping SLAM and saved-map AMCL both may broadcast a global `map -> odom` correction; only one should own it at a time. The watchdog owns output `/drive`, exploration/controller owns input `/mission/drive`. `ego_racecar/odom` must be used only by simulator emulation and independent evaluation, not as the global localization or controller pose.

## B3. Sensor model and EKF mathematics

Simulator emulation in `tools/fusion/sim_vesc_imu.py` takes simulator twist, adds configured sensor characteristics and produces `/sim/vesc/wheel_odom` and `/sim/imu/data_raw`. The fused EKF in `config/fusion_ekf.yaml` consumes **forward linear velocity** (`vx`) from the VESC-like measurement and **yaw rate** (`r`) from IMU. It does not fuse oracle global pose, pseudo yaw, fake lateral velocity or IMU orientation. The local planar kinematic prediction in rear-axle terms is approximately

\[
\dot{x}=v\cos\psi,\quad \dot{y}=v\sin\psi,\quad \dot{\psi}=r
\]

with filters and noise handled by `robot_localization`. The separate command-integrated kinematic bicycle model (`command_model_odom.py`) is a diagnostic, not real wheel feedback. A highly accurate *simulated* EKF may reflect idealized sensors and cannot be cited as evidence for the physical wheel encoder, steering backlash or tire slip.

## B4. SLAM / AMCL separation

- **SLAM Toolbox mapping** estimates a corrected trajectory and the observed occupancy map from `/fusion/scan` plus fused local odometry. It publishes `slam_map → fusion_odom` and `/mission/slam_map`.
- **Nav2 AMCL** later consumes an existing saved map plus LiDAR and EKF odometry. It estimates global pose and may publish its own `slam_map → fusion_odom`. This must start only after mapping stops and graph ownership is released.
- The configured AMCL uses `nav2_amcl::DifferentialMotionModel`. This describes a stochastic model of relative odometry increments; its presence alone does not establish an Ackermann-geometry bug. Motion-model uncertainty and observability must be investigated from measured data.
- Only the simulation has a privileged reference map. Do not bypass real exploration or true localization by secretly using it in the map server or route planner.

---

# C. Official known-good mission and artifacts

## C1. Experiment 10 — verified loop closure OFF, authoritative mapping baseline

Mission archive `autonomous_20261101_164525_862826.zip`, corresponding ROS bag recorded as `no_loop_20261101_164508_0.db3` (on laptop under `~/f1tenth_engineering_evidence/exp09/no_loop_20261101_164508/`). Runtime check: `ros2 param get /slam_toolbox do_loop_closing` printed **Boolean value is: False**. `mission_status.json` final stage: **`COMPLETED_MAPPING_AND_PLANNING`**, and racing not run.

- EKF-integrated first-lap distance **34.329 m**; exploration duration **228.74 s**.
- EKF return-position difference **0.848 m**; heading difference **13.59°**. These values do **not** demonstrate that odometry closes perfectly around the circuit.
- Observed corridor topology: `closed=true`; **73,911** known-free cells; **40,833** safe-eroded cells; largest enclosed area approximately **61.06 m²**; erosion clearance **0.30 m**. An observed topological loop is not equal to a physically drivable clearance certificate.
- Map saved as `new_track.pgm` + `new_track.yaml`, **0.025 m/pixel**.
- Nominal planner accepted **smoothing_sigma_points=1.0** after inspecting alternative candidates; audited conservative minimum body clearance ~**0.354 m** in the generated map; peak nominal steering ~**21.54°**, peak candidate lateral shift ~**0.023 m**. Planner minimum acceptance boundary in route audit is 0.150 m; that acceptance check is separate from 0.30 m map-corridor erosion.
- Planned speed envelope **0.61589–0.800 m/s**; this is the route's *future* profile, not the 0.18 m/s exploration setpoint and not achieved racing speed.
- Final saved occupancy wall discrepancy after independent rigid SE(2) alignment to original simulator track: **median 0.0331 m**, **RMS 0.04687 m**, **95th percentile 0.0892 m**, **97.69%** of observed wall points within 0.10 m of the reference. These are distance-to-boundary metrics subject to rasterization, map thickness, coverage, correspondence and alignment ambiguity.
- Maximum one-step recorded map-to-odom correction: **0.0801 m**, **0.400°**. Offline audit: **0 regression violations**.
- No proven estimated-pose race lap completion, no hardware validation, no proof global map frame accurately aligned with robot at all times.

Use this directory as **baseline evidence**. Keep matching PNG/PGM/YAML and CSV files together; map and route share the `slam_map` coordinate frame and should not be combined with another run's frame. The preferred run's route includes `new_track_centerline.csv`, `new_track_mission_forward.csv`, `new_track_body_optimized.csv`, `new_track_speed_profile.csv` and `route_preparation.json`.

## C2. Experiment 11 — loop closure ON, known bad map

Mission archive `autonomous_20261101_170420_346072.zip`, ROS bag `no_loop_20261101_170413_0.db3` (misleading directory basename inherited from recorder; effective live parameter **Boolean value is: True**). `debug_logging: true` was also confirmed at runtime. Mission completed mapping and *nominal* route planning, but the map is **not accepted** for downstream use.

- Estimated first lap **34.312 m** over **228.46 s**, EKF return difference **0.849 m**, heading **14.85°**.
- Corridors topology `closed=true`, 0.30 m erosion, nominal body-audited route minimum ~**0.348 m**. **This planning acceptance did not detect independent map geometry failure.**
- Measured `slam_map → fusion_odom` one-step jump **1.6085 m and 25.333°** during a **~0.020 s** TF sample interval; event about **217.9 s after exploration start** (or 221.08 s after the first recorded TF, because recorder began earlier).
- Ceres graph optimization log appears at the same event. A reduction of the optimizer's internal cost (~851 to ~1.83) is **not proof of map correctness**.
- Final map against independently rigidly aligned reference: **median 0.15713 m**, **RMS 0.22545 m**, **95th percentile 0.48035 m**. Around **4.8×** the no-loop RMSE.
- Offline correction audit: **1 violation** at the event. Treat map as distorted for this benchmark.

**Important scientific conclusion:** Enabled loop-closure path is **necessary for the observed catastrophic degradation under these controlled conditions**; this does **not** identify the specific accepted scan-pair, edge covariance, innovation or solver cost inconsistency. Do not simply increase loop score thresholds or enable a robust loss without a measured hypothesis and revalidation.

---

# D. Chronological experiment register (updated 1 November)

**Dates, artifacts and limitations are part of the result.** Entries below deliberately distinguish what was *measured* from what was merely implemented or requested.

| Experiment / date | Input or run ID | Measured result and conclusion |
|---|---|---|
| Early mapping and oracle Pure Pursuit (26–27 Oct) | `uj_technical_slam_01` + registered trajectory | Five truth-pose-controlled laps ~26.5 s each, mean CTE ~0.026 m; does not demonstrate independent localization or controller safety on estimated poses. |
| Robot geometry audit (29 Oct) | pinned F1TENTH Gym, fused sensor frame | Corrected rear-axle reference offset 0.17145 m and rear-axle→laser 0.27145 m. Reference-point mismatch previously contaminated turn diagnostics. |
| Initial integrated unknown map mission (29 Oct) | `autonomous_20261029_164139_586206` | Exploration, saved map and route preparation; ROS handoff stale graph-check stop; later fixed with fresh bounded ROS graph checks. |
| Integrated estimated-pose attempt (29 Oct) | `autonomous_20261029_182122_744147` | Reached localization/start, then confidence/recovery stopped; not a complete estimated-pose lap. |
| Sim full mission CI (30 Oct) | GH Actions run `41001639441` | 0 full laps; AMCL along-track confidence at ~0.124 m exceeded 0.12 m gate after ~0.706 m of drive; full mission blocked despite passing offline tests. |
| Saved-map moving localization (29–31 Oct) | `profiled_20261029_142531`, `race_20261031_134058` | Controller can track *estimated* route while AMCL corrections diverge from truth; uncertainty gate stopped before lap. Oct31 moving test AMCL relative RPE over 1/3/5 s greatly exceeds EKF RPE. |
| Older map clearance failure (31 Oct) | `results/missions/20261031_231805` | On older 5 cm map, free-space loop survived 0.29 m erosion but not 0.30 m. Do not lower clearance merely to force extraction. |
| Exp 04 stationary bag (31 Oct) | `stationary_20261031_231325` | 90.411 s static; raw/relayed LiDAR ranges identical; EKF moved <=0.00793 m, AMCL <=0.05258 m; pose/map endpoint consistency insufficient to prove geometry. |
| Exp 05 moving bag (31 Oct) | `moving_20261031_233228` | 2.615 m actual travel; first-pose-aligned EKF RMS 0.0024 m, AMCL 0.0671 m; 477 relayed scans matched raw; AMCL moved after stopping. |
| Exp 06 AMCL runtime audit (31 Oct) | `exp05_runtime_config.tar.gz` | Repeated no-motion update calls and along-track uncertainty identified; no proof this caused error. Led to testing map geometry first. |
| Exp 07 independent map geometry (31 Oct) | `map_evidence.tar.gz`, actual reference map | Old distorted SLAM map: median wall discrepancy 0.136 m, RMS 0.193 m, p95 0.378 m after rigid registration; scan endpoints align better with actual reference than the distorted SLAM map. |
| Exp 08 recorded full mapping (1 Nov) | `autonomous_20261101_104527_365599` | 34.273 m exploratory lap; intermediate wall RMSE 0.041 m before late graph event → 0.204 m after, while EKF stayed smooth; major map-to-odom jump. |
| Exp 09 **invalid ablation** (1 Nov) | `autonomous_20261101_110323_054722` | YAML `do_loop_closing:false` was overridden by `mission_slam.launch.py` forcing `True`; repeated 1.5 m / 24° correction. Important integration bug fixed. |
| Exp 10 **valid** no-loop ablation (1 Nov) | `autonomous_20261101_164525_862826` | Live param `False`; no large event; final wall RMSE 0.04687 m; correction-step audit PASS (0). Preferred baseline. |
| Exp 11 enabled-loop diagnostic (1 Nov) | `autonomous_20261101_170420_346072` | Live params `do_loop_closing=True`, `debug_logging=True`; late 1.6085 m/25.333° jump and map wall RMSE 0.22545 m; audit flags 1. |

### D1. Important distinction: historical localization values

The early `profiled_20261029_142531` audit reports moving AMCL relative RMS ~0.087 m vs a historically recorded EKF error; later complete-lap Experiment 08 reports EKF RMS ~0.0356 m and SLAM RMS ~0.0851 m **before** a late correction under a first-pose alignment. Experiment 05 reported EKF 0.0024 m on a much shorter, more idealized segment. These use **different conditions, track distances, frames, timing and observation windows**; do not compare them as if they were a single benchmark table.

The stationary bag's low mean scan endpoint-to-occupied-cell residual (~6.9 mm) is **not** evidence of an absolutely accurate occupancy map. A separate first-hit raycast gave ~9.3 cm mean absolute range difference for a sampled subset, illustrating map thickness and ray-casting assumptions. The correct subsequent approach was independent wall geometry against the actual simulator track and then a controlled loop-closure intervention.

### D2. Experiment 08 intermediate map evidence

The map wall-error calculation extracted observed occupied boundary cells adjacent to known free regions, independently rigidly registered each time snapshot to the original simulator-track wall boundaries, and measured nearest-boundary distance. It does not conflate a shifted map origin with deformation:

| Time after exploration start | Loop ON (Exp08) wall RMSE | Loop OFF (Exp10) wall RMSE |
|---|---:|---:|
| ~216 s (before event) | 0.041 m | 0.047 m |
| ~222 s (after event) | 0.204 m | 0.047 m |
| ~228 s | 0.206 m | 0.047 m |
| ~237 s | 0.207 m | 0.047 m |

The loop-enabled correction happened in a fraction of a second, while the EKF and truth continued smoothly. This is the most compelling data-backed reason to defer AMCL tuning and keep the stable map.

### D3. Final audit reproduced by the user on the laptop

The user ran the same offline `f1tenth_mapping_correction_audit.py` script on both locally preserved `.db3` bags:

```text
LOOP CLOSURE OFF (exp09/no_loop_20261101_164508/...)
  slam_map -> fusion_odom
  12112 TF samples; max translation step 0.0801 m;
  max yaw step 0.400°; violations: 0; PASS (correction-step only).

LOOP CLOSURE ON (exp10/no_loop_20261101_170413/...)
  slam_map -> fusion_odom
  12081 TF samples; max translation step 1.6085 m;
  max yaw step 25.333°; t=221.08 s from first TF;
  dt=0.020 s; violations: 1; REVIEW REQUIRED.
```

Names `exp09` and `exp10` above are *recorder folders*, not the experiment labels; this naming mismatch caused two avoidable path errors. **Use actual UTC timestamps, mission ID and runtime parameter evidence instead of inferring loop state from a bag folder name.**

---

# E. Maps and artifacts: exact provenance

## E1. Map families

| Map | Input/representation | Truth status | Role |
|---|---|---|---|
| `uj_oval_debug` | original synthetic circuit under `maps/` | simulator reference fixture | quick smoke tests only |
| `uj_technical_loop_v1` | original PNG/YAML under `maps/`, 720×480 at 0.025 m/cell, origin (-9,-6,0) | **privileged simulator world reference** | simulation rendering and offline map-truth evaluations, never mapping/navigation feed |
| `uj_technical_slam_01` | historical saved SLAM PGM/YAML + `.posegraph` + `.data` | derived from an earlier mapping run | early exploration and localized/registered reference experiments; historical coordinate convention may differ |
| `new_track` from 31 Oct supervisor demo | `autonomous_20261031_131817_595257` | derived map; later shown distorted | not recommended for AMCL/hardware baseline |
| `new_track` from Exp08 | `autonomous_20261101_104527_365599` | loop-enabled derived map | reference for identifying graph correction; distorted later in lap |
| `new_track` from Exp10 | `autonomous_20261101_164525_862826` | **preferred, verified no-loop derived map** | candidate for controlled saved-map localization tests; still not a safety certificate |
| `new_track` from Exp11 | `autonomous_20261101_170420_346072` | loop-enabled derived map | diagnostic only; reject for downstream driving |

## E2. How map files relate

- **`.yaml`**: map raster filename, origin and metres/cell. Keep it next to the PGM; do not substitute the simulator's YAML.
- **`.pgm`**: occupancy raster. Pixel row orientation and unknown/free/occupied values matter. `map_saver_cli` creates this from `/mission/slam_map`.
- **`.posegraph`/`.data`**: serialized SLAM graph/data from runs that saved the serialized graph. They are *not* interchangeable with PGM-only localization and are not necessarily generated by the current mapping-only launcher.
- **`new_track_centerline.csv`**: extracted centreline in `slam_map` coordinates; **`new_track_mission_forward.csv`**: direction/origin oriented to estimated final SLAM pose; **`new_track_body_optimized.csv`**: path from steering/vehicle-body optimization; **`new_track_speed_profile.csv`**: speed-planning input, not measured speed.
- **`route_preparation.json`**: attempted smoothing candidates and their exact reasons for rejection/acceptance. Some attempts show `REJECTED` even when the *final* selected attempt is accepted; inspect the complete JSON.
- **`lap_pose.json`**: integrated EKF distance, estimated start-return error, `observed_corridor` coverage, final SLAM pose and `source` annotation. Its distance is *not* a surveyed track length.
- **`mission_status.json`**: authoritative mission stage and event timestamps; `COMPLETED_MAPPING_AND_PLANNING` is not an autonomous race completion record.
- **`commands.jsonl` and component logs**: command reproducibility, node startup, abnormal warnings and failure origin; original workstation paths may need updating for another machine.

## E3. Prefer archive + manifest over raw Git bulk upload

The laptop had a large amount of untracked material (`results/missions/autonomous_*` directories and ZIPs, `.tar.gz`, logs, old driver backups). **Do not `git add -A`** and unintentionally commit ROS bags and binary output histories. Keep a curated full-copy evidence archive on disk, add text README/log/source to Git, then separately distribute the preferred map + YAML + CSV + JSON + preview. The final team handoff ZIP created from the shared evidence contains this preferred bundle.

An intentionally portable artifact should carry (1) code SHA, (2) actual ROS parameter values from runtime, (3) map pair, (4) route pair, (5) mission JSONs, (6) ROS bag location/checksum and (7) the audit verdict. A ZIP containing a map without its YAML or generated route is insufficient.

---

# F. How teammates reproduce and verify without damaging their workspace

## F1. Environment layout

**Code checkout:** `~/f1tenth_supervisor_demo` (or another directory with this repository's source).  
**Built ROS/Gym workspace:** `~/f1tenth_sim` (separate; `.venv/`, `install/setup.zsh`, `src/f1tenth_gym_ros/f1tenth_gym`).  
**Environment helper:** `scripts/sim_env.zsh` (must be sourced by zsh; domain 42, CycloneDDS, site-packages bridge).  
**Config:** `config/` in the code checkout, not automatically copied from the built workspace.

```zsh
cd ~/f1tenth_supervisor_demo
source scripts/sim_env.zsh
python3 tools/mission/full_autonomy.py --help | grep -E 'mapping-only|explore-speed'
```

The expected flags exist in GitHub's 1 November main commit. If `source` fails, follow `scripts/sim_env.zsh` error: locate the built workspace, activate its virtualenv, then verify the pinned Gym source and ROS installation. A source-only Git clone does not compile Gym ROS.

## F2. One-command mapping-only demonstration

```zsh
cd ~/f1tenth_supervisor_demo
zsh scripts/run_full_autonomy.zsh --mapping-only --explore-speed 0.18
```

The launcher starts the simulator itself; no other simulator instance or `/drive` owner. `--mapping-only` stops after route preparation. The output is a unique mission folder under `results/missions/`. Allow the automatic safety shutdown to complete. Never assume a printed `PLANNING` stage means planning *passed*; inspect the last status and route report.

## F3. Optional complete mapping bag

```zsh
# Terminal A, BEFORE launching mission:cd ~/f1tenth_supervisor_demo
source scripts/sim_env.zsh
ros2 bag record -o "$HOME/f1tenth_engineering_evidence/recording_$(date +%Y%m%d_%H%M%S)" \
  /scan /fusion/scan /odometry/filtered /ego_racecar/odom \
  /tf /tf_static /mission/slam_map /drive /ego_racecar/collision

# Terminal B, start mission:
cd ~/f1tenth_supervisor_demo
zsh scripts/run_full_autonomy.zsh --mapping-only --explore-speed 0.18
```

After mission exits, stop Terminal A with Ctrl+C (to finish the SQLite metadata correctly). Confirm `ros2 bag info`. The raw simulator odom topic appears in the bag for passive truth audit only.

## F4. Inspect mission result and run audit

```zsh
cd ~/f1tenth_supervisor_demo
RUN="$(ls -dt results/missions/autonomous_20* | head -1)"
python3 -m json.tool "$RUN/mission_status.json"
python3 -m json.tool "$RUN/lap_pose.json"
python3 -m json.tool "$RUN/route_preparation.json"

# Audit recorded /tf; substitute your REAL recorded bag path:
python3 tools/validation/mapping_correction_audit.py \
  /absolute/path/to/recording_0.db3
```

Audit code exit zero means no overly large consecutive correction *under provisional bounds*, **not** proof a route is safe. The current reference-track validation metrics are produced by external offline analysis; the reference map may not be present on a teammate's laptop until simulator fixtures are installed.

## F5. Simulator-only test commands

```zsh
# Inspect existing pytest suite first. Does not drive the simulated car.
cd ~/f1tenth_supervisor_demo
source scripts/sim_env.zsh
python3 -m pytest -q tests

# Verify current ROS domain configuration:
echo "$ROS_DOMAIN_ID"
echo "$RMW_IMPLEMENTATION"

# Verify a live SLAM mapping parameter DURING the mission:
ros2 param get /slam_toolbox do_loop_closing
ros2 param get /slam_toolbox debug_logging
```

There is a separate offline CI workflow and an integrated Gym/ROS workflow. The Oct30 claim that 110 offline tests passed describes its exact historical checkout, not an assertion that every new local/teammate environment has been freshly run. We have not run the full ROS/Gym CI in this response.

---

# G. Detailed source code map

| Location | Role and important distinction |
|---|---|
| `scripts/sim_env.zsh` | Sources ROS/venv/Gym, sets ROS domain and RMW; for zsh only |
| `scripts/run_full_autonomy.zsh` | Wraps Python mission with `--autonomous --start-simulator`; forwards user flags |
| `scripts/run_sim.zsh` | Standalone simulator; do **not** launch concurrently with automatic mission |
| `tools/mission/full_autonomy.py` | Single manager, process lifecycle, stage times, watchdog ownership, map save, route prep, optional estimated driving |
| `tools/mission/mission_explore.py` | Starts exploration controller and sets requested exploration speed |
| `tools/exploration/exploration_controller.py` | LiDAR-guided exploration; local demo updates are now in main; observe its bounds and collision behavior |
| `tools/mission/mission_first_lap.py` | Tracks odom distance/return and requires observed free-space loop; records final SLAM pose |
| `tools/mission/map_coverage.py` | Erodes only **observed** free space; keeps unknown blocked; requires topological enclosed interior; 0.30 m applied in tests |
| `tools/mission/mission_slam.launch.py` | Launches dedicated SLAM mapping TF chain and maps to `/mission/slam_map`; removed hardcoded loop-closing override |
| `config/slam_sim.yaml` | 0.025 m map, matching resolution, loop closure **disabled** for baseline; its parameters are authoritative unless independently proven otherwise |
| `tools/fusion/sim_vesc_imu.py` | Simulator-only sensor emulator; **derives velocity from truth twist** and applies sensor noise |
| `config/fusion_ekf.yaml` | EKF fuses forward velocity and yaw rate, publishes fused rear-axle odometry TF |
| `tools/localization/fusion_scan_relay.py` | Reframes raw LiDAR to fusion laser frame preserving source acquisition stamps and ranges |
| `tools/mission/mission_amcl.launch.py` | Managed saved-map localization stage, separate from mapping TF ownership |
| `config/fusion_amcl.yaml` | Historical AMCL likelihood/motion model configuration; not certified on new 1 Nov map |
| `tools/localization/initialize_amcl_pose.py` | Handles initial-pose handoff/ROS graph and seed checks |
| `tools/mission/run_profiled_saved_map.py` | Historical estimated-pose race launcher; must not be inferred as validated from mapping-only success |
| `tools/navigation/profiled_amcl_drive.py` | Conservative estimated-pose path follower with covariance, freshness and local collision policy |
| `tools/navigation/drive_watchdog.py` | Single simulator `/drive` owner; command heartbeat, LiDAR and collision stop |
| `tools/navigation/trajectory_safety.py` | Swept-collision/footprint checks for planned controller motion |
| `tools/navigation/confidence_policy.py`, `pose_uncertainty.py` | Route-relative covariance and policy validation |
| `tools/planning/extract_robust_centerline.py` | Converts saved occupancy into path skeleton/centerline; reports no closed corridor when erosion fails |
| `tools/planning/prepare_vehicle_route.py`, `optimize_vehicle_route.py` | Smooth body/steering-feasible path and planner report |
| `tools/planning/vehicle_footprint_audit.py` | Model-based body clearance audit; distinct from real dynamic safety |
| `tools/localization/sim_localization_audit.py` | Passive simulator-truth comparison, not navigation input |
| `tools/localization/summarize_sim_localization_audit.py` | Analysis of estimated pose vs independent simulator reference and controller logs |
| `tests/` and `.github/workflows/` | Offline unit/fixture tests and live ROS/Gym integration workflows |
| `tools/validation/mapping_correction_audit.py` | Offline TF correction regression tool distributed in 1 Nov handoff; no ROS runtime required |
| `docs/` | Date-stamped engineering history and source-specific experiment findings |

The entire repo has other diagnostic/archived scripts beyond this operational list. Old scripts such as `run_unknown_track_mission.zsh`, former `rf2o_sim.yaml`, oracle controllers, `*.before_fix` backups and older `reference/generated` map data are **historical or alternative experiments**, not interchangeable with the final mission pipeline.

---

# H. Root-cause analysis: why the SLAM map broke

## H1. The observation

In the loop-enabled full mapping recording, the car's actual motion and local EKF remained continuous, yet `slam_map → fusion_odom` jumped ~1.6085 m and ~25.33° in ~20 ms, and the independently registered wall geometry became far less accurate. This jump is a change in *estimated map correction*, not actual vehicle dynamics. The map itself deformed under graph adjustment: not merely a global origin/heading change.

## H2. Clean intervention and configuration trap

Initially, Experiment 09 edited `config/slam_sim.yaml` from `true` to `false` but saw the same jump. It was **not an effective ablation** because `tools/mission/mission_slam.launch.py` supplied a second parameter dict with `'do_loop_closing': True`, overriding YAML. We removed that override, reran the mission, and verified the live ROS parameter as `False`. Then the global jump disappeared and final wall RMSE dropped to 4.69 cm. Re-enabling loop closing and `debug_logging` brought the failure back. This is a reproducible parameter-level intervention.

## H3. The optimizer and candidate uncertainty

Humble SLAM Toolbox calls Karto's `TryCloseLoop()` to find a candidate chain, do coarse scan matching, check response and coarse x/y covariances, refine matching, accept a response-threshold pass, link the graph, and invoke Ceres. The default configured `ceres_loss_function: None` uses squared residuals. The debug log confirms optimization, but does **not** identify accepted scan IDs, relative pose, full covariance, information matrix, or candidate innovation. A valid Ceres convergence is not independent evidence of a correct physical wall map.

A possible mathematical validation for proposed loop edge \((i,j)\) is

\[
e_{ij}=\mathrm{Log}\left(Z_{ij}^{-1}T_i^{-1}T_j\right),\qquad
D^2=e_{ij}^{T}S_{ij}^{-1}e_{ij},
\]

where \(Z_{ij}\) is the candidate scan-matched transform and \(S_{ij}\) correctly accounts for measurement and accumulated odometry uncertainty. A poorly justified fixed threshold can reject legitimate loop closures after genuine drift, so **no such gate has yet been implemented**. This is deferred research, not an emergency blocker for initial physical mapping with loop closure off.

## H4. What the experiments *do not* conclude

- They do **not** prove every large map correction is invalid; an accurate loop closure can move map-to-odom suddenly.
- They do **not** prove that increasing AMCL particles, tightening `sigma_hit`, changing EKF process variance or switching AMCL's differential motion model will fix this map fault.
- They do **not** prove the simulator EKF will be accurate when integrating physical motor feedback and an actual BMI160.
- They do **not** prove the original simulator reference geometry matches a future physical track.
- They do **not** establish which particular loop-closure graph edge should be rejected. Proper instrumentation remains a later task.
- They do **not** prove reliable multi-lap estimated-pose racing at 1–5 m/s; historical speed parameters and oracle laps are separate.

---

# I. Safety, acceptance gates and failure classification

1. **Preflight:** exactly one actuation owner; no active duplicate autonomy or odometry TF; sensors publishing correct frames; watchdog responsive.
2. **Exploration:** source LiDAR timestamps valid; forward command inside bounded simulator policy; no simulator collision; only observed free-space cells considered.
3. **Lap/coverage:** enough travel and estimated return; observed corridor remains closed after 0.30 m erosion; otherwise continue or stop with failure, **do not lower clearance** to force a demo.
4. **Map QA:** save map and validate independent geometry when simulator truth available; run TF correction audit on complete bag; map with flagged global jump is quarantined pending review even if route geometry passes.
5. **Planning:** centerline in same SLAM frame, route orientation consistent with estimated finish pose, body/steering/speed audit passes; unknown cells treated as blocked.
6. **Localization handoff:** all mapping TF publishers shut down, map server active, AMCL reinitialized using scan evidence, fresh global poses observed; measured global errors and covariance calibrated before accepting driving.
7. **Control:** one writer, bounded speed and steering, scan/cmd heartbeat, collision latch, route-relative confidence and local swept-path stop.
8. **Completion:** only `COMPLETED_ESTIMATED_LAPS` with an actual lap record proves estimated-pose mission completion. A mapping-only `COMPLETED_MAPPING_AND_PLANNING` is never equivalent.
9. **Physical use:** independent hardwired stop/manual override, validated VESC conversion and servo angles, emergency-stop test, realistic body clearance and low-speed commissioning. The simulator watchdog is **not** sufficient.

Avoid using the same CTE estimated from AMCL to claim AMCL is accurate; both controller and error would share the same estimate. Use a passive simulator ground-truth audit, held-out map comparisons and physical independent measurements.

---

# J. Practical team handoff and collaboration

**Moatasem — sensor fusion:** characterize real motor velocity/ERPM and IMU yaw rate; estimate noise and bias from bench/static measurements; validate temporal alignment and heading sign; enforce one odometry TF publisher; compare physical odometry against independently measured straight-line and turning motion. Do not copy simulator noise/covariance as if measured.

**Ahmad — path planning:** own occupancy mask, centerline and route extraction; keep unknown-space blocked; audit complete body footprint, steering feasibility, interpolated path clearance and speed envelope; report the exact map YAML/PGM and frame used; prepare a holdout-track test. Do not optimize directly against privileged simulator reference in production mode.

**Integration lead:** coordinate single `/drive` watchdog owner; stages and launch, Git/ROS domain, known-good map artifact, confidence handling, stop semantics, and readable test reports; schedule low-speed physical tests. Preserve mapping-only and oracle-lap evidence as separate metrics. Hardware improvements belong in physical repo.

**Definition of handoff complete for simulation:** code in `main` has a current README; master log is discoverable; baseline no-loop mapping mission and its original output bundle are available; test scripts and logs are preserved; all known failure modes are visible; next owner knows exactly which system stages remain unvalidated. This is deliberately weaker than declaring the whole robotics research goal completed.

---

# K. Hardware transition: explicit transfer and non-transfer list

**Transferable ideas/code with adaptation:** robot_localization EKF structure, frame contract, SLAM Toolbox config (temporary loop-off), map saving, safe corridor checks, centreline and body-aware route design, Pure Pursuit formulation, top-level mission lifecycle and protective command arbitration.

**Must be replaced/recalibrated:** `sim_vesc_imu.py` (truth-derived sensor emulator); raw `ego_racecar` topics; simulator collision topic; simulator CoG/laser/footprint constants; the old `speed_to_erpm` gain unless physically measured; IMU axes, bias, wheel slippage, servo mapping and system latencies. The simulated initial pose may be unsuitable for a physical starting location.

**Physical repo gaps based on currently published source:** sensor launch had BMI160 node commented out (separate IMU launch available), LiDAR static X=0.195 m vs simulator model 0.27145 m, RF2O was the original TF odom owner in physical SLAM launch, and physical teleop configuration alone did not establish independent E-stop arbitration. Local Jetson code may differ; inventory before modifying. Keep simulator and physical worktrees separate.

**Hardware order:** bench emergency stop and servo/drive calibration → LiDAR/IMU/VESC topics with correct timestamps and extrinsics → fused local odom against measured motion → conservative physical mapping → validate saved-map global localization → low-speed autonomous closed-loop route → repeatable safe laps → speed optimization. Loop-closure research can continue in parallel.

---

# L. Evidence index — files and reproducibility

### Current no-loop and loop-enabled mission bundles

- `autonomous_20261101_164525_862826.zip`: no-loop final map, `mission_status.json`, `lap_pose.json`, `route_preparation.json`, CSV routes, PGM/YAML, component logs.
- `autonomous_20261101_170420_346072.zip`: loop-on diagnostic, internally accepted planned route but independent distorted map.
- `no_loop_20261101_164508_0.db3`: effective loop OFF; local archive in laptop's `exp09/` recorder directory.
- `no_loop_20261101_170413_0.db3`: effective loop ON; local archive in laptop's `exp10/` recorder directory, despite misleading `no_loop` filename.

### Offline analytical documents preserved from prior work

- `F1TENTH_Experiment_04_Stationary_Bag_Findings.md`
- `F1TENTH_Experiment_05_Moving_Localization_Analysis_2026-10-31.md`
- `F1TENTH_Experiment_06_Runtime_AMCL_Audit.md`
- `F1TENTH_Experiment_07_Reference_Track_Geometry_Review.md`
- `F1TENTH_Experiment_08_Engineering_Report_2026-11-01.md`
- `F1TENTH_Experiment_09_Ablation_Audit_2026-11-01.md`
- `F1TENTH_Experiment_10_Verified_No_Loop_Closure_Report.md`
- `F1TENTH_Engineering_Change_01_Loop_Closure_2026-11-01.md`
- `F1TENTH_exp10_map_vs_reference.png`, `F1TENTH_exp10_trajectory_errors.png`, `F1TENTH_exp08_wall_comparison.png`; time-series CSVs and wall-accuracy CSVs.
- `f1tenth_mapping_correction_audit.py`: standalone stdlib SQLite/CDR `/tf` regression (Python). Prefer tracked copy under `tools/validation/`.

### Historical reports already in GitHub (do not overwrite their dated conclusions)

- `docs/2026-10-27-development-log.md`
- `docs/2026-10-29-autonomous-mission.md`
- `docs/2026-10-29-recorded-localization-findings.md`
- `docs/2026-10-29-amcl-confidence-root-cause.md`
- `docs/2026-10-30-simulation-handoff.md`
- `docs/F1TENTH_UJ_Simulator_Technical_Report_2026-10-28.md`
- `tests/` and `.github/workflows/autonomy-offline.yml`, `.github/workflows/autonomy-gym.yml`.

All filenames in this section refer either to tracked files or external locally held evidence; a filename alone is **not** proof it has been uploaded to GitHub. The README should identify that distinction.

---

# M. Controlled next steps (post simulation handoff)

- **Immediately**: use validated no-loop map and keep its exact output bundle with code revision. Do not use the loop-enabled distorted map for AMCL or hardware.
- **Next hardware task**: validate Jetson runtime packages, driver topics, single TF ownership and real drive safety. Bring up sensors and estimator in stages, no autonomous motion until verified.
- **Next simulator research task in parallel**: validate AMCL on the 1 November no-loop PGM/YAML, measure global pose against simulator truth during motion and stationary pauses, diagnose confidence and along-track observability using held-out LiDAR measurements. This experiment is not yet claimed complete.
- **Later**: instrument Karto accepted loop-closure candidate IDs, coarse/fine score, relative edge pose/covariance and Ceres before/after. Develop a covariance-consistency gate using real odometry uncertainty, evaluate on the technical loop and a second unseen track; re-enable loop closure only after stable map comparison and safe mission validation.
- **Do not** increase simulator speed toward the earlier requested 5 m/s until estimated-pose tracking, tire/steering constraints, localization and collision margin validate it. The current mapping-only speed envelope stops at 0.8 m/s; it is not a speed-increase demonstration.

---

# N. Team glossary

- **Simulator truth / oracle:** privileged `/ego_racecar/odom` pose; useful only for synthetic sensor emulation and independent diagnostics.
- **Odometry (`fusion_odom`)**: locally smooth motion integration; accumulates drift, not global map location.
- **SLAM map (`slam_map`)**: map coordinate system estimated during exploration; unrelated to simulator world `map` except through an estimated transform.
- **AMCL:** particle-filter global localization against an already saved occupancy map.
- **EKF:** local fusion of forward wheel-speed and IMU yaw rate; current simulator uses emulated measurements.
- **Loop closure:** nonlocal graph constraint matching earlier and later scans to correct drift; can catastrophically distort a map if wrong or poorly weighted.
- **Map QA:** independent wall-geometry registration and TF jump audit; different from nominal path body clearance.
- **CTE:** cross-track error of a controller relative to its estimated or reference path; the estimator used must be stated.
- **Full mission completion:** actual estimated-pose lap count and explicit success state, not a waypoint preview or planner acceptance.
- **Stage-limited mission:** `--mapping-only`; stops after mapping and route preparation, no racing.

---

# O. Historical handoff preservation note

The following full 31 October project log is kept as an **unmodified historical appendix** so the next engineer can examine every older failure, branch, run ID, command and filename. The status therein precedes the 1 November loop-closure investigations. Treat Sections A–N above as the current decision record where they differ.

---

# HISTORICAL APPENDIX — Full project log, 31 October 2026 (archived)

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

- Revision: `4f8f774`.- Started sensor emulator, EKF and scan relay; matched a live scan to the recovered map.
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
| Measured raw laser transform | Assumed mount disagreed with raw scan geometry | Better sensor geometry; not a complete localization fix || Composed rear-axle geometry | CoG and rear-axle references were mixed | Consistent physical reference for mapping/guard/audit |
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
