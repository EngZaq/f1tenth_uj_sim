# Simulation mission handoff — 30 October 2026

Code revision under test: `2f60ebb5a68dd671a5e3631547e90008fa58258f`.
Branch: `engineering/autonomous-mission-20261029`.

## Current result: full mission remains blocked

[Latest full run 41001639441](https://github.com/EngZaq/f1tenth_uj_sim/actions/runs/41001639441) **failed** with zero completed laps. Startup passed the route-relative readiness check. After 0.706 m of physical travel, the controller stopped because along-track AMCL uncertainty did not recover below 0.12 m within four seconds (last reported 0.124 m; cross-track 0.019 m). Driving CTE maximum was 0.0169 m and sampled body clearance minimum 0.530 m. These small errors do not override a failed localization gate. This version is not a verified full-mission release.

The latest [offline CI run](https://github.com/EngZaq/f1tenth_uj_sim/actions/runs/41001639410) passed, including the real ROS fixtures.

Two further changes are included:
- Tracking speed decreases from the nominal cap as CTE rises from 0.02 to 0.06 m, reaching a cap of 0.10 m/s. The 0.09 m stop boundary remains active.
- Automatic startup projects covariance into the nearest route tangent/normal, consistent with the driving bounds (0.08 m across, 0.12 m along, 0.16 m principal standard deviation). Four fresh distinct observations remain required. Standalone readiness without a route retains its legacy stricter map-axis check.

The preceding [run 41000729268](https://github.com/EngZaq/f1tenth_uj_sim/actions/runs/41000729268) stopped before arming at the inconsistent legacy startup gate. Thus the tracking-speed change has passed offline regression, but its intended corner recovery has not yet been demonstrated in the combined mission.

The earlier [mapping run 40999826215](https://github.com/EngZaq/f1tenth_uj_sim/actions/runs/40999826215) travelled 12.308 m, reached progress 117/339, and stopped at estimated CTE 0.092 m. Sampled true-body clearance was 0.354 m. No run completed a lap.

Next engineering work is to diagnose AMCL motion/measurement noise and corridor observability using the recorded covariance, scan and motion evidence. Do not reduce reported covariance or relax the gate merely to obtain a lap. A validated motion/observation model and a complete integrated rerun are still needed.

## What the previous full test established

[Run 40939102256](https://github.com/EngZaq/f1tenth_uj_sim/actions/runs/40939102256) completed exploration, map saving, planning, clean ROS graph handoff and AMCL initialization. Racing stopped on the local collision guard after 5.708 m with zero completed laps. It was not a successful mission.

Estimated driving cross-track error was 0.0099 m RMS and 0.0374 m maximum. The passive simulator audit sampled a minimum full-body clearance of 0.125 m, with no occupied body samples. These are sampled observations, not continuous clearance guarantees. A replay of the last recorded scan at the last driving pose gave approximately 0.442 m RMS endpoint distance to occupied saved-map cells for valid ranges below 5 m, subsampled every third beam. This is a scan/map disagreement diagnostic, not ground-truth localization error.

## What changed for this test

The mapping configuration now uses 0.025 m cells instead of 0.05 m. SLAM accepts travel updates at 0.05 m or 0.035 rad instead of the launch overrides of 0.12 m or 0.10 rad; its minimum scan interval is 0.1 s instead of 0.5 s. The correlation smear is 0.03 m instead of 0.10 m. These changes test whether denser constraints and narrower matching resolve the bend distortion; passing unit tests alone cannot establish that.

The YAML also names the fusion odometry, rear-axle body and laser scan topic explicitly. The launch no longer hides conflicting travel thresholds in overrides.

Collision margin, braking assumptions, confidence limits, CTE limit and watchdog remain active.

## Stage checks

| Stage | Independent check |
|---|---|
| Geometry and synthetic sensors | Actual pinned Gym model contract checks CoG, rear axle, laser origin and collision body |
| Exploration | Estimated return plus observed known-free closed corridor; no assumed unknown free space |
| Planning | Full-body occupancy/unknown-space audit, dense between-waypoint checks and steering feasibility |
| SLAM → AMCL handoff | Real ROS fixture rejects a live mapping stack and accepts its clean departure |
| Initialization | Live scan seed, isolated initial-pose topic and bounded fresh AMCL observations; real ROS missing-first-reply fixture |
| Driving | Offline tracking/progress/confidence/guard regression tests, then actual ROS/Gym mission |
| Completion | Requested lap record, final STOP/zero command, watchdog state and passive physical travel/winding/body occupancy audit |

All 110 offline tests pass locally. The optimizer CLI checks also pass separately (2 tests included in that total). The Actions offline workflow also passed, including the two real ROS fixtures.

## Run on the laptop

```zsh
cd ~/f1tenth_sim_seed_check
git fetch origin engineering/autonomous-mission-20261029
git merge --ff-only origin/engineering/autonomous-mission-20261029
zsh scripts/run_full_autonomy.zsh
```

The command creates a fresh map and route, then requests three estimated-pose laps. It prints the output folder and uploadable ZIP on completion or failure. A stop is a failed attempt unless the stage is `COMPLETED_ESTIMATED_LAPS`; screenshots or small CTE alone do not establish success. This remains simulation-only.
