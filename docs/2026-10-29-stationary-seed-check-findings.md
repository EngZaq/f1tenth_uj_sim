# Stationary seed check: initialization is verified; accuracy remains unresolved

Run `profiled_20261029_153559`, revision
`4f8f77491731c827972576f74331c65728a870aa`. The supplied console, initialization
record, runtime YAML, logs and 28-row full-covariance CSV were inspected.
Checksums and exact results: [metrics](evidence/profiled_20261029_153559_metrics.json).

## What worked

- The runtime config has `set_initial_pose: false`.
- The one-shot seed contains Pxx=Pyy=0.01 m^2 and yaw variance
  0.0076154355 rad^2, matching 0.10 m / 5 degree standard deviations.
- AMCL received the seed and returned a fresh pose. The console's initial
  seed-to-AMCL difference was approximately 5 mm / 0.5 degrees.
- The 28 recorded truth poses all remain at (-5.8,-2.1) m, heading 124.252 deg.
  These agree with the previous known simulator start and show no sampled reset
  to the SLAM-frame seed. No autonomous controller was started.

This verifies the initialization contract in this live run. It does not verify
absolute localization accuracy or certify that every instant between sparse
samples was stationary.

## What remains

| Quantity | Result |
| --- | --- |
| Recorded duration | 45.636 s |
| AMCL mean displacement from first estimate | 0.07764 m maximum/final |
| AMCL mean shift along the initial heading | +0.07694 m |
| AMCL mean shift sideways to the initial heading | -0.01039 m |
| AMCL heading change from first estimate | -1.603 degrees |
| Final position sigma along estimated vehicle heading | 0.11536 m |
| Final position sigma normal to estimated vehicle heading | 0.02935 m |
| Maximum EKF relative position / heading drift | 0.0131 m / 0.874 degrees |
| Median observation interval | About 1.668 s |

The position covariance was projected using the off-diagonal terms:
`variance_along = t^T Pxy t`, `variance_sideways = n^T Pxy n`. Here t/n follow
AMCL's vehicle heading, not necessarily the route tangent/normal. This is a
diagnostic projection, not a calibrated clearance bound.

The uncertainty is now much larger than the previous misleading millimeter
dispersion. The mostly longitudinal mean shift and growing longitudinal spread
are consistent with weak longitudinal constraints at this start, but do not prove
that map ambiguity is the only cause. A stationary estimator can legitimately
refine an initially incorrect hypothesis; first-pose alignment cannot determine
whether its new mean is closer to the true saved-map pose. Scan/map fit and
independent registration are still needed. It would be incorrect to claim this
run improved position accuracy, or to arm a driving test based only on it.

One AMCL startup warning reports an odometry transform approximately 24 ms behind
the callback's requested current time. AMCL continued using its startup fallback.
This is distinct from sustained TF drops; no evidence supplied establishes that
it explains the subsequent stationary mean shift.

## A diagnostic problem found and corrected

The old observer started a new `ros2 service call` process for every refresh.
Process startup plus the one-second interval produced only 28 paired samples,
with 1.53-1.84 s gaps. Consequently every interval failed the analyzer's existing
0.5 s motion-classification limit. The data were still usable for observed
endpoint displacement and covariance, but not that interval classifier.

The analyzer now reports observed truth endpoint span/displacement separately
without relabeling sparse intervals as continuously stationary or weakening its
motion filter. It also reports the last valid XY covariance in estimated vehicle
axes and rejects asymmetric or indefinite covariance matrices.

The observer now runs `stationary_localization_probe.py`, a single ROS process
with an asynchronous no-motion service client, a 0.30 s target request interval,
one outstanding request, and a response timeout. No ROS CLI process is spawned
for each refresh. It records:

- First and last captured raw and relayed scans, including angles, limits,
  timestamps, frame names and ranges. Snapshot copying is limited to 1 Hz while
  every incoming scan is counted. Nonfinite ranges remain null, not fake hits.
- Actual source-frame laser transforms for Gym and fusion, obtained from TF.
  Missing transforms remain explicit errors.
- Source arrival intervals and VESC-like wheel speed, raw gyro, true and EKF
  twist statistics; first/last sampled poses and observed displacement.
- Request/completion counts and the installed Nav2 AMCL package version when
  its package metadata can be read.

The node publishes no drive, pose or TF. Truth is evaluation-only and never
affects initialization or the request cadence. This added instrumentation is
needed because the supplied run did not contain measured scan arrays, original
laser extrinsics or raw measurement statistics. It is not an accuracy fix.

## Next observation

Keep the usual simulator running. Update the existing detached review worktree
without modifying the main workspace or deleting recorded runs:

```bash
cd ~/f1tenth_sim_seed_check
git fetch origin engineering/localization-evidence-20261029
git merge --ff-only origin/engineering/localization-evidence-20261029
source ~/f1tenth_sim/scripts/sim_env.zsh
cd ~/f1tenth_sim_seed_check
python3 -u tools/mission/run_profiled_saved_map.py \
  --localization-only --observe-seconds 45
```

Send `stationary_probe.json`, `localization_truth_audit.csv` and `amcl.log` from
the new printed run directory. Include `stationary_probe.log` if the observation
aborts. These measurements enable an offline scan/map and sensor-model check
before another driving experiment. Request cadence is different again, so do
not interpret any changed mean drift as a controlled localization-algorithm win.

## Foxglove screenshot

The screenshot's fixed/display frame is `map`, the simulator frame. The saved
localization map and AMCL pose use `slam_map`. While the localization nodes are
running, use `slam_map` and display `/fusion/localization_map`, `/fusion/scan`
and `/amcl_pose` together. No privileged map-frame connection should be added
just to align that display. After the observer exits, its processes stop; missing
fusion data or transforms in the viewer then do not diagnose live localization.

## Verification

The offline suite passes 40 tests. New checks cover one pending service request,
cadence, unavailable service, service errors/timeouts, scan snapshot throttling,
finite-value statistics, serialization, covariance projection and sparse truth
endpoints. Launcher routing still excludes autonomous-controller startup and
arming. No ROS graph or live scan model was executed in the editing environment;
the updated probe still needs the above live observation.
