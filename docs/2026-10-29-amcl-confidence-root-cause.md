# 2026-10-29: AMCL confidence growth at waypoint 26 — diagnostic plan

The latest profiled saved-map mission reached waypoint 26 at commanded 0.40 m/s
with very small **estimated** CTE (typically <0.01 m), then entered a stationary
confidence pause and timed out after 4 seconds.

Observed AMCL position std on the driving controller:
0.008 m (waypoint 1), 0.034 m (10), 0.055 m (18), 0.077 m (26).
No evidence in these logs establishes *true* position error. Estimated CTE
cannot independently verify localization, because it uses the same TF.

The previous confidence gate used **world-axis** covariance
sqrt(max(Pxx,Pyy)), not route-relative covariance. The updated controller
logs both sigma_across and sigma_along obtained by projecting the full
xy covariance matrix onto the nearest racing-route normal/tangent. The
existing gate is **not relaxed** until these projections are validated.

The previously introduced "recover below 0.8 times soft std" hysteresis
required <0.064m, which measured stationary AMCL never achieved. Increasing
the timeout without changing the filter is not a localization fix.

A subscriber-only ROS node logs synchronized simulator ground-truth pose,
AMCL map pose, and EKF pose in a separate CSV. It derives a rigid 2D
alignment from the first synchronized truth/estimate sample and measures
subsequent *relative drift*. The node **publishes nothing** and has no
connection into the controller, map seed or filter. It cannot validate the
absolute accuracy of the original scan seed.

### Display correct frame in Foxglove

The screenshot showed "Fixed frame: map", which belongs to the simulator.
The recovered SLAM map and AMCL output instead use "slam_map". For AMCL
visual debugging set the 3D fixed frame to **slam_map**, enable
/fusion/localization_map, /amcl_pose, /fusion/scan and relevant TF.
This UI configuration alone has no effect on estimation or control.

### Update local engineering worktree

\`\`\`bash
cd ~/f1tenth_sim_autonomy_audit
git fetch origin
git switch --detach origin/engineering/swept-path-safety-20261029
\`\`\`

### When running the next *instrumented* simulation

Only run after reviewing the safety limitations of the existing
controller. The saved-map mission remains capped at 0.40 m/s and its
current confidence gate is unchanged, so another stop at a similar
waypoint is possible and should NOT be treated as a successful lap.

After run completion:

\`\`\`bash
cd ~/f1tenth_sim_autonomy_audit
LATEST=$(ls -dt results/missions/profiled_* | head -1)
python3 tools/localization/summarize_sim_localization_audit.py \
  "$LATEST/localization_truth_audit.csv" \
  --controller-csv "$LATEST/estimated_drive.csv"
\`\`\`

### Decision rules

- **Small truth-relative drift, large AMCL world-axis sigma, small
  cross-track sigma:** investigate and replace scalar world-axis guard
  with a bounded error-envelope check, requiring map clearance and
  controller tracking error as part of the budget. Do NOT blindly
  raise the maximum std threshold.
- **Large AMCL drift while EKF relative drift is small:** investigate
  AMCL scan likelihood, scan matching, motion model and saved map
  distortions. Retune against recorded data, not lap completion.
- **Large EKF and AMCL drift together:** inspect wheel/IMU measurement
  timing, bias and TF extrinsics before localization filter tuning.
- **Significant truth-relative error while AMCL reported low sigma:**
  the filter is inconsistent, and covariance-only driving gates are
  not adequate. Require an independent localization consistency check.

The vehicle-body route has only 0.150 m nominal clearance. None of these
cases justify higher speed or moving to hardware without repeatable
independent validation.
