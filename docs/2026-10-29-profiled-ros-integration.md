# Profiled saved-map ROS/Gym integration — experimental

**Status:** implemented in the engineering branch; **NOT yet tested in a live ROS/Gym lap**.
This is intentionally separate from the previously working (slow) saved-map
launcher and **must not be deployed on physical hardware**.

## Prerequisites

- Have the original simulator running with **no other /drive publisher**.
- The October 28 saved map and lap seed are already committed in this branch.
- The improved CSV exists in the original laptop workspace at:
  \`~/f1tenth_sim/results/missions/20261028_112407/new_track_body_optimized.csv\`.
- Source the **working simulator installation** from the main workspace;
  run the **new source code** from the review worktree.
- Requires Python/ROS packages: rclpy, numpy, scipy, PyYAML, Pillow,
  nav2_amcl, nav2_map_server, robot_localization, ackermann_msgs, std_srvs.

## Minimal software-only checks (before driving)

\`\`\`bash
cd ~/f1tenth_sim_autonomy_audit
git fetch origin
git switch --detach origin/engineering/swept-path-safety-20261029
python3 -m py_compile tools/navigation/{profiled_amcl_drive,racing_policy,trajectory_safety}.py \
  tools/mission/run_profiled_saved_map.py
python3 tests/test_racing_policy.py
python3 tests/test_trajectory_safety.py
\`\`\`

The path profile test uses the committed 377-point fixture. The geometry
optimizer has already been tested against the Oct 28 saved map.

## Launch: one genuine integrated simulator trial

**Terminal 1** (main workspace and existing simulator only):

\`\`\`bash
cd ~/f1tenth_sim
zsh scripts/run_sim.zsh
\`\`\`

**Terminal 2** (source the main environment, run code from review worktree):

\`\`\`bash
cd ~/f1tenth_sim
source scripts/sim_env.zsh
python3 -u ~/f1tenth_sim_autonomy_audit/tools/mission/run_profiled_saved_map.py
\`\`\`

The launcher first checks the complete body against the saved map (minimum
nominal body margin 0.15 m), then starts independent VESC-like/IMU-like
measurements, EKF, scan relay, local scan-to-map start estimate, map server,
AMCL and the new profiled controller. Explicit operator arming is required.
The new controller is the **only drive command publisher**.

The actual commanded speed ramps toward **min(per-waypoint speed plan,
0.40 m/s)**; it is not fixed at 0.15 m/s. It uses a speed-dependent lookahead
and a LiDAR check for obstacles intersecting the *predicted vehicle body*
along the route. The known static fixed-forward 0.50 m stop is not used.
No truth odometry feeds this controller. Failures latch zero speed.

### Acceptance criteria for this run

- No competing drive publishers; no collision event.
- At least one complete lap of 377 progress points with credible traveled
  distance; not only an index wrap.
- AMCL pose/scan and EKF odometry stay fresh.
- Estimated route CTE remains below 0.09 m.
- No predicted collision with the swept vehicle body (7 cm safety margin).
- Full run logs and \`estimated_drive.csv\` are retained in a new
  \`results/missions/profiled_YYYYMMDD_HHMMSS/\` folder **in the review worktree**.

If the controller prints **SAFETY HOLD**, it stays stopped and the launcher
exits. DO NOT re-enable it. Preserve the CSV and log to analyze the cause.

### Important remaining limitations

1. Only the **saved-map racing stage** is integrated here. End-to-end unknown
   map exploration, map quality verification and automatic safe path production
   remain unimplemented as a single robust workflow.
2. The LiDAR swept-path guard is a *prototype in-process* check, **not** an
   independent physical collision-avoidance system. It considers valid
   observed LiDAR endpoints, not full ray-based free-space certification or
   occlusion; no independent arbitration/heartbeat stops a failed node.
3. The route has 0.15 m nominal body clearance in the saved occupancy grid
   and is validated against assumed 0.58 m x 0.31 m body geometry. Pose and
   tracking uncertainty are not included in that 0.15 m figure.
4. Static margin and localization covariance gates may legitimately STOP the
   mission before a lap. Do not lower guards solely to force a success.
5. Simulation ROS processes have not been run from this workspace by the
   assistant. Expected performance at 0.40 m/s is a **test hypothesis**,
   not a demonstrated result.

## Revisions / next phase

- Integrate the optimized footprint planner into the mapping stage.
- Add independent safety command arbitration and collision/TTC guard.
- Replace fixed exploration speed with a measured-clearance adaptive policy.
- Multi-seed, multi-track, multi-lap automated integration tests.
- Hardware bringup only after sensor, actuator and e-stop calibration.
