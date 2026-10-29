# Autonomous simulation architecture review — 2026-10-29

This is an engineering review and **first implementation step**, not a claim that
the complete autonomous mission is fixed or has completed an end-to-end lap.

## Compared codebases

- [F1TENTH system](https://github.com/f1tenth/f1tenth_system):
  SLAM Toolbox mapping, VESC/odometry, and prioritized joystick/navigation
  command multiplexing.
- [F1TENTH particle filter](https://github.com/f1tenth/particle_filter):
  map-based Monte Carlo localization with a LiDAR likelihood model.
  Its repository documents RangeLibc and particle resampling.
- [University of Waterloo F1TENTH workspace](https://github.com/CL2-UWaterloo/f1tenth_ws):
  Pure Pursuit with speed-sensitive lookahead and speed profiles;
  independent emergency-braking logic based on time-to-collision.
- [Our simulator](https://github.com/EngZaq/f1tenth_uj_sim):
  custom follow-gap mapping, map skeleton route extraction, EKF,
  AMCL and a constant-speed saved-map Pure Pursuit controller.

## Confirmed structural weaknesses

1. A single fixed forward-cone obstacle threshold does not model the car's
   intended turning trajectory or footprint. The same bend repeatedly
   triggers front-range stops even with small estimated CTE.
2. The skeleton extractor validates a **point** along the centerline, not the
   physical car's rectangular swept footprint plus localization/tracking error.
3. The saved-map controller ignores the generated speed profile and uses a
   constant 0.15 m/s; racing fast requires curvature/clearance/braking limits.
4. Restart/map registration relies on a local scan-match pose *hypothesis*;
   estimator confidence and independent pose-error checks need stronger gates.
5. The mission declares first-lap closure mainly from an odometry return
   distance; this is not proof that the entire loop was mapped reliably.
6. The map-specific manual curvature correction is not integrated into
   automatic map-to-racing-line generation.
7. The drive path is published directly, whereas a real vehicle needs a
   command mux with independent safety priority, physical e-stop and watchdog.

## Offline measurements — saved Oct 28 map

Source: uploaded `new_track.pgm/.yaml` and the 377-point feasible map-frame
route. Pixels: 0.05 m. Point-to-obstacle minimum ~0.304 m. Assuming a 0.58 m
long, 0.31 m wide vehicle (rear-axle reference), minimum estimated distance
from the complete rectangular footprint to unknown/occupied grid cells:
**0.112 m** at waypoint 44. This is only about 11 cm of nominal body clearance;
pose/tracking error eats into it. All sampled nominal body positions remained
in observed free cells. Dimensions must be verified against the active simulator
vehicle configuration and physical vehicle before using this as a hard gate.

An **idealized** kinematic bicycle model using path geometry and an ideal
pose source completed an entire 37.6 m circuit in offline trials at 0.3,
0.5, 0.8 m/s (lookahead 0.5–0.65 m; peak CTE around 0.06 m).
These are not ROS/Gym results and do not include sensor latency, tire dynamics,
localization jumps, or independent collision checking.

## Implementation and immediate limitations

- `tools/planning/vehicle_footprint_audit.py` rejects a nominal map-frame
  route with insufficient full-body clearance or excessive steering angle.
- `tools/navigation/trajectory_safety.py` provides a **prototype** map-frame
  predicted-footprint check against LiDAR endpoints for a moving car.
- `tests/test_trajectory_safety.py` checks that a measured obstacle on a
  straight predicted trajectory blocks and a distant lateral obstacle does not.
- These are **not yet wired into the live ROS controller**, and the prototype
  must not be used by itself to replace a hardware or simulator emergency stop.
  It does not yet ray-trace unknown/open-space beam coverage, validate scans
  across the entire swept envelope, or handle multimodal localization failure.
- The existing mission remains unchanged on main.

### Preflight on the existing saved map

```bash
cd ~/f1tenth_sim
python3 tools/planning/vehicle_footprint_audit.py \
  results/missions/20261028_112407/new_track.yaml \
  results/missions/20261028_112407/new_track_mission_forward_feasible.csv \
  --required-body-margin 0.10
python3 tests/test_trajectory_safety.py
```

A pass is **nominal geometry only**. Require additional body margin for estimated
localization and tracking uncertainty before enabling high-speed runs.

## Remaining integration sequence

1. Determine exact vehicle geometry, LiDAR extrinsics, brakes and steering rate.
2. Integrate full-body swept planning and a curvature/clearance-based speed
   envelope into the map-to-route generator; do not rely on manually edited
   waypoints.
3. Validate path and speed against simulation replay with multiple seeds.
4. Implement a separate safety arbiter with timeout, collision, reachable
   forward sweep and TTC/hard braking. Reject blind sectors conservatively.
5. Integrate speed-dependent lookahead and tested speed profile into estimated
   pose controller, with jerk/steering-rate limiting and lap-progress tracking.
6. Verify SLAM map quality and geometric loop closure before switching to AMCL.
7. Demonstrate repeated complete ROS/Gym laps with low CTE and no collisions,
   including forced LiDAR/odometry dropouts. Only then increase speed.
8. Transfer ROS interfaces to Jetson hardware and validate hardware e-stop,
   measured steering/ERPM and physical sensor calibration.

No safety limit should be reduced just to get a completed lap.


## 2026-10-29 implementation update: map-specific body clearance optimization

The initial full-body clearance audit was confirmed by the developer's local test:
0.1118034 m minimum on the 377-point recovered track, limiting waypoint 44;
steering demand 20.1853 degrees vs 24.0012-degree limit. A required margin
0.10 m passed; 0.15 m correctly failed (exit status 2).

The new \`tools/planning/optimize_vehicle_route.py\` searches for smooth,
periodic local path offsets around the most restrictive body-clearance
section, recomputes headings and curvature and checks both conservative
occupancy-cell clearance and sub-cell interpolated clearance. Eight
intermediate positions per segment are also checked, not only waypoint poses.
No output is written when the requested margin fails.

An offline run against the actual recovered map and feasible CSV gave:
- Prior conservative nominal body clearance: 0.112 m at waypoint 44.
- Optimized candidate: approximately 0.150 m conservative minimum clearance,
  including intermediate body poses; peak steering about 19.75 degrees.
- Largest local lateral route adjustment: about 0.017 m.
- Generated simulation-only speed envelope: approximately 0.20-0.80 m/s.

This is **not** sufficient uncertainty reserve for high-speed autonomous
operation or the physical vehicle: 0.150 m is a *nominal geometric bound*.
The 0.05 m map grid, assumed car dimensions, pose drift, tracker CTE,
obstacle-sensing uncertainty, actuator behavior, and dynamic tracking must
all be accounted for before deployment. The saved-map controller still uses
its previous constant speed and old front-cone guard: it does not consume
this candidate speed profile yet.

Generate an offline candidate from the engineering branch:

\`\`\`bash
python3 tools/planning/optimize_vehicle_route.py \
  results/missions/20261028_112407/new_track.yaml \
  results/missions/20261028_112407/new_track_mission_forward_feasible.csv \
  --margin 0.15 \
  --out results/missions/20261028_112407/new_track_body_optimized.csv
\`\`\`

Do not rename the candidate over the previously validated route. Integration
with a predictive safety arbiter and a speed-profile-aware controller, followed
by genuine repeated ROS/Gym lap tests, remains unfinished.
