# F1TENTH UJ — VESC + IMU + EKF simulation prototype

**Scope:** Simulation only; no physical VESC, IMU or Jetson required. These scripts are deliberately outside the existing working ROS control package; the original Pure Pursuit controller stays unchanged.

## Goal

Demonstrate a reusable state estimation pipeline:

```text
simulator /ego_racecar/odom  [privileged: SENSOR EMULATOR ONLY]
            |
         sim_vesc_imu.py    [noise/bias -> measurements; no truth pose passed]
           /  \
/sim/vesc/erpm  /sim/imu/data_raw
/sim/vesc/wheel_odom  -----+
                           |
                    robot_localization EKF
                             |
                 /odometry/filtered
                  fusion_odom -> fusion_base
                             |
             evaluate_fusion.py  <- simulator truth (EVALUATOR ONLY)

oracle Pure Pursuit (/ego_racecar/odom -> /drive) runs separately to move car.
```

This is *local odometry* (continuous state estimate), not map localization; it cannot independently correct long-term position drift. The IMU message supplies gyro yaw rate; it intentionally marks accelerometer/orientation unavailable. The wheel odometry message supplies velocity only. The raw simulated ERPM topic is for diagnostics; a later physical VESC converter should replace the simulator adapter.

## Install on Ubuntu ROS 2 Humble (one-time)

```bash
sudo apt update
sudo apt install -y ros-humble-robot-localization
```

Use `/opt/ros/humble/setup.zsh` through the workspace's `scripts/sim_env.zsh`. Run `ros2 pkg executables robot_localization` to verify `ekf_node` exists.

## Exact terminals (all separate)

**0. Before starting:** Stop old RF2O, SLAM Toolbox, any bicycle-model odometry, racing, and accuracy monitor nodes. Keep this a clean experiment. Extract the provided archive into `~/f1tenth_sim`.

**1. Simulator:**

```bash
cd ~/f1tenth_sim
zsh scripts/run_sim.zsh
```

**2. Sensor emulator:**

```bash
cd ~/f1tenth_sim
source scripts/sim_env.zsh
python3 tools/fusion/sim_vesc_imu.py --ros-args --params-file config/fusion_sim_sensors.yaml
```

**3. EKF:**

```bash
cd ~/f1tenth_sim
source scripts/sim_env.zsh
ros2 run robot_localization ekf_node --ros-args --params-file config/fusion_ekf.yaml
```

**4. Evaluator (run while stationary, then leave open):**

```bash
cd ~/f1tenth_sim
source scripts/sim_env.zsh
python3 tools/fusion/evaluate_fusion.py
```

Wait for `Initial reference captured; start the racing controller`.

**5. Race controller (still oracle pose; safety baseline):**

```bash
cd ~/f1tenth_sim
source scripts/sim_env.zsh
python3 src/f1tenth_control/f1tenth_control/racing_pure_pursuit.py \
  --ros-args \
  -p waypoints_file:="$HOME/f1tenth_sim/reference/generated/uj_technical_slam_01_registered.csv" \
  -p speed_profile_file:="$HOME/f1tenth_sim/reference/generated/uj_technical_slam_01_speed_profile.csv" \
  -p results_file:="$HOME/f1tenth_sim/results/fusion/racing_oracle.csv" \
  -p use_speed_profile:=true \
  -p speed_scale:=1.0 \
  -p lookahead_distance:=0.5
```

Run **one lap**, stop racing via `Ctrl+C`, then stop evaluator via `Ctrl+C`. Record its printed evaluation and CSV in `results/fusion/fusion_evaluation.csv`.

**6. Topic check (optional):**

```bash
cd ~/f1tenth_sim
source scripts/sim_env.zsh
ros2 topic hz /sim/vesc/wheel_odom
ros2 topic hz /sim/imu/data_raw
ros2 topic hz /odometry/filtered
ros2 run tf2_ros tf2_echo fusion_odom fusion_base
```

The frequency checks block until interrupted; use `Ctrl+C` between each command. Never run the two odometry publishers on the same frame edge.

## Configuration and calibration

- `wheel_diameter_m=0.1016` is the **simulator** wheel diameter (model radius 0.0508 m), not the physical vehicle's 65 mm diameter. Update for the real robot before converting ERPM.
- `gear_ratio=4.092`, `motor_pole_pairs=7`, and `ERPM = v / (pi*D) * gear_ratio * pole_pairs * 60`. These parameters are simulation conversion constants; confirm the mechanical ratio and VESC telemetry meaning on hardware.
- `vesc_speed_scale` injects a multiplicative error (1.005 = +0.5%); `vesc_erpm_noise_std` injects electrical RPM noise.
- `gyro_bias_rad_s` injects gyro zero-offset; `gyro_noise_std_rad_s` injects random noise.
- Set `seed` to a fixed value for repeatable sensor noise; runs may still differ due to scheduling.
- `fusion_odom -> fusion_base` is the EKF's TF. The simulator's `map -> ego_racecar/base_link` remains a separate TF tree, so the EKF does not receive a global pose correction.
- Both EKF measurement masks contain **only** `vx` (wheel, index 6) and `vyaw` (IMU, index 11) in the documented `[x,y,z,roll,pitch,yaw,vx,vy,vz,vroll,vpitch,vyaw,ax,ay,az]` ordering.
- Covariances are nonzero; pose and unmodeled IMU fields are not selected for fusion.

## Acceptance criteria

Do not define success merely as the EKF node starting. Validate:

1. Correct topics and unique TF ownership.
2. Stable `/odometry/filtered` output at about 30 Hz while simulated sensors run.
3. No abrupt estimated-position jumps during the test lap.
4. Record position RMSE, max drift, yaw error and the 95th-percentile error; compare at least two noise settings and three seeds.
5. Repeat at higher speed **only after** low-speed controlled tests pass. Nothing in this prototype justifies controlling the physical vehicle.

## Next increment

Once EKF velocity + yaw-rate fusion works, replace the simulator adapter with actual VESC and IMU drivers, keeping the same odometry/IMU input contracts. Then evaluate AMCL/particle-filter global localization on the saved `uj_technical_slam_01.yaml` occupancy map and add robust `map -> fusion_odom` TF corrections. Map-frame trajectory and pose must match before ever routing the estimated pose to Pure Pursuit.
