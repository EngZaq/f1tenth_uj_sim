# Moatasem — EKF / localization / mapping experiments (branch `moatasem/ekf-experiments`)

All tests ran in the F1TENTH Gym simulation, ROS 2 Humble. The simulator truth was used **only** for offline evaluation, never for control, localization or mapping. Every result below is a simulation result: the noise values (R, Q) must be measured again on the real car.

## Final decisions

| Topic | Decision |
|---|---|
| EKF for racing and mapping | **Config C**: `config/fusion_ekf_moatasem.yaml` (VESC vx + IMU gyro vyaw) |
| Rejection gates | **None** (a 3σ gate locked out good VESC/gyro data after fast transients) |
| rf2o laser odometry | **Not used**: no gain with AMCL or for mapping, and it costs ~42–46 % of a CPU core. Kept as an option. |
| slam_toolbox loop closing | **Off** (team default). On, it accepts a false closure near the start (1.2–1.7 m, 12–24°) and bends the map. |
| SLAM correction audit | Run `tools/validation/mapping_correction_audit.py` with **`--max-gap-s 2.0`** |
| Localizer (Test 4) | **Nav2 AMCL + EKF C**, AMCL `sigma_hit 0.05`, `z_rand 0.1` (`config/fusion_amcl_moatasem_A3.yaml`); the F1TENTH particle filter is not used (less accurate, 1 core of CPU, 4/5 guard stops in closed loop) |

## Issues for the team

1. **The correction audit misses loop-closure jumps with its default 0.10 s gap.** slam_toolbox stops publishing `slam_map → fusion_odom` while it optimises the graph, so the jump falls across a gap longer than 0.10 s and is skipped. In Test 3 the default audit reported only 0.07 m / 2.2°; with `--max-gap-s 2.0` it reported 1.56–1.72 m / ~24° and flagged a violation. The 9 loop-off runs give the same numbers with either gap. Suggested fix: default `--max-gap-s` to 2.0.
2. **The mapping mission accepts and plans on broken maps.** `tools/mission/capture_map_pose.py` only refuses a SLAM jump larger than 2 m. In Tests 3 and 3b, missions with a 1.2–1.7 m false loop closure (wall RMS 0.44–0.49 m) still reached `COMPLETED_MAPPING_AND_PLANNING` and produced a route. Suggested fix: run the correction audit (2 s gap) and/or a wall-consistency check before planning, and stop on a jump above ~0.5 m / 5°.

## My files (vs `origin/main`)

**ACTIVE** = part of the final setup. **EXPERIMENT RECORD** = used only in a finished test, kept so the result can be reproduced. Rows marked "team file" change an existing file but keep its default behaviour.

| File | Status | What it is | Test(s) |
|---|---|---|---|
| `config/fusion_ekf_moatasem.yaml` | ACTIVE | main EKF "C": VESC vx + gyro vyaw, no gates, Q_vx 0.05, `smooth_lagged_data` | all |
| `config/fusion_sim_sensors_calibrated.yaml` | ACTIVE | sim sensor noise after calibration + measured R | EKF, AMCL shadow, closed loop |
| `config/fusion_amcl_moatasem.yaml` | ACTIVE | copy of the team AMCL parameters (unchanged values) | AMCL shadow, closed loop |
| `tools/localization/amcl_shadow.launch.py` | ACTIVE | AMCL + odometry source: `odom_source:=ekf2` (C) / `ekf` (with rf2o) / `vesc` | AMCL shadow, closed loop |
| `tools/localization/sim_vesc_to_odom.py` | ACTIVE | raw-VESC odometry with optional steering bias/noise (the A / A_play baselines) | AMCL shadow, closed loop |
| `tools/localization/amcl_shadow_sampler.py` | ACTIVE | live TF/truth/covariance sampler | AMCL shadow |
| `tools/navigation/profiled_amcl_drive_moatasem.py` | ACTIVE | copy of the team controller with `odom_topic`, `uncertainty_slowdown_start_m`, `max_allowed_speed_mps` | closed loop |
| `tools/validation/evaluate_map_walls.py` | ACTIVE | saved SLAM map vs original track walls: wall error, start/finish doubling | Tests 2 / 3 / 3b |
| `tools/fusion/sim_vesc_imu.py` (team file) | ACTIVE | `reported_vx_variance` / `reported_gyro_variance` (-1 = team formula) | all |
| `tools/fusion/evaluate_fusion.py` (team file) | ACTIVE | `truth_reference` (`rear_axle` default; `cog` = old behaviour) | EKF tests |
| `tools/localization/fusion_scan_relay.py` (team file) | ACTIVE | `min_period_s` parameter | rf2o, all |
| `tools/mission/full_autonomy.py` (team file) | ACTIVE | `--ekf-config`, `--sensor-config`, `--with-rf2o`, `--slam-config` (defaults = team baseline) | Tests 2 / 3 / 3b |
| `tools/mission/mission_slam.launch.py` (team file) | ACTIVE | `slam_config:=` (default `config/slam_sim.yaml`) | Tests 3 / 3b |
| `config/ekf_template_ORIGINAL.yaml` | EXPERIMENT RECORD | robot_localization template, kept as a reference | EKF setup |
| `config/fusion_sim_sensors_real.yaml` | EXPERIMENT RECORD | placeholder "real car" noise | early EKF tests |
| `config/fusion_ekf_moatasem_rf2o.yaml` | EXPERIMENT RECORD | three-sensor EKF option (C1: rf2o, no gate) | rf2o R×gate, AMCL shadow B, Test 2 CR |
| `config/cov_relay_rf2o.yaml` | EXPERIMENT RECORD | rf2o R, C1 (also used by `--with-rf2o`) | rf2o R×gate, Test 2 CR |
| `config/cov_relay_rf2o_c4.yaml` | EXPERIMENT RECORD | rf2o R, C4 (data-based R + 3.7σ gate) | rf2o R×gate |
| `config/rf2o_fusion.yaml` | EXPERIMENT RECORD | rf2o node parameters (also used by `--with-rf2o`) | rf2o tests, Test 2 CR |
| `tools/fusion/cov_relay.py` | EXPERIMENT RECORD | adds R to the rf2o odometry | rf2o tests |
| `tools/fusion/fusion_three_sensors.launch.py` | EXPERIMENT RECORD | VESC + IMU + rf2o EKF launch | rf2o tests |
| `config/fusion_sim_sensors_mapping_team.yaml` | EXPERIMENT RECORD | calibrated noise, team R formula | Test 2 (T) |
| `config/fusion_sim_sensors_mapping_mine.yaml` | EXPERIMENT RECORD | calibrated noise, measured R | Test 2 (C, CR), Tests 3 / 3b |
| `config/slam_sim_loop_closing.yaml` | EXPERIMENT RECORD | `slam_sim.yaml` + `do_loop_closing: true` | Test 3 |
| `config/slam_sim_loop_closing_tight.yaml` | EXPERIMENT RECORD | loop closing on + tight search/response thresholds | Test 3b |
| `config/fusion_amcl_moatasem_A3.yaml` | ACTIVE | tuned AMCL: team values with `sigma_hit 0.05`, `z_rand 0.1` | Test 4 |
| `config/fusion_amcl_moatasem_A1.yaml`, `config/fusion_amcl_moatasem_A2.yaml` | EXPERIMENT RECORD | AMCL laser-model variants (`sigma_hit 0.05` / `z_rand 0.1` alone) | Test 4 |
| `config/particle_filter_moatasem.yaml` | EXPERIMENT RECORD | F1TENTH particle filter parameters (package values, `range_method rm`) | Test 4 |
| `tools/localization/pf_shadow.launch.py` | EXPERIMENT RECORD | particle filter + EKF C launch (needs `~/Data/pf_ws`) | Test 4 |
| `tools/localization/pf_map_odom_adapter.py` | EXPERIMENT RECORD | particle filter map->laser -> `slam_map -> fusion_odom` only; covariance re-layout; scan QoS relay | Test 4 |
| `results/missions/autonomous_20261103_114924_818822/` (map, route, lap_pose) | ACTIVE | my no-loop SLAM map (wall RMS 0.033 m) and route. It is the default map of `amcl_shadow.launch.py` and the closed-loop route. | AMCL shadow, closed loop |

## Tests and results

Summaries and plots are in `docs/moatasem_results/<test>/`.

| Test | Folder | Result |
|---|---|---|
| Early EKF tuning (gates, Q, NIS) | `step1-4/`, `early_ekf/` | Gates off; Q_vx 0.05; VESC R = 0.0009 |
| Late rf2o data (`smooth_lagged_data`) | `lag/` | Accuracy neutral, no CPU cost → kept on |
| rf2o R × gate (2×2, 5 runs each) | `rf2o_R_gate/`, `rf2o_R_gate/rear_axle/`, `summary_rear_axle_all.md` | Rear-axle grading: C1 (R 0.01 / 0.0025, no gate) best, 0.041 ± 0.005 m; the gate does not help |
| AMCL shadow (A / B / A_play / C) | `amcl_shadow/` (`summary.md`, `compare_C.md`) | AMCL + C 0.048 m vs AMCL + rf2o EKF (B) 0.045 m: no difference beyond the spread; rf2o ≈ 42 % of a core |
| Closed loop (car steered by the AMCL pose, 2 laps) | `closed_loop/` | C drives like perfect steering (A): true cross-track error 0.015 m. Raw VESC with 1° steering play is worse: 0.029 m, +1° heading error, rougher steering, one guard stop. → AMCL + EKF |
| Test 2: mapping T / C / CR (3 runs each, loop off) | `mapping/` | No difference beyond the spread (C vs T wall RMS +0.033 m, p = 0.097); map quality follows SLAM drift at the lap end (r = 0.94), not EKF drift; rf2o gives no reliable gain at ~46 % of a core |
| Test 3: loop closing ON, default (C) | `mapping_loop/` | One false closure per run (1.56–1.72 m / ~24°); wall RMS 0.477 m vs 0.100 m with loop off |
| Test 3b: loop closing ON, tight (C) | `mapping_loop_tight/` | Still a false closure every run (1.23–1.26 m / 12–20°); wall RMS 0.326 ± 0.209 m |
| Test 4: localizer check — AMCL laser model (A0–A3, shadow, 3 runs each) and F1TENTH particle filter vs tuned AMCL (shadow 3 runs, closed loop 5 runs each) | `test4_localizer/` | AMCL configs equal in true error (RMSE 0.040–0.046 m), all report σ 3–4.6× too large; A3 (σhit 0.05, z_rand 0.1) closest → tuned. Particle filter: RMSE 0.138 m, 55 heading jumps > 2°/lap, 100 % + 12 % CPU; closed loop 1/5 runs finished (4 guard stops) vs 5/5 for AMCL A3 → keep AMCL |

How to run a mapping mission with these options (defaults = the team baseline):

```
python3 tools/mission/full_autonomy.py --autonomous --start-simulator --mapping-only \
    --ekf-config config/fusion_ekf_moatasem.yaml --sensor-config config/fusion_sim_sensors_mapping_mine.yaml
```

## Raw data (not in git)

Everything is archived on the laptop in `~/Data/f1tenth_results_archive/`:

| Folder | Contents |
|---|---|
| `raw_logs/` | 2.8 GB: all rosbags (`/tf`, odometry, drive …), per-run logs, NIS/debug logs, and void runs (`cl/void`, `map/void_net`) |
| `missions/` | Test 2 / 3 / 3b mission folders by test, plus `void_net_test3b` (Wi-Fi drop); see its `README.md` for run name → folder |
| `results_fusion_csv/` | per-run `evaluate_fusion.py` CSVs (early EKF / rf2o tests) |
| `scripts/` | analysis and run scripts used for these tests (`eval_cl.py`, `analyze_*.py`, `drift_map.py`, `loop_events.py`, `run_*.sh`, …) |

The six team files with local `~/Data/f1tenth_sim` path edits stay uncommitted on purpose.
