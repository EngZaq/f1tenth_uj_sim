#!/usr/bin/env zsh
# Run the already-built SIMULATOR (different terminal) first.
# Then this orchestrates virtual sensors -> EKF -> LiDAR relay -> AMCL ->
# SIMULATION-ONLY closed-loop controller, and waits to be explicitly enabled.
emulate -L zsh
setopt ERR_EXIT
SCRIPT_DIR="${0:A:h}"
ROOT="${F1TENTH_SIM_WS:-$HOME/f1tenth_sim}"
cd "$ROOT" || exit 1
source scripts/sim_env.zsh || exit 1

# Do not take over a running oracle or any other /drive source.
if ! ros2 topic info /ego_racecar/odom >/dev/null 2>&1; then
  print -u2 'ERROR: Simulator is not running. Run zsh scripts/run_sim.zsh in another terminal first.'
  exit 2
fi
PUBCOUNT=$(ros2 topic info /drive | sed -n 's/Publisher count: //p' | tr -d '[:space:]')
if [[ -n "$PUBCOUNT" && "$PUBCOUNT" != '0' ]]; then
  print -u2 "ERROR: Existing /drive publisher(s)=$PUBCOUNT. Stop oracle/other controllers FIRST."
  exit 3
fi

LOG_DIR="$ROOT/results/localization/autonomous_run_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
typeset -a PIDS
PIDS=()
CLEANED=0
function cleanup() {
  if (( CLEANED )); then return; fi
  CLEANED=1
  trap - INT TERM EXIT
  print '\nSending zero-speed command / shutting down the autonomous stack...'
  # Give the controller time to process a disable event and publish zero speed.
  timeout 3 ros2 topic pub --once /amcl_drive/enable std_msgs/msg/Bool '{data: false}' >/dev/null 2>&1 || true
  sleep 0.2
  for pid in $PIDS; do
    /bin/kill -INT -- "-$pid" 2>/dev/null || /bin/kill -INT "$pid" 2>/dev/null || true
  done
  print "ROS logs: $LOG_DIR"
  print 'Leave physical actuators disconnected: this launch is SIMULATION ONLY.'
}
trap 'cleanup' EXIT INT TERM

function start_bg() {
  local label="$1"
  shift
  print "Starting $label ..."
  setsid "$@" >"$LOG_DIR/$label.log" 2>&1 &
  PIDS+=( $! )
}

start_bg virtual_sensors python3 -u tools/fusion/sim_vesc_imu.py --ros-args --params-file config/fusion_sim_sensors_unbiased.yaml
sleep 1
start_bg ekf ros2 run robot_localization ekf_node --ros-args --params-file config/fusion_ekf.yaml
sleep 1
start_bg scan_relay python3 -u tools/localization/fusion_scan_relay.py
sleep 1
start_bg amcl ros2 launch tools/localization/fusion_amcl.launch.py

print 'Waiting for /amcl and /fusion_map_server to activate...'
active=0
for i in {1..45}; do
  AMCL_STATE=$(timeout 3 ros2 lifecycle get /amcl 2>/dev/null || true)
  MAP_STATE=$(timeout 3 ros2 lifecycle get /fusion_map_server 2>/dev/null || true)
  if [[ "$AMCL_STATE" == *'active [3]'* && "$MAP_STATE" == *'active [3]'* ]]; then
    active=1
    break
  fi
  sleep 1
done
if (( ! active )); then
  print -u2 'ERROR: AMCL/map server did not activate. Last log lines:'
  tail -30 "$LOG_DIR/amcl.log"
  exit 4
fi

start_bg autonomous_drive python3 -u tools/localization/amcl_autonomous_drive.py \
    --ros-args -p lookahead_distance:=0.8 -p speed_cap_mps:=0.35 -p auto_stop_laps:=1
sleep 2
if ! kill -0 "${PIDS[-1]}" 2>/dev/null; then
  print -u2 'ERROR: Controller crashed on startup.'
  tail -40 "$LOG_DIR/autonomous_drive.log"
  exit 5
fi
print '\n*******************************************************'
print '* AUTONOMOUS SIM READY -- vehicle is STOPPED/DISABLED *'
print '* AMCL + EKF drive /drive, NOT ground-truth odometry.   *'
print '* Speed cap 0.35 m/s; auto-stop after estimated lap 1. *'
print '*******************************************************'
print "Logs: $LOG_DIR"
print 'Press ENTER to ENABLE simulation-only driving; Ctrl+C to quit.'
read -r confirm
ros2 topic pub --once /amcl_drive/enable std_msgs/msg/Bool '{data: true}'
print '\nFollowing autonomous control log. Ctrl+C to STOP and exit.'
tail -n 3 -f "$LOG_DIR/autonomous_drive.log"
