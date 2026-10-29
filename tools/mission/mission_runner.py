#!/usr/bin/env python3
"""One orchestration script: unknown-track LiDAR exploration -> SLAM -> map and
centerline -> AMCL localization -> Pure Pursuit, SIMULATION ONLY.

Stages have explicit user arming. Existing simulator must be running separately.
Does not ever subscribe to /ego_racecar/odom; only simulator sensor emulator does.
"""
import json
import os
import pathlib
import signal
import subprocess
import sys
import time
import yaml
from datetime import datetime

ROOT=pathlib.Path.home()/'f1tenth_sim'
MISSION=ROOT/'tools/mission'
LOGDIR=ROOT/'results/missions'/datetime.now().strftime('%Y%m%d_%H%M%S')
LOGDIR.mkdir(parents=True,exist_ok=True)
MAP_PREFIX=LOGDIR/'new_track'
PATH_PREFIX=LOGDIR/'new_track'
LAP_FILE=LOGDIR/'lap_pose.json'
PROCS={}
FILES={}


def banner(s): print(f'\n=== {s} ===',flush=True)


def run(args,timeout=30,check=True):
    args=[str(z) for z in args]
    result=subprocess.run(args,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                          text=True,timeout=timeout)
    if check and result.returncode:
        raise RuntimeError(f'Command failed ({result.returncode}): {" ".join(args)}\n'
                           f'{result.stdout[-4000:]}')
    return result


def start(name,cmd):
    print(f'Starting {name}...',flush=True)
    handle=(LOGDIR/f'{name}.log').open('w')
    p=subprocess.Popen([str(v) for v in cmd],stdout=handle,stderr=subprocess.STDOUT,
                       start_new_session=True,cwd=ROOT)
    PROCS[name]=p
    FILES[name]=handle
    time.sleep(.7)
    if p.poll() is not None: raise RuntimeError(f'{name} exited early; see {LOGDIR}/{name}.log')


def terminate(name):
    p=PROCS.pop(name,None)
    if p is None:return
    try:os.killpg(p.pid,signal.SIGINT)
    except ProcessLookupError:pass
    try:p.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:os.killpg(p.pid,signal.SIGTERM)
        except ProcessLookupError:pass
        try:p.wait(timeout=3)
        except subprocess.TimeoutExpired:
            try:os.killpg(p.pid,signal.SIGKILL)
            except ProcessLookupError:pass
    handle=FILES.pop(name,None)
    if handle:handle.close()


def stop_drive():
    # CLI temporary source of zero speed; do not issue motion commands here.
    run(['ros2','topic','pub','--once','/drive','ackermann_msgs/msg/AckermannDriveStamped',
         '{drive: {speed: 0.0, steering_angle: 0.0}}'],timeout=8,check=False)


def close_all():
    try:
        run(['ros2','topic','pub','--once','/amcl_drive/enable',
             'std_msgs/msg/Bool','{data: false}'],timeout=6,check=False)
    except Exception:pass
    for name in list(PROCS)[::-1]:terminate(name)
    try:stop_drive()
    except Exception:pass
    print(f'\nMission outputs and logs: {LOGDIR}',flush=True)


def wait_for_file(p,timeout,required):
    t=time.monotonic()
    while not p.exists():
        if time.monotonic()-t>timeout:
            raise RuntimeError(f'Timed out waiting for {p.name} after {timeout}s')
        for name in required:
            if name in PROCS and PROCS[name].poll() is not None:
                tail=(LOGDIR/f'{name}.log').read_text(errors='replace')[-2500:]
                raise RuntimeError(f'Process {name} exited early:\n{tail}')
        time.sleep(.75)


def wait_amcl_active():
    for _ in range(60):
        if 'amcl' in PROCS and PROCS['amcl'].poll() is not None:
            raise RuntimeError(f'AMCL launch crashed. See {LOGDIR}/amcl.log')
        try:
            a=run(['ros2','lifecycle','get','/amcl'],timeout=4,check=False)
            b=run(['ros2','lifecycle','get','/fusion_map_server'],timeout=4,check=False)
            if 'active [3]' in a.stdout and 'active [3]' in b.stdout:return
        except subprocess.TimeoutExpired:pass
        time.sleep(.5)
    raise RuntimeError('AMCL + new map server did not activate')


def main():
    required=[ROOT/'config/fusion_ekf.yaml',ROOT/'config/fusion_amcl.yaml',
              ROOT/'tools/fusion/sim_vesc_imu.py',
              ROOT/'tools/localization/fusion_scan_relay.py',
              ROOT/'tools/localization/amcl_autonomous_drive.py',
              ROOT/'tools/exploration/exploration_controller.py',
              ROOT/'tools/planning/extract_robust_centerline.py']
    missing=[str(v) for v in required if not v.exists()]
    if missing:raise RuntimeError('Missing required project files:\n'+'\n'.join(missing))
    sim=run(['ros2','topic','info','/ego_racecar/odom'],timeout=5,check=False)
    if sim.returncode:raise RuntimeError('Start simulator first: zsh scripts/run_sim.zsh')
    drive=run(['ros2','topic','info','/drive'],timeout=5,check=False)
    if 'Publisher count: 0' not in drive.stdout:
        raise RuntimeError(f'/drive already has publisher(s)! Stop all other controllers:\n{drive.stdout}')
    banner('UNKNOWN TRACK MISSION (SIMULATION ONLY)')
    print('1. LiDAR explorer traverses unknown loop; EKF + SLAM Toolbox map the circuit.\n'
          '2. On loop closure, save map, derive closed centerline, stop SLAM.\n'
          '3. Initialize AMCL at latest SLAM-frame pose and run estimated-pose Pure Pursuit.\n'
          'Never run old oracle Pure Pursuit or the old AMCL launcher concurrently.\n'
          f'Outputs: {LOGDIR}',flush=True)
    input('Press ENTER to start unknown-track exploration (Ctrl+C cancels): ')
    start('sensor_emulator',['python3','-u',ROOT/'tools/fusion/sim_vesc_imu.py',
                             '--ros-args','--params-file',ROOT/'config/fusion_sim_sensors_unbiased.yaml'])
    start('ekf',['ros2','run','robot_localization','ekf_node','--ros-args',
                 '--params-file',ROOT/'config/fusion_ekf.yaml'])
    start('scan_relay',['python3','-u',ROOT/'tools/localization/fusion_scan_relay.py'])
    start('slam',['ros2','launch',MISSION/'mission_slam.launch.py'])
    # Do not drive until SLAM Toolbox has advertised its map publisher.
    ready=False
    for _ in range(40):
        if PROCS['slam'].poll() is not None:
            raise RuntimeError('SLAM died before map publisher became ready')
        try:
            status=run(['ros2','topic','info','/mission/slam_map'],timeout=4,check=False)
            if 'Publisher count: 1' in status.stdout:
                ready=True
                break
        except subprocess.TimeoutExpired:
            pass
        time.sleep(1)
    if not ready:
        raise RuntimeError('SLAM map publisher did not appear; see slam.log')
    time.sleep(4)  # allow initial scan processing and TF to settle
    start('lap_watcher',['python3','-u',MISSION/'mission_first_lap.py','--ros-args',
                         '-p',f'output:={LAP_FILE}'])
    start('explorer',['python3','-u',MISSION/'mission_explore.py','--ros-args',
                      '-p','enable_drive:=true','-p','speed:=0.18'])
    banner('MAPPING FIRST LAP — car moving from LiDAR only')
    print('Track progress: tail -f '+str(LOGDIR/'lap_watcher.log'),flush=True)
    wait_for_file(LAP_FILE,timeout=1000,required=('slam','explorer','lap_watcher','ekf','sensor_emulator'))
    banner('FIRST EXPLORATION LAP DETECTED')
    terminate('explorer')
    stop_drive()
    print('Holding position while SLAM closes its loop and updates occupancy map...',flush=True)
    time.sleep(6)
    banner('SAVING NEW SLAM MAP')
    # Verified Humble CLI supports -t map_topic and -f mapname.
    save=run(['ros2','run','nav2_map_server','map_saver_cli',
              '-t','/mission/slam_map','-f',MAP_PREFIX],timeout=40)
    print(save.stdout[-1500:])
    if not MAP_PREFIX.with_suffix('.yaml').exists() or not MAP_PREFIX.with_suffix('.pgm').exists():
        raise RuntimeError('Map saving did not produce both .yaml and .pgm')
    run(['python3',MISSION/'capture_map_pose.py',LAP_FILE],timeout=20)
    terminate('lap_watcher')
    terminate('slam')  # AMCL must be the ONLY map->odom TF publisher after this.
    banner('EXTRACTING CLOSED CENTERLINE AND SPEED PROFILE')
    # Use a cycle extractor that tolerates real SLAM skeleton branches and
    # checks unknown/occupied clearance. Do not fabricate unseen sections.
    measured_lap=json.loads(LAP_FILE.read_text())
    extract=run(['python3',ROOT/'tools/planning/extract_robust_centerline.py',
                 MAP_PREFIX.with_suffix('.yaml'),'--out',PATH_PREFIX,
                 '--step','0.10','--max-speed','0.35',
                 '--expected-length',str(measured_lap['distance_odom_m'])],timeout=70)
    print(extract.stdout[-2000:])
    raw=LOGDIR/'new_track_centerline.csv'
    oriented=LOGDIR/'new_track_mission_forward.csv'
    print(run(['python3',MISSION/'orient_mission_centerline.py',
               raw,LAP_FILE,oriented],timeout=20).stdout)
    pose=json.loads(LAP_FILE.read_text())
    cfg=yaml.safe_load((ROOT/'config/fusion_amcl.yaml').read_text())
    mapparams=cfg.pop('map_server',None)
    if 'fusion_map_server' not in cfg:
        cfg['fusion_map_server']=mapparams or {'ros__parameters':{}}
    cfg['fusion_map_server'].setdefault('ros__parameters',{})
    cfg['fusion_map_server']['ros__parameters'].update({
        'topic_name':'/fusion/localization_map','frame_id':'slam_map',
        'use_sim_time':False,'yaml_filename':str(MAP_PREFIX.with_suffix('.yaml'))})
    cfg['amcl']['ros__parameters'].update({
        'initial_pose':{'x':float(pose['finish_x']),'y':float(pose['finish_y']),
                        'z':0.0,'yaw':float(pose['finish_yaw'])},
        'set_initial_pose':True,
        'use_sim_time':False,'global_frame_id':'slam_map',
        'odom_frame_id':'fusion_odom','base_frame_id':'fusion_base',
        'scan_topic':'/fusion/scan','map_topic':'/fusion/localization_map'})
    amcl_cfg=LOGDIR/'mission_amcl.yaml'
    amcl_cfg.write_text(yaml.safe_dump(cfg,sort_keys=False))
    banner('SWITCHING SLAM -> AMCL (NO GROUND-TRUTH POSE)')
    start('amcl',['ros2','launch',MISSION/'mission_amcl.launch.py',
                  f'params:={amcl_cfg}',f'map_yaml:={MAP_PREFIX.with_suffix(".yaml")}'])
    wait_amcl_active()
    print('AMCL + saved map active.',flush=True)
    start('autonomous_drive',['python3','-u',ROOT/'tools/localization/amcl_autonomous_drive.py',
             '--ros-args','-p',f'waypoints_file:={oriented}',
             '-p',f'results_file:={LOGDIR/"estimated_drive.csv"}',
             '-p',f'start_x:={pose["finish_x"]}',
             '-p',f'start_y:={pose["finish_y"]}',
             '-p',f'start_yaw_rad:={pose["finish_yaw"]}',
             '-p','speed_cap_mps:=0.25','-p','lookahead_distance:=0.8',
             '-p','auto_stop_laps:=1'])
    banner('MAP, CENTERLINE, SPEED PROFILE SAVED')
    print(f'Map:      {MAP_PREFIX}.yaml / .pgm\n'
          f'Centerline:{oriented}\n'
          f'Preview:   {LOGDIR/"new_track_preview.png"}\n'
          f'Profile:   {LOGDIR/"new_track_speed_profile.csv"}\n'
          'Vehicle is currently STOPPED, AMCL-based controller DISABLED.\n'
          'Inspect the map preview in your file manager, if desired.',flush=True)
    input('Press ENTER to ENABLE slow AMCL-driven lap (Ctrl+C stops): ')
    print(run(['ros2','topic','pub','--once','/amcl_drive/enable',
               'std_msgs/msg/Bool','{data: true}'],timeout=15).stdout[-1000:])
    banner('AMCL CLOSED-LOOP DRIVING — NO ORACLE CONTROL')
    log=LOGDIR/'autonomous_drive.log'
    offset=0
    end=time.monotonic()+900
    while time.monotonic()<end:
        if PROCS['autonomous_drive'].poll() is not None:
            raise RuntimeError('Autonomous controller exited unexpectedly')
        with log.open(errors='replace') as fp:
            fp.seek(offset)
            chunk=fp.read()
            offset=fp.tell()
        for line in chunk.splitlines():
            if 'DRIVE:' in line or 'STOP:' in line or 'ARMED:' in line or 'LAP' in line:
                print(line.strip(),flush=True)
        if 'ESTIMATED LAP 1 COMPLETE' in chunk:
            banner('SUCCESS: FIRST AUTONOMOUS LAP ON NEWLY MAPPED TRACK')
            return
        if 'COLLISION LATCHED' in chunk or 'AMCL pose jump' in chunk:
            raise RuntimeError('Autonomous safety fault: see '+str(log))
        time.sleep(.7)
    raise RuntimeError('Autonomous lap timeout; inspect status log')


if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:
        print('\nStopped by user.',flush=True)
    except Exception as exc:
        print(f'\nMISSION STOPPED: {exc}',file=sys.stderr,flush=True)
        sys.exit(1)
    finally:close_all()
