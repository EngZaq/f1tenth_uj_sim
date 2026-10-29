#!/usr/bin/env python3
"""Resume previously mapped track, map-matched startup -> AMCL -> guarded driving.

Simulation ONLY. Does not invoke exploration or SLAM Toolbox. Uses no simulator
truth for controller commands or startup map scan matching. Arming is manual
unless the explicit --autonomous simulation option is selected.
Experimental profiled controller; leave original saved-map launcher untouched.
"""
from __future__ import annotations
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from sim_geometry import LASER
from mission.ros_graph import topic_info_args, wait_for_nodes_absent

ROOT=Path(__file__).resolve().parents[2]
MISSION=ROOT/'tools/mission'
RECOVERED=ROOT/'results/missions/20261028_112407'
MAP=RECOVERED/'new_track.yaml'
LAP=RECOVERED/'lap_pose.json'
PATH=Path.home()/'f1tenth_sim/results/missions/20261028_112407/new_track_body_optimized.csv'
RUN=ROOT/'results/missions'/('profiled_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
PROCS={}
HANDLES={}
DRIVE_TOPIC='/drive'


def call(args,timeout=12,check=True):
    r=subprocess.run([str(x) for x in args],capture_output=True,text=True,timeout=timeout)
    if check and r.returncode:
        raise RuntimeError('Command failed: '+repr(args)+'\n'+(r.stdout+r.stderr)[-2000:])
    return r


def start(name,args):
    log=(RUN/(name+'.log')).open('w')
    print('Starting',name,flush=True)
    p=subprocess.Popen([str(x) for x in args],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    PROCS[name]=p;HANDLES[name]=log
    time.sleep(.8)
    if p.poll() is not None:raise RuntimeError(f'{name} failed, see {RUN/(name+".log")}')


def stop(name):
    p=PROCS.pop(name,None)
    if not p:return
    if p.poll() is None:
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
    h=HANDLES.pop(name,None)
    if h:h.close()


def zero_drive():
    try:call(['ros2','topic','pub','--once',DRIVE_TOPIC,'ackermann_msgs/msg/AckermannDriveStamped',
              '{drive: {speed: 0.0, steering_angle: 0.0}}'],timeout=5,check=False)
    except Exception:pass


def closeall():
    # Argument errors/help or an aborted preflight own no ROS processes.
    if not PROCS:
        return
    try:call(['ros2','topic','pub','--once','/amcl_drive/enable','std_msgs/msg/Bool','{data: false}'],timeout=4,check=False)
    except Exception:pass
    for name in list(PROCS)[::-1]:stop(name)
    zero_drive()
    print('Logs:',RUN,flush=True)


def wait_active():
    deadline=time.monotonic()+60
    while time.monotonic()<deadline:
        if PROCS['amcl'].poll() is not None:raise RuntimeError('AMCL crashed; see amcl.log')
        a=call(['ros2','lifecycle','get','/amcl'],timeout=5,check=False)
        b=call(['ros2','lifecycle','get','/fusion_map_server'],timeout=5,check=False)
        if 'active [3]' in a.stdout and 'active [3]' in b.stdout:return
        time.sleep(.5)
    raise RuntimeError('AMCL/map server did not activate; see amcl.log')


def read_amcl_pose():
    # Stationary AMCL may require a no-motion update after activation.
    for _ in range(5):
        call(['ros2','service','call','/request_nomotion_update','std_srvs/srv/Empty','{}'],timeout=8,check=False)
        try:
            r=call(['ros2','topic','echo','--once','/amcl_pose','--field','pose.pose'],timeout=8,check=False)
            if not r.returncode: return yaml.safe_load(r.stdout.split('\n---')[0])
        except subprocess.TimeoutExpired:pass
    raise RuntimeError('No AMCL pose received while stationary; see amcl.log')


def wrap(a):return math.atan2(math.sin(a),math.cos(a))


def main():
    global MAP,LAP,PATH,RUN,DRIVE_TOPIC
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--localization-only',action='store_true',
                        help='Record stationary localization; never start or arm a controller')
    parser.add_argument('--observe-seconds',type=float,default=45.0)
    parser.add_argument('--seed-position-std-m',type=float,default=0.10)
    parser.add_argument('--seed-yaw-std-deg',type=float,default=5.0)
    parser.add_argument('--mission-dir',type=Path)
    parser.add_argument('--autonomous',action='store_true',help='Automatically prepare, wait for confidence, and arm in simulation')
    parser.add_argument('--laps',type=int,default=3)
    parser.add_argument('--speed-cap',type=float,default=.40)
    parser.add_argument('--drive-topic',choices=['/drive','/mission/drive'],default='/drive')
    args=parser.parse_args()
    if args.laps<1 or not math.isfinite(args.speed_cap) or not .05<args.speed_cap<=.8:
        parser.error('laps must be positive; speed cap must be in (0.05,0.8]')
    DRIVE_TOPIC=args.drive_topic
    if args.mission_dir:
        directory=args.mission_dir.expanduser().resolve()
        MAP=directory/'new_track.yaml';LAP=directory/'lap_pose.json'
        PATH=directory/'new_track_body_optimized.csv'
        RUN=directory/('race_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    if args.autonomous or not args.localization_only:
        metadata=MAP.parent/'laser_geometry.json'
        if not metadata.is_file() or json.loads(metadata.read_text())!=LASER:
            raise RuntimeError('Saved map has no matching laser calibration. Use the full mission to build a fresh map.')
    for key in ('observe_seconds','seed_position_std_m','seed_yaw_std_deg'):
        value=getattr(args,key)
        if not math.isfinite(value) or value<=0:
            parser.error(f'{key} must be finite and positive')
    required=[MAP,MAP.with_suffix('.pgm'),LAP,PATH,
              ROOT/'tools/fusion/sim_vesc_imu.py',ROOT/'tools/localization/fusion_scan_relay.py',
              ROOT/'tools/navigation/profiled_amcl_drive.py',ROOT/'tools/planning/vehicle_footprint_audit.py',ROOT/'tools/localization/sim_localization_audit.py',MISSION/'mission_amcl.launch.py',
              MISSION/'map_scan_seed.py',ROOT/'config/fusion_amcl.yaml',
              ROOT/'tools/localization/initialize_amcl_pose.py']
    if args.localization_only:
        required.append(ROOT/'tools/localization/stationary_localization_probe.py')
    for f in required:
        if not f.exists():raise RuntimeError(f'Missing {f}')
    RUN.mkdir(parents=True,exist_ok=True)
    wait_for_nodes_absent(call,output=RUN/'localization_handoff.json')
    sim=call(topic_info_args('/ego_racecar/odom'),check=False)
    if sim.returncode or 'Publisher count: 1' not in sim.stdout:
        raise RuntimeError('Simulator is not running. Start zsh scripts/run_sim.zsh in Terminal 1')
    drive=call(topic_info_args(DRIVE_TOPIC),check=False)
    if drive.returncode==0 and 'Publisher count: 0' not in drive.stdout:
        raise RuntimeError(f'Other drive publisher exists. Stop prior autonomous controllers!\n{drive.stdout}')
    print('\n===== PROFILED SAVED-MAP LAP (SIMULATION ONLY / ENGINEERING BRANCH) =====',flush=True)
    print('No SLAM exploration; no old finish pose assumed.\nMap:',MAP,'\nRoute:',PATH,'\nRun log:',RUN,flush=True)
    print('Running conservative vehicle-body route preflight (0.15m nominal margin)...',flush=True)
    route_audit=call(['python3',ROOT/'tools/planning/vehicle_footprint_audit.py',
                      MAP,PATH,'--required-body-margin','0.15'],timeout=40)
    print(route_audit.stdout[-850:],flush=True)
    if not args.autonomous:
        input('Press ENTER to prepare localization (vehicle stays stopped): ')
    start('sensor_emulator',['python3','-u',ROOT/'tools/fusion/sim_vesc_imu.py','--ros-args',
                             '--params-file',ROOT/'config/fusion_sim_sensors_unbiased.yaml'])
    start('ekf',['ros2','run','robot_localization','ekf_node','--ros-args',
                 '--params-file',ROOT/'config/fusion_ekf.yaml'])
    start('scan_relay',['python3','-u',ROOT/'tools/localization/fusion_scan_relay.py'])
    seed_path=RUN/'live_scan_seed.json'
    print('Estimating current pose by matching live LiDAR to recovered map (NO oracle)...',flush=True)
    seed=call(['python3','-u',MISSION/'map_scan_seed.py','--map',MAP,'--path',PATH,
              '--lap-pose',LAP,'--output',seed_path],timeout=50)
    print(seed.stdout[-1600:],flush=True)
    pose=json.loads(seed_path.read_text())
    if pose['route_distance_m']>0.20:raise RuntimeError('Startup is too far from the recovered path')
    cfg=yaml.safe_load((ROOT/'config/fusion_amcl.yaml').read_text())
    prior=cfg.pop('map_server',None)
    if 'fusion_map_server' not in cfg: cfg['fusion_map_server']=prior or {'ros__parameters':{}}
    cfg['fusion_map_server'].setdefault('ros__parameters',{})
    cfg['fusion_map_server']['ros__parameters'].update({
        'topic_name':'/fusion/localization_map','frame_id':'slam_map',
        'use_sim_time':False,'yaml_filename':str(MAP)})
    cfg['amcl']['ros__parameters'].update({
        'initial_pose':{'x':pose['x'],'y':pose['y'],'z':0.0,'yaw':pose['yaw']},
        # Humble's parameter-only initialization leaves covariance zero.
        # Supply a separate PoseWithCovarianceStamped after activation instead.
        'set_initial_pose':False,'use_sim_time':False,'global_frame_id':'slam_map',
        'odom_frame_id':'fusion_odom','base_frame_id':'fusion_base',
        'scan_topic':'/fusion/scan','map_topic':'/fusion/localization_map'})
    cfg['amcl']['ros__parameters']['update_min_d']=0.02
    cfg['amcl']['ros__parameters']['update_min_a']=0.02
    params=RUN/'recovered_map_amcl.yaml';params.write_text(yaml.safe_dump(cfg,sort_keys=False))
    start('amcl',['ros2','launch',MISSION/'mission_amcl.launch.py',f'params:={params}',f'map_yaml:={MAP}'])
    wait_active()
    initialized=call(['python3','-u',ROOT/'tools/localization/initialize_amcl_pose.py',
                      '--seed-json',seed_path,'--output',RUN/'amcl_initial_pose.json',
                      '--position-std-m',args.seed_position_std_m,
                      '--yaw-std-deg',args.seed_yaw_std_deg],timeout=25)
    print(initialized.stdout.strip(),flush=True)
    # Passive simulator-only localization audit: truth is never fed back to
    # controller, TF, map seeding, or estimator.
    start('localization_audit',['python3','-u',
          ROOT/'tools/localization/sim_localization_audit.py',
          '--output',RUN/'localization_truth_audit.csv'])
    live=read_amcl_pose()
    x,y=live['position']['x'],live['position']['y']
    q=live['orientation'];yaw=math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
    error=math.hypot(x-pose['x'],y-pose['y']);yawerr=abs(wrap(yaw-pose['yaw']))
    print(f'AMCL first pose=({x:.3f},{y:.3f},{math.degrees(yaw):.1f}deg)'
          f' | scan seed delta={error:.3f}m/{math.degrees(yawerr):.1f}deg',flush=True)
    if error>0.4 or yawerr>math.radians(25):
        raise RuntimeError('AMCL disagrees with scan-matched startup: refuses to arm')
    if args.localization_only:
        print(f'LOCALIZATION ONLY: observing for {args.observe_seconds:.1f}s. '
              'No autonomous controller is started; no arm request is sent.',flush=True)
        start('stationary_probe',['python3','-u',
              ROOT/'tools/localization/stationary_localization_probe.py',
              '--duration',args.observe_seconds,'--refresh-period',0.30,
              '--output',RUN/'stationary_probe.json'])
        deadline=time.monotonic()+args.observe_seconds+15.0
        while PROCS['stationary_probe'].poll() is None:
            if time.monotonic()>deadline:
                raise RuntimeError('Stationary probe exceeded its observation deadline')
            for name in ('amcl','ekf','scan_relay','sensor_emulator','localization_audit'):
                if PROCS[name].poll() is not None:
                    raise RuntimeError(f'{name} exited during stationary observation')
            time.sleep(.1)
        if PROCS['stationary_probe'].returncode:
            raise RuntimeError('Stationary probe failed; see stationary_probe.log')
        print('STATIONARY OBSERVATION COMPLETE; review truth audit before driving.',flush=True)
        return
    if args.autonomous:
        ready=call(['python3','-u',ROOT/'tools/localization/wait_amcl_ready.py',
                    '--output',RUN/'localization_ready.json'],timeout=55)
        print(ready.stdout,flush=True)
    start('autonomous_drive',['python3','-u',ROOT/'tools/navigation/profiled_amcl_drive.py',
        '--ros-args','-p',f'waypoints_file:={PATH}',
        '-p',f'results_file:={RUN/"estimated_drive.csv"}',
        '-p',f'start_x:={pose["x"]}','-p',f'start_y:={pose["y"]}',
        '-p',f'start_yaw_rad:={pose["yaw"]}',
        '-p','start_position_tolerance_m:=0.45',
        '-p','start_heading_tolerance_deg:=25.0',
        '-p',f'speed_cap_mps:={args.speed_cap}','-p','max_path_cte_m:=0.09',
        '-p','swept_margin_m:=0.07',
        '-p',f'auto_stop_laps:={args.laps}','-p',f'drive_topic:={DRIVE_TOPIC}'])
    time.sleep(2)
    if PROCS['autonomous_drive'].poll() is not None:raise RuntimeError('Controller crashed, see autonomous_drive.log')
    print('\n=== READY, DISABLED / SPEED 0 ===\n'
          'Map and Feasible Route loaded; scan-matched initialization passed.\n'
          'Stops on stale localization/scan, CTE>0.09m, predicted swept-body obstacles or collision.\n'
          'SIM ONLY. Inspect vehicle position before arming. Ctrl+C cancels.',flush=True)
    if not args.autonomous:
        input('Press ENTER to ARM profiled laps (simulation only): ')
    call(['ros2','topic','pub','--once','/amcl_drive/enable','std_msgs/msg/Bool','{data: true}'],timeout=15)
    print('=== PROFILED AMCL DRIVING — NO SIM GROUND TRUTH FOR STEERING ===',flush=True)
    log=RUN/'autonomous_drive.log';offset=0
    deadline=time.monotonic()+max(600,args.laps*400)
    while time.monotonic()<deadline:
        if PROCS['autonomous_drive'].poll() is not None:raise RuntimeError('Controller exited, see autonomous_drive.log')
        with log.open(errors='replace') as f:
            f.seek(offset);chunk=f.read();offset=f.tell()
        for line in chunk.splitlines():
            if any(token in line for token in ('DRIVE:','STOP:','ARMED:','LAP','SAFETY HOLD:')):
                print(line.strip(),flush=True)
        if f'ESTIMATED LAP {args.laps} COMPLETE' in chunk:
            (RUN/'race_result.json').write_text(json.dumps({
                'completed_estimated_laps':args.laps,'requested_laps':args.laps,
                'independent_truth_acceptance':'not evaluated by controller'},indent=2)+'\n')
            print('REQUESTED ESTIMATED LAPS COMPLETE; commanded STOP. Log:',RUN,flush=True)
            return
        if 'SAFETY HOLD:' in chunk:
            raise RuntimeError('Guarded controller stopped: see '+str(log))
        time.sleep(.7)
    raise RuntimeError('Timeout; see autonomous_drive.log')


if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:print('\nOperator stopped mission.',flush=True)
    except Exception as exc:
        print('\nRESUME STOPPED:',str(exc),file=sys.stderr,flush=True)
        sys.exit(1)
    finally:closeall()
