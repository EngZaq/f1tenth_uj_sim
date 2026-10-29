#!/usr/bin/env python3
"""SIM ONLY: explore -> fresh map -> body-audited route -> estimated-pose laps.

--autonomous removes intermediate prompts; stage and controller guards remain.
Simulator truth is used only by the sensor emulator and passive evaluation.
"""
import argparse
import json
import math
import os
import re
import signal
import subprocess
import sys
import time
import zipfile
from datetime import datetime
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from sim_geometry import LASER
from mission.ros_graph import (LOCALIZATION_CONFLICTS, node_list_args,
                               topic_info_args, wait_for_nodes_absent)

ROOT = Path(__file__).resolve().parents[2]
MISSION = ROOT/'tools/mission'


class Mission:
    def __init__(self, folder):
        self.folder = Path(folder)
        self.folder.mkdir(parents=True,exist_ok=False)
        self.jobs, self.handles, self.events = {}, {}, []
        self.fault_file = self.folder/'watchdog_fault.json'
        self.stage = 'CREATED'

    def enter(self, stage, **details):
        self.stage = stage
        self.events.append({'stage':stage,'time_s':time.time(),**details})
        (self.folder/'mission_status.json').write_text(json.dumps({
            'stage':stage,'events':self.events,'simulation_only':True},indent=2)+'\n')
        print('\n=== '+stage+' ===',flush=True)

    def command(self, args, timeout=30, check=True):
        result = subprocess.run([str(x) for x in args],capture_output=True,text=True,timeout=timeout,cwd=ROOT)
        with (self.folder/'commands.jsonl').open('a') as fp:
            fp.write(json.dumps({'args':[str(x) for x in args],'returncode':result.returncode,
                                 'stdout':result.stdout,'stderr':result.stderr})+'\n')
        if check and result.returncode:
            raise RuntimeError('Command failed: '+repr([str(x) for x in args])+'\n'+
                               (result.stdout+result.stderr)[-3000:])
        return result

    def start(self, name, args):
        fp = (self.folder/(name+'.log')).open('w')
        self.handles[name] = fp
        self.jobs[name] = subprocess.Popen([str(x) for x in args],cwd=ROOT,
            stdout=fp,stderr=subprocess.STDOUT,start_new_session=True)
        print('Starting '+name,flush=True)

    def stop(self, name):
        process = self.jobs.pop(name,None)
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid,signal.SIGINT)
                process.wait(timeout=20 if name=='race' else 5)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid,signal.SIGTERM)
                    process.wait(timeout=3)
                except ProcessLookupError:
                    pass
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL)
                    process.wait(timeout=3)
        fp = self.handles.pop(name,None)
        if fp:
            fp.close()

    def check(self, ignore=()):
        if self.fault_file.exists():
            raise RuntimeError('Drive watchdog stopped mission: '+self.fault_file.read_text())
        for name,process in self.jobs.items():
            if name not in ignore and process.poll() is not None:
                raise RuntimeError(f'{name} exited; see {self.folder/(name+".log")}')

    def wait(self, predicate, timeout, ignore=()):
        deadline = time.monotonic()+timeout
        while not predicate():
            self.check(ignore)
            if time.monotonic()>=deadline:
                raise RuntimeError('Stage timed out: '+self.stage)
            time.sleep(.2)
        self.check(ignore)

    def zero(self):
        watchdog = self.jobs.get('watchdog')
        topic = '/mission/drive' if watchdog and watchdog.poll() is None else '/drive'
        self.command(['ros2','topic','pub','--once',topic,
            'ackermann_msgs/msg/AckermannDriveStamped',
            '{drive: {speed: 0.0, steering_angle: 0.0}}'],timeout=5,check=False)

    def close(self):
        if not self.jobs:
            return
        try:
            self.command(['ros2','topic','pub','--once','/amcl_drive/enable',
                         'std_msgs/msg/Bool','{data: false}'],timeout=3,check=False)
        except Exception:
            pass
        try:
            self.zero()
        except Exception:
            pass
        for name in list(self.jobs)[::-1]:
            if name not in ('watchdog','simulator'):
                self.stop(name)
        self.stop('watchdog')
        self.stop('simulator')

    def package(self):
        target=self.folder.with_suffix('.zip')
        with zipfile.ZipFile(target,'w',compression=zipfile.ZIP_DEFLATED) as archive:
            for file in sorted(self.folder.rglob('*')):
                if file.is_file():
                    archive.write(file,file.relative_to(self.folder))
        return target

    def stream_log(self, name, offset):
        with (self.folder/(name+'.log')).open(errors='replace') as fp:
            fp.seek(offset)
            chunk = fp.read()
            offset = fp.tell()
        if chunk:
            print(chunk, end='', flush=True)
        return offset


def run_mission(ctx, args):
    ctx.enter('PREFLIGHT')
    if not args.autonomous:
        input('Press ENTER to start exploration and the automatic stage sequence: ')
    nodes = ctx.command(node_list_args(),timeout=6).stdout.splitlines()
    if any(name in nodes for name in (*LOCALIZATION_CONFLICTS,'/simulation_drive_watchdog')):
        raise RuntimeError('A previous autonomy stack is running; stop it normally first')
    drive = ctx.command(topic_info_args('/drive'),timeout=6,check=False)
    if 'Publisher count: 0' not in drive.stdout and drive.returncode==0:
        raise RuntimeError('Another /drive publisher already owns actuation')
    def simulator_ready():
        info = ctx.command(topic_info_args('/ego_racecar/odom'),timeout=5,check=False)
        return info.returncode==0 and 'Publisher count: 1' in info.stdout
    if not simulator_ready():
        if not args.start_simulator:
            raise RuntimeError('Simulator unavailable; use --start-simulator or start it separately')
        ctx.start('simulator',['zsh',ROOT/'scripts/run_sim.zsh'])
        ctx.wait(simulator_ready,45)
    (ctx.folder/'laser_geometry.json').write_text(json.dumps(LASER,indent=2)+'\n')
    ctx.start('watchdog',['python3','-u',ROOT/'tools/navigation/drive_watchdog.py',
                         '--fault-file',ctx.fault_file])
    ctx.start('sensor_emulator',['python3','-u',ROOT/'tools/fusion/sim_vesc_imu.py',
                                '--ros-args','--params-file',ROOT/'config/fusion_sim_sensors_unbiased.yaml'])
    ctx.start('ekf',['ros2','run','robot_localization','ekf_node','--ros-args',
                     '--params-file',ROOT/'config/fusion_ekf.yaml'])
    ctx.start('scan_relay',['python3','-u',ROOT/'tools/localization/fusion_scan_relay.py'])
    ctx.start('slam',['ros2','launch',MISSION/'mission_slam.launch.py'])
    def map_available():
        try:
            result = ctx.command(['ros2','topic','echo','--once','/mission/slam_map',
                                  '--field','info.width','--qos-durability','transient_local'],timeout=5,check=False)
            widths = re.findall(r'^\s*(\d+)\s*$',result.stdout,re.MULTILINE)
            return result.returncode==0 and bool(widths) and int(widths[0])>0
        except subprocess.TimeoutExpired:
            return False
    ctx.wait(map_available,60)
    lap_file = ctx.folder/'lap_pose.json'
    ctx.start('lap_watcher',['python3','-u',MISSION/'mission_first_lap.py',
                            '--ros-args','-p',f'output:={lap_file}'])
    ctx.start('explorer',['python3','-u',MISSION/'mission_explore.py','--ros-args',
                         '-p','enable_drive:=true','-p',f'speed:={args.explore_speed}',
                         '-r','/drive:=/mission/drive'])
    ctx.enter('EXPLORING')
    ctx.wait(lap_file.exists,1000)
    ctx.stop('explorer')
    ctx.zero()
    ctx.enter('SAVING_MAP',loop_evidence='estimated return; SLAM graph closure not independently verified')
    settled = time.monotonic()+6
    ctx.wait(lambda:time.monotonic()>=settled,8)
    prefix = ctx.folder/'new_track'
    ctx.command(['ros2','run','nav2_map_server','map_saver_cli',
                 '-t','/mission/slam_map','-f',prefix],timeout=45)
    if not prefix.with_suffix('.yaml').is_file() or not prefix.with_suffix('.pgm').is_file():
        raise RuntimeError('Map saver did not produce YAML and PGM')
    ctx.command(['python3',MISSION/'capture_map_pose.py',lap_file],timeout=25)
    for name in ('lap_watcher','slam','scan_relay','ekf','sensor_emulator'):
        ctx.stop(name)
    ctx.enter('PLANNING')
    lap = json.loads(lap_file.read_text())
    ctx.command(['python3',ROOT/'tools/planning/extract_robust_centerline.py',
                 prefix.with_suffix('.yaml'),'--out',prefix,'--step','.10','--max-speed',args.speed_cap,
                 '--expected-length',lap['distance_odom_m']],timeout=90)
    oriented = ctx.folder/'new_track_mission_forward.csv'
    ctx.command(['python3',MISSION/'orient_mission_centerline.py',
                 ctx.folder/'new_track_centerline.csv',lap_file,oriented],timeout=25)
    ctx.command(['python3',ROOT/'tools/planning/prepare_vehicle_route.py',
                 prefix.with_suffix('.yaml'),oriented,'--out',ctx.folder/'new_track_body_optimized.csv',
                 '--report',ctx.folder/'route_preparation.json'],timeout=180)
    ctx.enter('WAITING_FOR_LOCALIZATION_HANDOFF')
    wait_for_nodes_absent(ctx.command, output=ctx.folder/'mapping_handoff.json', check=ctx.check)
    ctx.enter('LOCALIZING_AND_RACING')
    ctx.start('race',['python3','-u',MISSION/'run_profiled_saved_map.py',
                     '--mission-dir',ctx.folder,'--autonomous','--laps',args.laps,
                     '--speed-cap',args.speed_cap,'--drive-topic','/mission/drive'])
    deadline = time.monotonic()+max(700,args.laps*420)
    offset = 0
    while ctx.jobs['race'].poll() is None:
        ctx.check(ignore=('race',))
        if time.monotonic()>deadline:
            raise RuntimeError('Racing stage timed out')
        offset = ctx.stream_log('race',offset)
        time.sleep(.3)
    # A fast child failure can happen between loop iterations. Always drain its
    # final output, so the operator sees the actual error rather than a filename.
    ctx.stream_log('race',offset)
    if ctx.jobs['race'].returncode:
        tail = (ctx.folder/'race.log').read_text(errors='replace')[-3000:].strip()
        raise RuntimeError('Racing stopped: '+tail+'\nLog: '+str(ctx.folder/'race.log'))
    ctx.check(ignore=('race',))
    results = list(ctx.folder.glob('race_*/race_result.json'))
    if len(results)!=1 or json.loads(results[0].read_text()).get('completed_estimated_laps')!=args.laps:
        raise RuntimeError('Racing exited without the requested completed-lap record')
    race_dir = results[0].parent
    summary = ctx.command(['python3',ROOT/'tools/localization/summarize_sim_localization_audit.py',
                          race_dir/'localization_truth_audit.csv',
                          '--controller-csv',race_dir/'estimated_drive.csv'],timeout=15)
    (ctx.folder/'autonomy_summary.txt').write_text(summary.stdout)
    ctx.enter('COMPLETED_ESTIMATED_LAPS',laps=args.laps,
              independent_truth_acceptance='summary saved; absolute registration and clearance remain unverified')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--autonomous',action='store_true')
    parser.add_argument('--start-simulator',action='store_true')
    parser.add_argument('--laps',type=int,default=3)
    parser.add_argument('--speed-cap',type=float,default=.40)
    parser.add_argument('--explore-speed',type=float,default=.18)
    args = parser.parse_args()
    if (args.laps<1 or not math.isfinite(args.speed_cap) or not .05<args.speed_cap<=.8 or
            not math.isfinite(args.explore_speed) or not .05<=args.explore_speed<=.35):
        parser.error('Invalid laps or simulation speed bounds')
    ctx = Mission(ROOT/'results/missions'/('autonomous_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')))
    try:
        run_mission(ctx,args)
    except BaseException as exc:
        ctx.enter('STOPPED',reason=str(exc),failed_stage=ctx.stage)
        raise
    finally:
        try:
            ctx.close()
        finally:
            print('Mission outputs: '+str(ctx.folder),flush=True)
            print('Uploadable mission bundle: '+str(ctx.package()),flush=True)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Operator stopped simulation mission.',flush=True)
        raise SystemExit(130)
    except Exception as exc:
        print('MISSION STOPPED: '+str(exc),file=sys.stderr,flush=True)
        raise SystemExit(1)
