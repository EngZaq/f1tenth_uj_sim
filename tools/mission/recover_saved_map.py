#!/usr/bin/env python3
"""Offline recovery: derive validated route from an already saved mission map.
No ROS, simulator, SLAM, or VESC calls. Never publishes /drive.
"""
import json
import argparse
import subprocess
import sys
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('mission_dir',type=Path)
    args=p.parse_args()
    root=Path.home()/'f1tenth_sim'
    folder=args.mission_dir.expanduser().resolve()
    mapfile=folder/'new_track.yaml'
    posefile=folder/'lap_pose.json'
    if not mapfile.exists() or not posefile.exists():
        raise SystemExit('Missing new_track.yaml or lap_pose.json in mission folder')
    pose=json.loads(posefile.read_text())
    out=folder/'new_track'
    command=[sys.executable,str(root/'tools/planning/extract_robust_centerline.py'),
             str(mapfile),'--out',str(out),'--step','0.10',
             '--max-speed','0.35',
             '--expected-length',str(pose['distance_odom_m'])]
    print('Recovering previously mapped track; NO exploration or driving...',flush=True)
    subprocess.run(command,check=True)
    command=[sys.executable,str(root/'tools/mission/orient_mission_centerline.py'),
             str(folder/'new_track_centerline.csv'),str(posefile),
             str(folder/'new_track_mission_forward.csv')]
    subprocess.run(command,check=True)
    print('\nOFFLINE MAP AND PATH RECOVERED SUCCESSFULLY')
    print(f'Map:       {mapfile}')
    print(f'Centerline:{folder/"new_track_mission_forward.csv"}')
    print(f'Preview:   {folder/"new_track_preview.png"}')
    print('The car is NOT localized by this script. Do not directly enable driving using'
          ' the previous lap finish pose after the simulator has been restarted.')

if __name__=='__main__':
    main()
