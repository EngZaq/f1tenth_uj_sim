#!/usr/bin/env python3
"""Read final SLAM map->base transform after SLAM settles. Never uses truth."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def run(out):
    rclpy.init()
    node=Node('mission_capture_slam_pose')
    buffer=Buffer()
    listener=TransformListener(buffer,node)
    data=None
    try:
        end=time.monotonic()+10
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=0.10)
            try:
                m=buffer.lookup_transform('slam_map','fusion_base',Time()).transform
                data={'finish_x':m.translation.x,'finish_y':m.translation.y,
                      'finish_yaw':yaw(m.rotation),'frame_id':'slam_map',
                      'source':'latest SLAM TF after mapping settled; no oracle'}
                break
            except TransformException:
                continue
        if data is None: raise RuntimeError('Failed to read slam_map->fusion_base TF')
        original=json.loads(Path(out).read_text())
        old=(original['finish_x'],original['finish_y'])
        if math.hypot(old[0]-data['finish_x'],old[1]-data['finish_y'])>2.0:
            raise RuntimeError('SLAM pose jumped >2m after loop; refuse to hand off to AMCL')
        original.update(data)
        Path(out).write_text(json.dumps(original,indent=2))
        print(f'Latest SLAM finish pose: ({data["finish_x"]:.3f},{data["finish_y"]:.3f},'
              f'{math.degrees(data["finish_yaw"]):.1f}deg)')
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('output');run(p.parse_args().output)
