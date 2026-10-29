#!/usr/bin/env python3
"""Live ROS Humble/CycloneDDS regression; no simulator or actuation topics.

Warm CLI discovery with a real node named slam_toolbox, verify the guard rejects
that live node, stop it cleanly, then verify the fresh handoff becomes ready.
Run separately from the dependency-free offline test discovery.
"""
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from mission.ros_graph import node_list_args, topic_info_args, wait_for_nodes_absent


def command(args, **kwargs):
    return subprocess.run(args,capture_output=True,text=True,**kwargs)


def main():
    folder=Path(os.environ.get('ROS_HANDOFF_EVIDENCE','/tmp/ros-handoff-evidence'))
    folder.mkdir(parents=True,exist_ok=True)
    code = '''
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
rclpy.init()
node=Node('slam_toolbox')
pub=node.create_publisher(String,'/handoff_test_probe',10)
try:
    rclpy.spin(node)
except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):
    pass
finally:
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
'''
    with (folder/'node.log').open('w') as log:
        process=subprocess.Popen([sys.executable,'-u','-c',code],stdout=log,stderr=subprocess.STDOUT)
        try:
            deadline=time.monotonic()+20
            while True:
                if process.poll() is not None:
                    raise RuntimeError('Fixture node failed: '+(folder/'node.log').read_text())
                nodes=command(node_list_args(),timeout=6,check=True)
                if '/slam_toolbox' in nodes.stdout.splitlines():
                    break
                if time.monotonic()>deadline:
                    raise RuntimeError('Live fixture discovery timed out')
            # Exercise the real Humble topic-info flags as well as node-list.
            info=command(topic_info_args('/handoff_test_probe'),timeout=6,check=True)
            if 'Publisher count: 1' not in info.stdout:
                raise RuntimeError('Expected the live fixture publisher: '+info.stdout)
            cached=command(['ros2','node','list'],timeout=6,check=True)
            (folder/'cached_before_stop.txt').write_text(cached.stdout)
            try:
                wait_for_nodes_absent(command,timeout=3,output=folder/'while_alive.json')
            except RuntimeError as exc:
                if 'still visible: /slam_toolbox' not in str(exc):
                    raise
            else:
                raise AssertionError('Guard accepted an actually running SLAM node')
            process.send_signal(signal.SIGINT)
            process.wait(timeout=8)
            if process.returncode:
                raise RuntimeError('Fixture did not exit cleanly: '+(folder/'node.log').read_text())
            wait_for_nodes_absent(command,timeout=30,output=folder/'after_clean_exit.json')
            print('PASS live Humble/CycloneDDS: rejected live SLAM, accepted clean departure.')
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
            command(['ros2','daemon','stop'],timeout=6,check=False)


if __name__=='__main__':
    main()
