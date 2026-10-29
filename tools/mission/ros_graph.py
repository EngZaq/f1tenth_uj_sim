"""Fresh, bounded ROS graph checks for simulator stage ownership.

The CLI daemon can still list a node after its process exited. Humble's
--no-daemon queries a new DDS participant instead. Discovery still needs time,
so require two clean observations and never treat a query error as absence.
This module does not import ROS or publish commands.
"""
import json
import subprocess
import time
from pathlib import Path


LOCALIZATION_CONFLICTS = (
    '/amcl', '/slam_toolbox', '/fusion_map_server', '/ekf_filter_node',
    '/sim_vesc_imu', '/fusion_scan_relay', '/profiled_amcl_drive',
    '/mission_first_lap', '/uj_exploration_controller',
)


def node_list_args():
    return ['ros2', 'node', 'list', '--no-daemon', '--spin-time', '1.0']


def topic_info_args(topic):
    return ['ros2', 'topic', 'info', topic, '--no-daemon', '--spin-time', '1.0']


def wait_for_nodes_absent(command, names=LOCALIZATION_CONFLICTS, *, timeout=30.,
                          output=None, check=None, clock=None, sleep=None):
    """Wait for departed nodes, without ignoring a genuinely running stack."""
    clock = clock or time.monotonic
    sleep = sleep or time.sleep
    start = clock()
    deadline = start + timeout
    clean = 0
    remaining = sorted(names)
    record = {'query': node_list_args(), 'expected_absent_nodes': sorted(names),
              'timeout_s': timeout, 'ready': False, 'observations': []}

    def save():
        if output is not None:
            Path(output).write_text(json.dumps(record, indent=2)+'\n')

    try:
        while True:
            if check:
                check()
            budget = deadline-clock()
            if budget <= 0:
                raise RuntimeError('ROS graph handoff timed out; still visible: '+
                                   ', '.join(remaining))
            try:
                result = command(node_list_args(), timeout=min(6., budget), check=False)
            except subprocess.TimeoutExpired as exc:
                if record['observations'] and remaining:
                    raise RuntimeError('ROS graph handoff timed out; still visible: '+
                                       ', '.join(remaining)) from exc
                raise
            if result.returncode:
                raise RuntimeError('Fresh ROS node discovery failed: '+result.stderr.strip())
            nodes = set(result.stdout.splitlines())
            remaining = sorted(nodes.intersection(names))
            record['observations'].append({'elapsed_s': round(clock()-start, 3),
                                           'remaining_nodes': remaining})
            clean = 0 if remaining else clean+1
            if clean >= 2:
                if check:
                    check()
                record['ready'] = True
                save()
                print('ROS graph handoff ready: previous stack absent in two fresh checks.', flush=True)
                return record
            if remaining:
                print('Waiting for ROS graph departure: '+', '.join(remaining), flush=True)
            save()
            sleep(min(.2, max(0., deadline-clock())))
    except (Exception, KeyboardInterrupt) as exc:
        record['error'] = ('Fresh ROS node discovery timed out' if isinstance(exc, subprocess.TimeoutExpired)
                           else str(exc))
        save()
        raise
