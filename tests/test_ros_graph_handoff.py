"""Regression for clean SLAM exit followed by stale ROS graph discovery."""
import contextlib
import io
import json
import subprocess
import sys
import tempfile
import types
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from mission.ros_graph import wait_for_nodes_absent


class GraphHandoffTest(unittest.TestCase):
    def exercise(self, snapshots, *, timeout=3., failure=None, check=None):
        calls=[]
        now=[0.]
        def command(args, **kwargs):
            calls.append(args)
            now[0] += .25
            if failure:
                raise failure
            # A cached daemon would report SLAM forever; only fresh queries
            # observe the departure sequence below.
            output = '/slam_toolbox' if '--no-daemon' not in args else snapshots[min(len(calls)-1,len(snapshots)-1)]
            return types.SimpleNamespace(returncode=0,stdout=output,stderr='')
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            output=Path(tmp)/'handoff.json'
            try:
                record=wait_for_nodes_absent(command, timeout=timeout, output=output,
                    clock=lambda:now[0], sleep=lambda dt:None,check=check)
            except BaseException:
                self.last_record=json.loads(output.read_text())
                raise
            self.assertEqual(record,json.loads(output.read_text()))
        return calls,record

    def test_clean_process_exit_with_delayed_graph_departure_can_handoff(self):
        calls,record=self.exercise(['/slam_toolbox','/slam_toolbox','/bridge','/bridge'])
        self.assertEqual(len(calls),4)
        self.assertTrue(record['ready'])
        self.assertEqual(record['observations'][0]['remaining_nodes'],['/slam_toolbox'])
        self.assertTrue(all('--no-daemon' in args and '--spin-time' in args for args in calls))

    def test_transient_empty_graph_is_not_enough(self):
        calls,record=self.exercise(['','/slam_toolbox','',''])
        self.assertEqual(len(calls),4)
        self.assertTrue(record['ready'])

    def test_real_remaining_slam_cannot_be_ignored(self):
        with self.assertRaisesRegex(RuntimeError,'still visible: /slam_toolbox'):
            self.exercise(['/slam_toolbox'],timeout=.6)
        self.assertFalse(self.last_record['ready'])

    def test_discovery_failure_does_not_mean_empty_graph(self):
        def failed(*args,**kwargs):
            return types.SimpleNamespace(returncode=1,stdout='',stderr='DDS failed')
        with self.assertRaisesRegex(RuntimeError,'DDS failed'):
            wait_for_nodes_absent(failed)

    def test_hung_discovery_preserves_failure_record(self):
        with self.assertRaises(subprocess.TimeoutExpired):
            self.exercise([''],failure=subprocess.TimeoutExpired('ros2',1.))
        self.assertFalse(self.last_record['ready'])
        self.assertIn('timed out',self.last_record['error'])

    def test_watchdog_fault_interrupts_wait(self):
        def failed():
            raise RuntimeError('watchdog collision')
        with self.assertRaisesRegex(RuntimeError,'watchdog collision'):
            self.exercise(['/slam_toolbox'],check=failed)

    def test_only_the_watchdog_can_remain_during_stage_handoff(self):
        _,record=self.exercise(['/simulation_drive_watchdog\n/bridge'])
        self.assertTrue(record['ready'])


if __name__=='__main__':
    unittest.main()
