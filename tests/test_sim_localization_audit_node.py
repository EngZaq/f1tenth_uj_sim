#!/usr/bin/env python3
"""Regression: construct passive audit node with ROS-like read-only Node.handle.

No ROS installation required. This test deliberately reproduces the
AttributeError that previously aborted the saved-map mission.
"""
import csv
import importlib.util
import sys
import tempfile
import types
import unittest
from pathlib import Path


def namespace(**kwargs):
    return types.SimpleNamespace(**kwargs)


class Logger:
    def warn(self, *args): pass
    def info(self, *args): pass


class StubNode:
    def __init__(self, *args, **kwargs):
        self._ros_node_handle = object()

    @property
    def handle(self):
        return self._ros_node_handle

    def create_subscription(self, *args, **kwargs):
        return object()

    def get_logger(self):
        return Logger()


def import_audit_with_ros_stubs():
    def module(name):
        obj = types.ModuleType(name)
        sys.modules[name] = obj
        return obj

    rclpy = module('rclpy')
    rclpy.node = module('rclpy.node')
    rclpy.node.Node = StubNode
    rclpy.executors = module('rclpy.executors')
    rclpy.executors.ExternalShutdownException = type('ExternalShutdownException', (Exception,), {})
    geometry = module('geometry_msgs')
    geometry.msg = module('geometry_msgs.msg')
    geometry.msg.PoseWithCovarianceStamped = type('PoseWithCovarianceStamped', (), {})
    nav = module('nav_msgs')
    nav.msg = module('nav_msgs.msg')
    nav.msg.Odometry = type('Odometry', (), {})
    path = Path(__file__).resolve().parents[1] / 'tools/localization/sim_localization_audit.py'
    spec = importlib.util.spec_from_file_location('sim_localization_audit_under_test', path)
    instance = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(instance)
    return instance


def fake_pose(x, y):
    return namespace(position=namespace(x=x, y=y),
                     orientation=namespace(x=0., y=0., z=0., w=1.))


def fake_odom(t, x, y):
    return namespace(
        header=namespace(stamp=namespace(sec=t, nanosec=0)),
        pose=namespace(pose=fake_pose(x, y)))


def fake_amcl(t, x, y):
    covariance = [0.] * 36
    covariance[0] = covariance[7] = .0025
    covariance[35] = .001
    return namespace(
        header=namespace(stamp=namespace(sec=t, nanosec=0), frame_id='slam_map'),
        pose=namespace(pose=fake_pose(x, y), covariance=covariance))


class AuditNodeConstructionTest(unittest.TestCase):
    def test_construct_log_and_close(self):
        audit_module = import_audit_with_ros_stubs()
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / 'relative_truth.csv'
            node = audit_module.Audit(output)
            # On instantiation, the file must have a flushed header even
            # before any source publishes odometry.
            self.assertIn('amcl_relative_pos_error_m', output.read_text())
            self.assertIsNotNone(node.handle)
            self.assertNotIsInstance(node.handle, type(node.output_fp))
            node.on_truth(fake_odom(100, 1., 1.))
            node.on_ekf(fake_odom(100, 0., 0.))
            node.on_amcl(fake_amcl(100, 6., 8.))
            node.on_truth(fake_odom(101, 1.5, 1.))
            node.on_ekf(fake_odom(101, 0.5, 0.))
            node.on_amcl(fake_amcl(101, 6.6, 8.))
            node.close()
            with output.open(newline='') as f:
                lines = list(csv.DictReader(f))
            self.assertEqual(len(lines), 2)
            self.assertAlmostEqual(float(lines[0]['amcl_relative_pos_error_m']), 0., places=5)
            self.assertAlmostEqual(float(lines[1]['amcl_relative_pos_error_m']), .1, places=4)
            self.assertTrue(node.output_fp.closed)


if __name__ == '__main__':
    unittest.main()
