"""Check nonzero seed covariance, one-shot acknowledgement and observer routing."""
import contextlib
import importlib.util
import io
import math
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT/path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


INIT = load('amcl_initialization', 'tools/localization/initialize_amcl_pose.py')
NS = types.SimpleNamespace


def message(t=0, frame='slam_map'):
    return NS(header=NS(frame_id=frame, stamp=NS(sec=t, nanosec=0)),
              pose=NS(pose=NS(position=NS(x=1., y=2.)), covariance=[.01]*36))


class SeedTest(unittest.TestCase):
    def test_covariance_is_variance_and_yaw_uses_radians(self):
        result = INIT.pose_payload({'x': 1, 'y': 2, 'yaw': math.pi}, .10, 5)
        cov = result['pose']['covariance']
        self.assertEqual(len(cov), 36)
        self.assertAlmostEqual(cov[0], .01)
        self.assertAlmostEqual(cov[7], .01)
        self.assertAlmostEqual(cov[35], math.radians(5)**2)
        self.assertEqual(cov[1], 0.)
        self.assertAlmostEqual(result['pose']['pose']['orientation']['z'], 1.)
        self.assertEqual(result['header']['frame_id'], 'slam_map')

    def test_invalid_priors_and_seed_are_rejected(self):
        seed = {'x': 1., 'y': 2., 'yaw': 0., 'route_distance_m': .03}
        for invalid in (0., -1., float('nan'), float('inf'), 1e-300, 1e300):
            with self.assertRaises(ValueError):
                INIT.pose_payload(seed, invalid, 5.)
            with self.assertRaises(ValueError):
                INIT.pose_payload(seed, .1, invalid)
        with self.assertRaises(ValueError):
            INIT.pose_payload(dict(seed, x=float('nan')))

    def test_publish_once_and_ignore_old_wrong_frame_or_invalid_replies(self):
        sent, replies = [], []
        pub = NS(get_subscription_count=lambda: 1, publish=sent.append)
        node = NS(get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: NS(sec=100, nanosec=0))))
        ticks = iter([i*.1 for i in range(100)])
        bad = message(101)
        bad.pose.covariance[0] = float('nan')
        pending = iter([message(99), message(101, 'wrong_map'), bad, message(102)])
        def spin(*args, **kwargs):
            replies.append(next(pending))
        reply = INIT.publish_and_confirm(node, pub, message(), replies, 2.,
                                         spin, lambda: next(ticks))
        self.assertEqual(len(sent), 1)
        self.assertEqual(INIT.stamp_s(sent[0]), 100)
        self.assertEqual(INIT.stamp_s(reply), 102)

    def test_missing_subscriber_does_not_publish(self):
        sent = []
        pub = NS(get_subscription_count=lambda: 0, publish=sent.append)
        ticks = iter([i*.1 for i in range(100)])
        with self.assertRaisesRegex(RuntimeError, 'No AMCL /fusion/initialpose subscriber'):
            INIT.publish_and_confirm(None, pub, message(), [], .2,
                                     lambda *a, **k: None, lambda: next(ticks))
        self.assertFalse(sent)

    def test_missing_acknowledgement_times_out_without_republishing(self):
        sent = []
        pub = NS(get_subscription_count=lambda: 1, publish=sent.append)
        node = NS(get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: NS(sec=100, nanosec=0))))
        ticks = iter([i*.1 for i in range(100)])
        with self.assertRaisesRegex(RuntimeError, 'unconfirmed'):
            INIT.publish_and_confirm(node, pub, message(), [], .2,
                                     lambda *a, **k: None, lambda: next(ticks))
        self.assertEqual(len(sent), 1)

    def test_missing_first_reply_requests_update_without_reseeding(self):
        sent,replies,updates=[],[],[]
        pub=NS(get_subscription_count=lambda:1,publish=sent.append)
        node=NS(get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:NS(sec=100,nanosec=0))))
        ticks=iter([i*.1 for i in range(100)])
        def request():
            updates.append(True)
            replies.append(message(102))
        reply=INIT.publish_and_confirm(node,pub,message(),replies,2.,
            lambda *a,**k:None,lambda:next(ticks),request_update=request)
        self.assertEqual(len(sent),1)
        self.assertEqual(len(updates),1)
        self.assertEqual(INIT.stamp_s(reply),102)


class ObserverRoutingTest(unittest.TestCase):
    def test_seed_topic_is_isolated_from_gym_reset(self):
        # Build the actual launch description with lightweight launch objects.
        # Verify its remapping matches the initializer and cannot reach the
        # Gym bridge's global /initialpose reset subscription.
        launch = types.ModuleType('launch')
        launch.LaunchDescription = lambda nodes: nodes
        actions = types.ModuleType('launch.actions')
        actions.DeclareLaunchArgument = lambda *a, **k: NS(kind='argument')
        substitutions = types.ModuleType('launch.substitutions')
        substitutions.LaunchConfiguration = lambda name: name
        ros = types.ModuleType('launch_ros')
        ros_actions = types.ModuleType('launch_ros.actions')
        ros_actions.Node = lambda **kwargs: NS(**kwargs)
        with patch.dict(sys.modules, {'launch': launch, 'launch.actions': actions,
                'launch.substitutions': substitutions, 'launch_ros': ros,
                'launch_ros.actions': ros_actions}):
            module = load('mission_amcl_launch_test', 'tools/mission/mission_amcl.launch.py')
            nodes = module.generate_launch_description()
        amcl = next(node for node in nodes if getattr(node, 'executable', '') == 'amcl')
        self.assertIn(('initialpose', INIT.INITIAL_POSE_TOPIC), amcl.remappings)
        self.assertNotEqual(INIT.INITIAL_POSE_TOPIC, '/initialpose')

    def exercise(self, fail_initialization=False, autonomous=False, fail_readiness=False):
        module = load('saved_map_launcher_test', 'tools/mission/run_profiled_saved_map.py')
        starts, calls = [], []
        seed = {'x': 1., 'y': 2., 'yaw': 0., 'route_distance_m': .03}
        def call(args, **kwargs):
            args = [str(x) for x in args]
            calls.append(args)
            if 'map_scan_seed.py' in ' '.join(args):
                import json
                Path(args[args.index('--output')+1]).write_text(json.dumps(seed))
            if 'initialize_amcl_pose.py' in ' '.join(args) and fail_initialization:
                raise RuntimeError('injected initialization failure')
            if 'wait_amcl_ready.py' in ' '.join(args) and fail_readiness:
                raise RuntimeError('injected readiness failure')
            out = ('Publisher count: 1' if '/ego_racecar/odom' in args
                   else 'Publisher count: 0' if '/drive' in args else '')
            return NS(returncode=0, stdout=out, stderr='')
        def start(name, args):
            starts.append(name)
            module.PROCS[name] = (NS(poll=lambda: 0, returncode=0)
                                  if name == 'stationary_probe' else NS(poll=lambda: None))
            if name=='autonomous_drive':
                (module.RUN/'autonomous_drive.log').write_text('ESTIMATED LAP 3 COMPLETE\n')
        ticks = iter([i*.1 for i in range(100)])
        with tempfile.TemporaryDirectory() as directory, contextlib.ExitStack() as stack:
            stack.enter_context(patch.object(module, 'RUN', Path(directory)))
            stack.enter_context(patch.object(module, 'PATH', ROOT/'results/missions/20261028_112407/new_track_mission_forward_feasible.csv'))
            stack.enter_context(patch.object(module, 'call', call))
            stack.enter_context(patch.object(module, 'start', start))
            stack.enter_context(patch.object(module, 'wait_active', lambda: None))
            stack.enter_context(patch.object(module, 'read_amcl_pose', lambda:
                {'position': {'x': 1., 'y': 2.},
                 'orientation': {'x': 0., 'y': 0., 'z': 0., 'w': 1.}}))
            stack.enter_context(patch.object(module.time, 'monotonic', lambda: next(ticks)))
            stack.enter_context(patch.object(module.time, 'sleep', lambda _: None))
            stack.enter_context(patch('builtins.input',
                (lambda _: self.fail('automatic mode prompted')) if autonomous else (lambda _: '')))
            argv=['launcher','--localization-only','--observe-seconds','.2']
            if autonomous:
                import json,shutil
                folder=Path(directory)/'fresh_map';folder.mkdir()
                for source,target in [('new_track.yaml','new_track.yaml'),('new_track.pgm','new_track.pgm'),
                        ('lap_pose.json','lap_pose.json'),
                        ('new_track_mission_forward_feasible.csv','new_track_body_optimized.csv')]:
                    shutil.copy(ROOT/'results/missions/20261028_112407'/source,folder/target)
                (folder/'laser_geometry.json').write_text(json.dumps(module.LASER))
                # A fresh mission must not depend on the historical map folder.
                stack.enter_context(patch.object(module,'RECOVERED',Path(directory)/'absent_old_map'))
                argv=['launcher','--autonomous','--mission-dir',str(folder),'--laps','3']
            stack.enter_context(patch('sys.argv',argv))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            if fail_initialization or fail_readiness:
                with self.assertRaisesRegex(RuntimeError, 'injected .* failure'):
                    module.main()
            else:
                module.main()
                import yaml
                params = yaml.safe_load((module.RUN/'recovered_map_amcl.yaml').read_text())
                self.assertIs(params['amcl']['ros__parameters']['set_initial_pose'], False)
                if autonomous:
                    self.assertEqual(json.loads((module.RUN/'race_result.json').read_text())['completed_estimated_laps'],3)
        if autonomous and not fail_initialization and not fail_readiness:
            self.assertIn('autonomous_drive',starts)
            self.assertTrue(any('{data: true}' in args for args in calls))
        else:
            self.assertNotIn('autonomous_drive', starts)
            self.assertFalse(any('{data: true}' in args for args in calls))
        return starts

    def test_observer_never_starts_controller_or_arms(self):
        self.assertIn('localization_audit', self.exercise())

    def test_initialization_failure_aborts_before_controller(self):
        self.exercise(fail_initialization=True)

    def test_automatic_racing_waits_then_arms_without_prompts(self):
        self.exercise(autonomous=True)

    def test_readiness_failure_cannot_start_or_arm_controller(self):
        self.exercise(autonomous=True,fail_readiness=True)


if __name__ == '__main__':
    unittest.main()
