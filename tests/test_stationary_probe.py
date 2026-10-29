"""Offline checks for diagnostic sampling and nonblocking AMCL refresh."""
import importlib.util
import json
import math
import types
import unittest
from pathlib import Path

PATH = Path(__file__).resolve().parents[1]/'tools/localization/stationary_localization_probe.py'
SPEC = importlib.util.spec_from_file_location('stationary_probe', PATH)
PROBE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROBE)
NS = types.SimpleNamespace


class Future:
    def __init__(self):
        self.finished = False
        self.error = None
    def done(self):
        return self.finished
    def result(self):
        if self.error:
            raise self.error
        return object()


class RefreshTest(unittest.TestCase):
    def test_one_request_pending_and_cadence(self):
        policy = PROBE.RefreshSchedule(.3, 2.)
        futures = []
        def send():
            futures.append(Future())
            return futures[-1]
        policy.tick(0., True, send)
        policy.tick(.1, True, send)
        self.assertEqual(len(futures), 1)
        futures[-1].finished = True
        policy.tick(.2, True, send)
        self.assertEqual(policy.completed, 1)
        self.assertEqual(len(futures), 1)
        policy.tick(.31, True, send)
        self.assertEqual(len(futures), 2)
        self.assertEqual(policy.sent, 2)

    def test_unavailable_service_does_not_enqueue(self):
        policy = PROBE.RefreshSchedule()
        policy.tick(0, False, lambda: self.fail('must not send'))
        self.assertEqual(policy.sent, 0)

    def test_timeout_does_not_enqueue_second_request(self):
        policy = PROBE.RefreshSchedule(.3, .5)
        future = Future()
        policy.tick(0, True, lambda: future)
        with self.assertRaisesRegex(RuntimeError, 'timed out'):
            policy.tick(.6, True, lambda: self.fail('must not send'))

    def test_service_errors_propagate(self):
        policy = PROBE.RefreshSchedule()
        future = Future()
        policy.tick(0, True, lambda: future)
        future.finished, future.error = True, RuntimeError('failed service')
        with self.assertRaisesRegex(RuntimeError, 'failed service'):
            policy.tick(.1, True, lambda: self.fail('must not send'))

    def test_invalid_cadence_rejected(self):
        for value in [0, -1, float('inf'), float('nan')]:
            with self.assertRaises(ValueError):
                PROBE.RefreshSchedule(value)


class DataTest(unittest.TestCase):
    def test_noise_statistics_and_missing_samples(self):
        values = PROBE.Moments()
        for x in [1., 2., 3., float('nan')]:
            values.add(x)
        result = values.result()
        self.assertEqual(result['count'], 3)
        self.assertEqual(result['mean'], 2.)
        self.assertEqual(result['sample_std'], 1.)

    def test_scan_nulls_preserve_geometry_and_throttle_only_snapshots(self):
        data = PROBE.ProbeData()
        message = NS(header=NS(frame_id='original_laser', stamp=NS(sec=1, nanosec=0)),
                     angle_min=-1., angle_max=1., angle_increment=1.,
                     range_min=.1, range_max=10., time_increment=.001, scan_time=.01,
                     ranges=[2., float('inf'), float('nan')])
        data.scan('raw_scan', message)
        message.header.stamp.nanosec = 100000000
        message.ranges = [3., 4., 5.]
        data.scan('raw_scan', message)
        self.assertEqual(data.streams['raw_scan']['count'], 2)
        self.assertEqual(data.scans['raw_scan']['last']['ranges'], [2., None, None])
        message.header.stamp.sec = 2
        data.scan('raw_scan', message)
        self.assertEqual(data.scans['raw_scan']['first']['ranges'], [2., None, None])
        self.assertEqual(data.scans['raw_scan']['last']['ranges'], [3., 4., 5.])
        self.assertEqual(data.frames['raw_scan'], 'original_laser')
        json.dumps(data.result(), allow_nan=False)

    def test_nonfinite_metadata_does_not_destroy_diagnostics(self):
        result = PROBE.json_safe({'a': [float('nan'), float('inf'), 2.], 'b': None})
        self.assertEqual(result, {'a': [None, None, 2.], 'b': None})
        json.dumps(result, allow_nan=False)


if __name__ == '__main__':
    unittest.main()
