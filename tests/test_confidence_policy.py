#!/usr/bin/env python3
"""Regression tests for transient AMCL covariance handling and hard stops."""
import math
import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools/navigation'))
from confidence_policy import ConfidenceGate


class ConfidenceTest(unittest.TestCase):
    def test_recorded_pause_recovers_inside_same_driving_gate(self):
        # Replay measured covariance and distinct pose stamps from the uploaded
        # run. Zero EKF speed is an explicit fixture assumption, not a new sensor.
        fixture=json.loads((Path(__file__).parent/'fixtures/confidence_pause_20261029.json').read_text())
        g=ConfidenceGate()
        def check(row,speed):
            return g.check(row['elapsed_s'],row['pose_stamp_s'],
                           row['position_std_m'],row['yaw_std_rad'],speed)[0]
        self.assertEqual(check(fixture['trigger'],.4),'PAUSE')
        states=[check(row,0.) for row in fixture['stationary_observations']]
        self.assertEqual(states[:3],['PAUSE']*3)
        self.assertEqual(states[3],'GO')
        self.assertNotIn('FAULT',states)

    def test_moving_or_above_entry_limit_samples_reset_recovery(self):
        g=ConfidenceGate();g.check(0.,100.,.081,.05,.4)
        for j in range(1,4):
            g.check(j*.1,100.+j*.1,.07,.05,0.)
        self.assertEqual(g.good_count,3)
        self.assertEqual(g.check(.4,100.4,.07,.05,.2)[0],'PAUSE')
        self.assertEqual(g.good_count,0)
        g.check(.5,100.5,.07,.05,0.)
        g.check(.6,100.6,.081,.05,0.)
        self.assertEqual(g.good_count,0)

    def test_out_of_order_estimate_cannot_count_as_recovery(self):
        g=ConfidenceGate();g.check(0.,100.,.081,.05,.4)
        for j in range(1,4):g.check(j*.1,100.+j*.1,.07,.05,0.)
        self.assertEqual(g.good_count,3)
        self.assertEqual(g.check(.4,100.1,.07,.05,0.)[0],'PAUSE')
        self.assertEqual(g.good_count,0)

    def test_normal_confidence_drives(self):
        g = ConfidenceGate()
        self.assertEqual(g.check(10.0, 100.0, .04, math.radians(3), .4)[0], 'GO')

    def test_uncertainty_pauses_then_recovers_only_when_stationary_and_new(self):
        g = ConfidenceGate()
        self.assertEqual(g.check(10., 100., .085, math.radians(3), .4)[0], 'PAUSE')
        for j in range(3):
            self.assertEqual(g.check(10.2+j*.2, 101.+j,
                                     .04, math.radians(4), .0)[0], 'PAUSE')
        # Repeated ticks with the same pose do not count as new evidence.
        self.assertEqual(g.check(10.7, 103., .04, math.radians(4), .0)[0], 'PAUSE')
        self.assertEqual(g.check(10.8, 104., .04, math.radians(4), .0)[0], 'GO')
        self.assertIsNone(g.paused_since)

    def test_uncertainty_times_out(self):
        g = ConfidenceGate()
        self.assertEqual(g.check(10., 100., .09, .03, .4)[0], 'PAUSE')
        state, reason = g.check(14.1, 102., .09, .03, .0)
        self.assertEqual(state, 'FAULT')
        self.assertIn('did not relocalize', reason)

    def test_severe_uncertainty_never_moves(self):
        g = ConfidenceGate()
        self.assertEqual(g.check(0., 1., .20, .1, .0)[0], 'FAULT')
        self.assertEqual(g.check(0., 1., .03, math.radians(21), .0)[0], 'FAULT')

    def test_nonfinite_rejected(self):
        g = ConfidenceGate()
        self.assertEqual(g.check(0., 1., float('nan'), .01, 0.)[0], 'FAULT')


if __name__ == '__main__':
    unittest.main()
