#!/usr/bin/env python3
"""Regression tests for transient AMCL covariance handling and hard stops."""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools/navigation'))
from confidence_policy import ConfidenceGate


class ConfidenceTest(unittest.TestCase):
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
