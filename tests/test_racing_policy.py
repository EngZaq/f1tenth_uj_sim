#!/usr/bin/env python3
"""Offline regression tests for the full profiled route. No ROS required."""
import math
import pathlib
import sys
import unittest
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/navigation'))
from racing_policy import ProfiledRoute, ramp_velocity


class ProfileTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Route fixture shipped in repository, distinct from generated candidate.
        cls.path = ROOT / 'results/missions/20261028_112407/new_track_mission_forward_feasible.csv'
        cls.route = ProfiledRoute(cls.path)

    def test_closed_waypoint_lookup(self):
        r = self.route
        self.assertEqual(r.n, 377)
        self.assertGreater(r.length, 35.)
        x, y = r.xy[-1]
        self.assertEqual(r.nearest(x, y, r.n-2), r.n-1)

    def test_aim_finite_near_corners_and_seam(self):
        r = self.route
        for i in [0, 1, 42, 44, 49, 55, 190, 375, 376]:
            x, y = r.xy[i]
            yaw = r.data[i, 3]
            steer = r.aim(x, y, yaw, i, .55)
            self.assertTrue(math.isfinite(steer))
            self.assertLessEqual(abs(steer), 0.4189 + 1e-7)
            self.assertGreater(r.speed_at(i), 0.)

    def test_odom_speed_ramp(self):
        self.assertAlmostEqual(ramp_velocity(0., .8, .05), .025)
        self.assertAlmostEqual(ramp_velocity(.8, 0., .05), .76)


if __name__ == '__main__':
    unittest.main()
