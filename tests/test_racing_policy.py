#!/usr/bin/env python3
"""Offline regression tests for the full profiled route. No ROS required."""
import math
import pathlib
import sys
import unittest
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/navigation'))
from racing_policy import ProfiledRoute, ramp_velocity, route_progress_delta, tracking_lookahead, normalized_angle


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


    def test_short_lookahead_tracks_whole_recorded_route_with_steering_slew(self):
        # Ideal estimated rear-axle pose: policy/model test, not a localization
        # or collision certificate. Perturb initial lateral pose and heading.
        r=self.route;dt=.05;speed=.4
        for offset,heading_error in ((.05,5.),(-.05,-5.)):
            with self.subTest(offset=offset):
                heading=float(r.data[0,3])
                x,y=r.xy[0]+offset*np.array([-math.sin(heading),math.cos(heading)])
                yaw=heading+math.radians(heading_error)
                steering=0.;index=0;progress=0;worst=0.
                for tick in range(6000):
                    current=r.nearest(x,y,index)
                    progress+=route_progress_delta(index,current,r.n)
                    index=current;worst=max(worst,r.error(x,y,index))
                    desired=r.aim(x,y,yaw,index,tracking_lookahead(speed))
                    steering+=float(np.clip(desired-steering,-3.2*dt,3.2*dt))
                    curvature=math.tan(steering)/r.wheelbase;distance=speed*dt
                    if abs(curvature)>1e-9:
                        x+=(math.sin(yaw+curvature*distance)-math.sin(yaw))/curvature
                        y-=(math.cos(yaw+curvature*distance)-math.cos(yaw))/curvature
                    else:
                        x+=distance*math.cos(yaw);y+=distance*math.sin(yaw)
                    yaw=normalized_angle(yaw+curvature*distance)
                    if progress>=r.n:break
                self.assertGreaterEqual(progress,r.n)
                self.assertLess(worst,.07)


class TrackingSpeedTest(unittest.TestCase):
    def test_tracking_error_slows_before_stop_boundary(self):
        from racing_policy import tracking_speed_cap
        values=[tracking_speed_cap(.4,x) for x in (0.,.02,.03,.04,.05,.06,.08)]
        self.assertEqual(values[:2],[.4,.4])
        self.assertTrue(all(a>=b for a,b in zip(values,values[1:])))
        self.assertAlmostEqual(values[-1],.1)
        self.assertAlmostEqual(tracking_speed_cap(.08,.08),.08)
        for bad in (-.01,float('nan')):
            with self.assertRaises(ValueError):tracking_speed_cap(.4,bad)


if __name__ == '__main__':
    unittest.main()
