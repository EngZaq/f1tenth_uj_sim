"""Commanded motion and scan compensation, independent of global pose."""
import math
import json
import sys
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/navigation'))
from trajectory_safety import command_swept_collision,stopping_horizon,select_clear_speed


class CommandCollisionTest(unittest.TestCase):
    def wall(self,distance):
        angles=np.linspace(-.4,.4,81)
        return (distance-.1)/np.cos(angles),angles[0],angles[1]-angles[0]

    def check(self,scan,steering=0.,speed=.4,**kwargs):
        ranges,a,step=scan
        kwargs.setdefault('scan_to_body',(.1,0.,0.))
        return command_swept_collision(ranges,a,step,.05,25.,steering,speed,**kwargs)

    def test_forward_obstacle_inside_braking_travel_is_blocked(self):
        for speed in (.1,.2,.4,.8):
            for reaction in (.35,.7):
                wall=.46145+stopping_horizon(speed,reaction)-.04
                self.assertTrue(self.check(self.wall(wall),speed=speed,reaction=reaction)[0])

    def test_clear_straight_corridor_is_clear(self):
        a=np.linspace(.4,1.4,80)
        scan=(.4/np.sin(a),a[0],a[1]-a[0])
        self.assertFalse(self.check(scan)[0])

    def test_requested_turn_toward_wall_blocks_but_turn_away_clears(self):
        a=np.linspace(.4,1.4,80)
        scan=(.4/np.sin(a),a[0],a[1]-a[0])
        self.assertTrue(self.check(scan,steering=.4189)[0])
        self.assertFalse(self.check(scan,steering=-.4189)[0])

    def test_steering_transition_enlarges_tube(self):
        scan=self.wall(2.)
        steady=self.check(scan,steering=.2,initial_steering=.2)[1]
        transition=self.check(scan,steering=.2,initial_steering=-.2)[1]
        self.assertLess(transition,steady)

    def test_local_noise_enlarges_tube(self):
        scan=self.wall(2.)
        exact=self.check(scan,speed_std=0.,yaw_rate_std=0.)[1]
        noisy=self.check(scan)[1]
        self.assertLess(noisy,exact)

    def test_scan_age_motion_compensation_moves_wall_toward_car(self):
        scan=self.wall(1.1)
        self.assertFalse(self.check(scan)[0])
        # Car moved 0.30 m after scan acquisition: old laser origin is now
        # 0.20 m behind current body origin, not +0.10 m ahead of it.
        self.assertTrue(self.check(scan,scan_to_body=(-.2,0.,0.))[0])

    def test_scan_transform_rotation_is_applied_once(self):
        ranges,a,step=self.wall(1.1)
        expected=self.check((ranges,a,step))
        actual=self.check((ranges,a-.7,step),scan_to_body=(.1,0.,.7))
        self.assertEqual(expected[0],actual[0])
        self.assertAlmostEqual(expected[1],actual[1],places=10)

    def test_sparse_scan_fails_closed(self):
        scan=self.wall(1.1);scan[0][:50]=float('nan')
        self.assertTrue(self.check(scan)[0])

    def test_invalid_motion_inputs_rejected(self):
        for kwargs in ({'initial_steering':float('nan')},{'scan_to_body':(0.,0.,float('nan'))},
                       {'speed_std':-.01},{'steering':.5},{'speed':-.1}):
            with self.assertRaises(ValueError):self.check(self.wall(2.),**kwargs)


class LocalSpeedTest(unittest.TestCase):
    def test_clear_road_preserves_route_speed(self):
        result=select_clear_speed(.4,lambda v:(False,.4,{'chosen':v}))
        self.assertEqual(result,(.4,.4,{'chosen':.4}))

    def test_shrinking_clearance_slows_before_hard_stop(self):
        # At 0.4 the tube is still hard-clear, but lacks the desired reserve.
        result=select_clear_speed(.4,lambda v:(False,.30-.5*v,v))
        self.assertLess(result[0],.4)
        self.assertGreaterEqual(result[1],.15)

    def test_forward_wall_allows_lower_speed_only_when_measured_tube_clear(self):
        angles=np.linspace(-.4,.4,81)
        ranges=(.95-.1)/np.cos(angles)
        def evaluate(proposal,measured):
            blocked,gap,hits=command_swept_collision(
                ranges,angles[0],angles[1]-angles[0],.05,25.,0.,max(measured,proposal),
                scan_to_body=(.1,0.,0.))
            return blocked,gap,hits
        self.assertLess(select_clear_speed(.8,lambda v:evaluate(v,.15))[0],.8)
        with self.assertRaisesRegex(ValueError,'no safe local speed'):
            select_clear_speed(.8,lambda v:evaluate(v,.8))

    def test_hard_clear_creep_if_no_proposal_has_reserve(self):
        result=select_clear_speed(.4,lambda v:(False,.09,v))
        self.assertEqual(result[0],.05)

    def test_obstructed_or_corrupt_tube_cannot_creep(self):
        with self.assertRaises(ValueError):
            select_clear_speed(.4,lambda v:(True,.01,v))
        with self.assertRaises(ValueError):
            select_clear_speed(.4,lambda v:(False,float('nan'),v))

    def test_geometry_results_can_be_saved_as_stop_evidence(self):
        scan=CommandCollisionTest().wall(.8)
        result=command_swept_collision(scan[0],scan[1],scan[2],.05,25.,0.,.4)
        json.dumps(dict(blocked=result[0],gap=result[1],hits=result[2]))


if __name__=='__main__':unittest.main()
