"""Route-relative confidence and covariance-budgeted collision regressions."""
import math
import json
import sys
import unittest
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools/navigation'))
from pose_uncertainty import planar_covariance, project_uncertainty
from confidence_policy import ConfidenceGate, uncertainty_speed_cap
from trajectory_safety import swept_path_collision, stopping_horizon
from racing_policy import route_progress_delta


class ProjectionTest(unittest.TestCase):
    def test_projection_is_invariant_to_map_rotation(self):
        original=np.diag([.10**2,.02**2,.03**2])
        for angle in (0.,.4,1.2,2.8):
            c,s=math.cos(angle),math.sin(angle)
            rotation=np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])
            metrics=project_uncertainty(rotation@original@rotation.T,angle)
            np.testing.assert_allclose(metrics,(.02,.10,.10,.03),atol=1e-10)

    def test_full_covariance_rejects_invalid_cross_terms(self):
        for matrix in (np.array([[.01,.02,0],[.02,.01,0],[0,0,.01]]),
                       np.array([[.01,0,.02],[0,.01,0],[.02,0,.01]]),
                       np.array([[.01,.001,0],[0,.01,0],[0,0,.01]])):
            with self.assertRaises(ValueError):planar_covariance(matrix)

    def test_6d_message_extracts_position_yaw_correlations(self):
        source=np.zeros((6,6));source[0,0]=.01;source[1,1]=.0025;source[5,5]=.001
        source[0,5]=source[5,0]=.001
        matrix=planar_covariance(source.ravel())
        self.assertEqual(matrix[0,2],.001)
        self.assertEqual(matrix[2,0],.001)


class RouteConfidenceTest(unittest.TestCase):
    def test_recorded_along_track_pause_stays_bounded_and_speed_limited(self):
        fixture=json.loads((Path(__file__).parent/'fixtures/route_covariance_recording.json').read_text())
        legacy_paused=False
        for item in fixture['observations']:
            covariance=planar_covariance(np.array(item['covariance']).reshape(3,3))
            across,along,major,yaw=project_uncertainty(covariance,item['route_heading_rad'])
            legacy_paused |= math.sqrt(max(covariance[0,0],covariance[1,1]))>.08
            state,_=ConfidenceGate().check(0.,item['pose_stamp_s'],across,yaw,0.,along_std=along,major_std=major)
            self.assertEqual(state,'GO')
            self.assertLess(uncertainty_speed_cap(.4,along),.4)
            self.assertLess(major,.16)
        self.assertTrue(legacy_paused)

    def test_corridor_longitudinal_ambiguity_slows_instead_of_lateral_fault(self):
        gate=ConfidenceGate()
        self.assertEqual(gate.check(1.,1.,.011,.035,.4,along_std=.095,major_std=.096)[0],'GO')
        self.assertLess(uncertainty_speed_cap(.4,.095),.4)
        self.assertGreater(uncertainty_speed_cap(.4,.095),.1)
        self.assertEqual(gate.check(2.,2.,.081,.035,.2,along_std=.095,major_std=.096)[0],'PAUSE')

    def test_longitudinal_limit_and_major_axis_hard_stop_remain(self):
        self.assertEqual(ConfidenceGate().check(0.,0.,.01,.03,0.,along_std=.13,major_std=.13)[0],'PAUSE')
        self.assertEqual(ConfidenceGate().check(0.,0.,.01,.03,0.,along_std=.17,major_std=.17)[0],'FAULT')
        self.assertEqual(uncertainty_speed_cap(.4,.08),.4)
        self.assertAlmostEqual(uncertainty_speed_cap(.4,.12),.1)


class CovarianceCollisionTest(unittest.TestCase):
    def route(self,angle=0.):
        s=np.arange(50)*.1
        return np.column_stack((s,s*math.cos(angle),s*math.sin(angle),np.full_like(s,angle)))

    def side_scan(self):
        angles=np.linspace(.4,1.4,80)
        return .4/np.sin(angles),angles[0],angles[1]-angles[0]

    def side_guard(self,covariance,angle=0.):
        ranges,a,step=self.side_scan()
        return swept_path_collision(ranges,a,step,.05,25.,self.route(angle),(0.,0.,angle),0,.4,
                                    pose_covariance=covariance)

    def test_along_spread_does_not_hide_transverse_or_yaw_risk(self):
        self.assertFalse(self.side_guard(np.diag([.10**2,.005**2,.001**2]))[0])
        self.assertTrue(self.side_guard(np.diag([.10**2,.08**2,.001**2]))[0])
        self.assertTrue(self.side_guard(np.diag([.01**2,.005**2,.08**2]))[0])

    def test_longitudinal_uncertainty_stops_before_a_forward_wall(self):
        angles=np.linspace(-.4,.4,81);ranges=(1.10-.1)/np.cos(angles)
        args=(ranges,angles[0],angles[1]-angles[0],.05,25.,self.route(),(0.,0.,0.),0,.4)
        self.assertFalse(swept_path_collision(*args)[0])
        self.assertTrue(swept_path_collision(*args,pose_covariance=np.diag([.10**2,.005**2,.001**2]))[0])

    def test_guard_covers_reaction_and_braking_before_forward_obstacle(self):
        angles=np.linspace(-.4,.4,81)
        for speed in (.2,.4,.8):
            reaction=.70
            distance=speed*reaction+speed*speed/(2*.7)
            obstacle=.46145+distance-.02
            ranges=(obstacle-.1)/np.cos(angles)
            self.assertTrue(swept_path_collision(ranges,angles[0],angles[1]-angles[0],
                .05,25.,self.route(),(0.,0.,0.),0,speed,reaction=reaction)[0])

    def test_stopping_horizon_increases_for_scan_age_and_contains_buffer(self):
        for speed in (0.,.2,.4,.8):
            expected=speed*.7+speed*speed/(2*.7)
            self.assertGreaterEqual(stopping_horizon(speed,.7),expected+.1-1e-10)
            self.assertGreaterEqual(stopping_horizon(speed,.7),stopping_horizon(speed,.35))

    def test_collision_budget_is_invariant_to_global_map_rotation(self):
        covariance=np.diag([.08**2,.01**2,.02**2])
        expected=self.side_guard(covariance)
        angle=.8;c,s=math.cos(angle),math.sin(angle)
        rotation=np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])
        actual=self.side_guard(rotation@covariance@rotation.T,angle)
        self.assertEqual(expected[0],actual[0])
        self.assertAlmostEqual(expected[1],actual[1],places=9)

    def test_corrupt_scan_geometry_cannot_pass_as_clear(self):
        ranges,a,step=self.side_scan()
        with self.assertRaises(ValueError):
            swept_path_collision(ranges,float('nan'),step,.05,25.,self.route(),(0.,0.,0.),0,.4)


class ProgressTest(unittest.TestCase):
    def test_estimator_jitter_counts_net_progress_across_route_seam(self):
        indices=[373,374,0,374,0,1]
        delta=sum(route_progress_delta(a,b,375) for a,b in zip(indices,indices[1:]))
        self.assertEqual(delta,3)
        self.assertEqual(route_progress_delta(20,19,375),-1)

    def test_real_progress_jump_still_rejected(self):
        with self.assertRaisesRegex(ValueError,'discontinuity'):
            route_progress_delta(20,100,375)


if __name__=='__main__':
    unittest.main()
