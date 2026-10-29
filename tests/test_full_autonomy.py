"""Offline coverage of mission sequencing, authority, freshness and geometry."""
import contextlib
import importlib.util
import io
import json
import math
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
sys.path.insert(0,str(ROOT/'tools/localization'))
sys.path.insert(0,str(ROOT/'tools/planning'))
from sim_geometry import LASER, laser_points, validate_reference_transform


def load(name, relative):
    spec=importlib.util.spec_from_file_location(name,ROOT/relative)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


WATCH=load('watchdog_test','tools/navigation/drive_watchdog.py')
READY=load('readiness_test','tools/localization/wait_amcl_ready.py')
FULL=load('full_mission_test','tools/mission/full_autonomy.py')
PLAN=load('prepare_route_test','tools/planning/prepare_vehicle_route.py')
NS=types.SimpleNamespace


class GeometryTest(unittest.TestCase):
    def test_recorded_scan_agrees_with_measured_mount_not_old_assumption(self):
        from PIL import Image
        import yaml
        fixture=json.loads((ROOT/'tests/fixtures/recorded_laser_geometry.json').read_text())
        meta=yaml.safe_load((ROOT/fixture['map_file']).read_text())
        occupied=np.asarray(Image.open(ROOT/'maps'/meta['image']).convert('L'))<80
        measured=np.array([np.nan if v is None else v for v in fixture['ranges']])
        angles=fixture['angle_min']+np.arange(len(measured))*fixture['angle_increment']+fixture['yaw_sim']
        distance=np.arange(fixture['range_min'],fixture['range_max'],meta['resolution']/4)
        def error(laser_x):
            ox=fixture['x_sim']+laser_x*math.cos(fixture['yaw_sim'])
            oy=fixture['y_sim']+laser_x*math.sin(fixture['yaw_sim'])
            xx=ox+np.cos(angles)[:,None]*distance; yy=oy+np.sin(angles)[:,None]*distance
            col=np.floor((xx-meta['origin'][0])/meta['resolution']).astype(int)
            row=occupied.shape[0]-1-np.floor((yy-meta['origin'][1])/meta['resolution']).astype(int)
            inside=(row>=0)&(row<occupied.shape[0])&(col>=0)&(col<occupied.shape[1])
            hit=np.ones_like(inside);hit[inside]=occupied[row[inside],col[inside]]
            predicted=distance[np.argmax(hit,axis=1)]
            predicted[~hit.any(axis=1)]=fixture['range_max']
            valid=np.isfinite(measured)&(measured<fixture['range_max']-.05)
            return float(np.sqrt(np.mean((predicted[valid]-measured[valid])**2)))
        self.assertLess(error(LASER['x_m']),.04)
        self.assertGreater(error(.275),.20)

    def test_actual_transform_mismatch_is_rejected(self):
        transform=NS(translation=NS(x=.1,y=0.),rotation=NS(x=0.,y=0.,z=0.,w=1.))
        validate_reference_transform(transform)
        transform.translation.x=.275
        with self.assertRaisesRegex(RuntimeError,'disagrees'):
            validate_reference_transform(transform)
        transform.translation.x=.1;transform.rotation.x=.1
        with self.assertRaisesRegex(RuntimeError,'Nonplanar'):
            validate_reference_transform(transform)
    def test_actual_measured_mount_used_by_seed_and_guard(self):
        self.assertEqual(LASER['x_m'],.1)
        result=laser_points(np.array([1.]),np.array([0.]))
        np.testing.assert_allclose(result,[[1.1,0.]])
        seed=load('seed_test','tools/mission/map_scan_seed.py')
        scan=NS(angle_min=0.,angle_increment=.01,ranges=[1.]*200,range_min=.05,range_max=25.)
        hits,inside=seed.scan_points(scan)
        np.testing.assert_allclose(hits[0],[1.1,0.])
        np.testing.assert_allclose(inside[0],[.85,0.])
        safety=load('trajectory_test','tools/navigation/trajectory_safety.py')
        self.assertEqual(safety.swept_path_collision.__defaults__[1],.1)


class WatchdogTest(unittest.TestCase):
    def test_fresh_command_then_controller_death_commands_zero(self):
        p=WATCH.DrivePolicy();p.scan_seen=0.
        p.accept(0.,.4,.1,.01)
        self.assertEqual(p.output(.1)[:2],(.4,.1))
        self.assertEqual(p.output(.3)[:2],(0.,0.))
        self.assertFalse(p.fault)

    def test_scan_loss_and_collision_latch(self):
        p=WATCH.DrivePolicy();p.scan_seen=0.;p.accept(0.,.4,0.)
        self.assertEqual(p.output(.5)[:2],(0.,0.))
        self.assertIn('LiDAR',p.fault)
        p.scan_seen=.6;p.accept(.6,.4,0.)
        self.assertEqual(p.output(.6)[:2],(0.,0.))
        p=WATCH.DrivePolicy();p.fault='simulator collision';p.scan_seen=0.
        p.accept(0.,.4,0.);self.assertEqual(p.output(0.)[:2],(0.,0.))

    def test_stale_future_invalid_and_competing_sources(self):
        p=WATCH.DrivePolicy();p.scan_seen=0.;p.accept(0.,.4,0.,2.)
        self.assertEqual(p.output(0.)[:2],(0.,0.))
        p.accept(0.,.4,0.,-.2);self.assertEqual(p.output(0.)[:2],(0.,0.))
        p.accept(0.,float('nan'),0.);self.assertTrue(p.fault)
        p=WATCH.DrivePolicy();p.scan_seen=0.;p.accept(0.,.4,0.)
        self.assertEqual(p.output(0.,True)[:2],(0.,0.))


class ReadyTest(unittest.TestCase):
    def test_four_fresh_distinct_estimates_required(self):
        p=READY.Readiness()
        self.assertFalse(p.observe(1.,.1,.004,0.,0.,.004,.001))
        for _ in range(5):
            self.assertFalse(p.observe(1.,.1,.004,0.,0.,.004,.001))
        self.assertFalse(p.observe(2.,.1,.004,0.,0.,.004,.001))
        self.assertFalse(p.observe(3.,.1,.004,0.,0.,.004,.001))
        self.assertTrue(p.observe(4.,.1,.004,0.,0.,.004,.001))

    def test_unsafe_or_indefinite_covariance_never_ready(self):
        for covariance in [(.01,0.,0.,.01,.001),(.004,.02,.02,.004,.001),
                           (.004,0.,0.,.004,-1.)]:
            p=READY.Readiness()
            for i in range(6):
                self.assertFalse(p.observe(float(i),.1,*covariance))


class FakeMission(FULL.Mission):
    def __init__(self,folder,fail_plan=False,record=True,race_exit=0,mapping_laps=1):
        super().__init__(folder)
        self.started=[];self.called=[];self.fail_plan=fail_plan;self.record=record
        self.race_exit=race_exit
        self.mapping_laps=mapping_laps
    def command(self,args,**kwargs):
        a=[str(x) for x in args];self.called.append(a);joined=' '.join(a)
        if self.fail_plan and 'extract_robust_centerline.py' in joined:
            raise RuntimeError('deliberate route failure')
        if 'map_saver_cli' in a:
            prefix=Path(a[a.index('-f')+1]);prefix.with_suffix('.yaml').write_text('map fixture')
            prefix.with_suffix('.pgm').write_text('image fixture')
        out=('Publisher count: 1' if '/ego_racecar/odom' in a else
             'Publisher count: 0' if '/drive' in a else '225\n---\n' if 'info.width' in a else '')
        return NS(returncode=0,stdout=out,stderr='')
    def start(self,name,args):
        a=[str(x) for x in args];self.started.append((name,a))
        self.jobs[name]=NS(poll=lambda:self.race_exit if name=='race' else None,returncode=self.race_exit)
        if name=='race':
            (self.folder/'race.log').write_text('RESUME STOPPED: injected child handoff failure\n' if self.race_exit else 'lap log\n')
        if name=='lap_watcher':
            (self.folder/'lap_pose.json').write_text(json.dumps({
                'distance_odom_m':37.6*self.mapping_laps,'estimated_lap_length_m':37.6}))
        if name=='race' and self.record:
            target=self.folder/'race_fixture';target.mkdir()
            (target/'race_result.json').write_text(json.dumps({'completed_estimated_laps':3}))
    def stop(self,name):
        self.jobs.pop(name,None)
    def wait(self,predicate,timeout,ignore=()):
        # Simulate the settling interval without wall-time delay.
        if self.stage!='SAVING_MAP':
            if not predicate():raise RuntimeError('fake readiness did not complete')
        self.check(ignore)
    def zero(self):
        pass


class SequenceTest(unittest.TestCase):
    def test_single_bundle_contains_nested_run_outputs(self):
        import zipfile
        with tempfile.TemporaryDirectory() as tmp:
            ctx=FULL.Mission(Path(tmp)/'mission')
            (ctx.folder/'race_1').mkdir()
            (ctx.folder/'race_1/log.txt').write_text('recorded evidence')
            ctx.enter('STOPPED',reason='test')
            bundle=ctx.package()
            self.assertEqual(bundle.parent,ctx.folder.parent)
            with zipfile.ZipFile(bundle) as archive:
                self.assertIn('race_1/log.txt',archive.namelist())
                self.assertIn('mission_status.json',archive.namelist())

    def execute(self,folder,**kwargs):
        ctx=FakeMission(folder,**kwargs)
        args=NS(autonomous=True,start_simulator=True,laps=3,speed_cap=.4,explore_speed=.18)
        with patch('builtins.input',side_effect=AssertionError('automatic mode prompted')),contextlib.redirect_stdout(io.StringIO()):
            FULL.run_mission(ctx,args)
        return ctx

    def test_entire_stage_sequence_no_prompts_and_private_drive_handoff(self):
        with tempfile.TemporaryDirectory() as tmp:
            ctx=self.execute(Path(tmp)/'mission')
            self.assertEqual([e['stage'] for e in ctx.events],
                ['PREFLIGHT','EXPLORING','SAVING_MAP','PLANNING','WAITING_FOR_LOCALIZATION_HANDOFF',
                 'LOCALIZING_AND_RACING','COMPLETED_ESTIMATED_LAPS'])
            start=dict(ctx.started)
            self.assertIn('/drive:=/mission/drive',start['explorer'])
            self.assertIn('/mission/drive',start['race'])
            self.assertIn('--autonomous',start['race'])
            self.assertNotIn('slam',ctx.jobs)
            self.assertEqual(json.loads((ctx.folder/'laser_geometry.json').read_text()),LASER)
            self.assertTrue((ctx.folder/'autonomy_summary.txt').exists())
            self.assertTrue(json.loads((ctx.folder/'mapping_handoff.json').read_text())['ready'])

    def test_failed_planning_cannot_start_racing(self):
        with tempfile.TemporaryDirectory() as tmp:
            ctx=FakeMission(Path(tmp)/'mission',fail_plan=True)
            with contextlib.redirect_stdout(io.StringIO()),self.assertRaisesRegex(RuntimeError,'route failure'):
                FULL.run_mission(ctx,NS(autonomous=True,start_simulator=True,laps=3,speed_cap=.4,explore_speed=.18))
            self.assertNotIn('race',dict(ctx.started))

    def test_process_exit_without_completed_lap_record_is_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(RuntimeError,'completed-lap record'):
                self.execute(Path(tmp)/'mission',record=False)

    def test_additional_mapping_lap_does_not_double_expected_route_length(self):
        with tempfile.TemporaryDirectory() as tmp:
            ctx=self.execute(Path(tmp)/'mission',mapping_laps=2)
            extract=next(a for a in ctx.called if 'extract_robust_centerline.py' in ' '.join(a))
            self.assertEqual(extract[extract.index('--expected-length')+1],'37.6')

    def test_fast_child_failure_is_drained_and_explained(self):
        with tempfile.TemporaryDirectory() as tmp:
            ctx=FakeMission(Path(tmp)/'mission',race_exit=1)
            output=io.StringIO()
            with contextlib.redirect_stdout(output), self.assertRaisesRegex(RuntimeError,'injected child handoff failure'):
                FULL.run_mission(ctx,NS(autonomous=True,start_simulator=True,laps=3,speed_cap=.4,explore_speed=.18))
            self.assertIn('RESUME STOPPED: injected child handoff failure',output.getvalue())


class RoutePreparationTest(unittest.TestCase):
    def test_failed_all_candidates_writes_rejection_without_route(self):
        fixture=ROOT/'results/missions/20261028_112407/new_track_mission_forward_feasible.csv'
        with tempfile.TemporaryDirectory() as tmp:
            output=Path(tmp)/'candidate.csv';report=Path(tmp)/'report.json'
            with patch.object(PLAN,'improve',return_value=2),contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(RuntimeError,'No automatically prepared route'):
                    PLAN.prepare('unused.yaml',fixture,output,report)
            self.assertFalse(output.exists())
            self.assertFalse(json.loads(report.read_text())['accepted'])


if __name__=='__main__':
    unittest.main()
