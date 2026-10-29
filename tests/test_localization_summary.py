#!/usr/bin/env python3
"""Regression: older run without truth CSV must still give usable diagnostics."""
import subprocess
import csv
import importlib.util
import math
import sys
import tempfile
import unittest
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / 'tools/localization/summarize_sim_localization_audit.py'
SPEC = importlib.util.spec_from_file_location('localization_summary', SCRIPT)
SUMMARY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SUMMARY)


def sample(t, x=0.0, y=0.0, lag=0.001, error=0.0):
    return dict(stamp_s=t, truth_x_sim=x, truth_y_sim=y, truth_yaw_deg=0,
                amcl_x_map=x, amcl_y_map=y, amcl_yaw_deg=0,
                truth_pair_lag_s=lag, amcl_relative_pos_error_m=error)


class SummaryTest(unittest.TestCase):
    def test_stationary_samples_cannot_hide_moving_error(self):
        # A long preparation period must not dominate the moving-only result.
        rows = [sample(100+i*.1) for i in range(101)]
        rows += [sample(110+i*.1, x=i*.04, error=.25) for i in range(1, 11)]
        groups, counts, duration, distance = SUMMARY.motion_audit(rows)
        self.assertEqual(len(groups['STATIONARY']), 100)
        self.assertEqual(len(groups['MOVING']), 10)
        self.assertEqual(SUMMARY.finite_values(groups['MOVING'],
                                              'amcl_relative_pos_error_m'), [.25]*10)
        self.assertAlmostEqual(duration['MOVING'], 1.0)
        self.assertAlmostEqual(distance['MOVING'], .4)
        self.assertEqual(counts['no_interval'], 1)

    def test_motion_uses_truth_not_command(self):
        rows = [sample(1), sample(1.1)]
        rows[-1]['cmd_speed_mps'] = 2.0
        groups, _, _, _ = SUMMARY.motion_audit(rows)
        self.assertEqual(len(groups['STATIONARY']), 1)
        self.assertFalse(groups['MOVING'])

    def test_late_and_missing_pairs_break_intervals(self):
        rows = [sample(1), sample(1.1, x=.04, lag=.18),
                sample(1.2, x=.08), sample(1.3, x=.12),
                sample(1.4, x=.16, lag=float('nan')),
                sample(1.5, x=.20)]
        groups, counts, _, distance = SUMMARY.motion_audit(rows)
        self.assertEqual(len(groups['MOVING']), 1)
        self.assertEqual(counts['late_pair'], 1)
        self.assertEqual(counts['invalid'], 1)
        self.assertAlmostEqual(distance['MOVING'], .04)

    def test_gaps_duplicates_and_clock_resets_do_not_create_motion(self):
        rows = [sample(1), sample(3, x=1), sample(3, x=2),
                sample(.1, x=5), sample(.2, x=6)]
        groups, counts, _, _ = SUMMARY.motion_audit(rows)
        self.assertFalse(groups['MOVING'])
        self.assertEqual(counts['gap'], 1)
        self.assertEqual(counts['nonmonotonic'], 2)

    def test_signed_components_respect_rotated_frame_and_heading(self):
        first = sample(1, x=1, y=2)
        first.update(truth_yaw_deg=30, amcl_x_map=8, amcl_y_map=-2,
                     amcl_yaw_deg=120)
        second = sample(1.1, x=1.04, y=2)
        h = math.radians(120)
        second.update(truth_yaw_deg=30, amcl_yaw_deg=120,
                      amcl_x_map=8+.12*math.cos(h)-.03*math.sin(h),
                      amcl_y_map=-1.96+.12*math.sin(h)+.03*math.cos(h))
        groups, _, _, _ = SUMMARY.motion_audit([first, second])
        result = groups['MOVING'][0]
        self.assertAlmostEqual(result['amcl_relative_vehicle_forward_error_m'], .12)
        self.assertAlmostEqual(result['amcl_relative_vehicle_left_error_m'], .03)

    def test_stationary_only_cli_rejects_driving_conclusion(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'truth.csv'
            rows = [sample(1), sample(1.1)]
            with path.open('w', newline='') as fp:
                writer = csv.DictWriter(fp, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            result = subprocess.run([sys.executable, str(SCRIPT), str(path)],
                                    text=True, capture_output=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('NO ACCEPTED MOVING EVIDENCE', result.stdout)
            self.assertIn('not absolute map registration', result.stdout)
            self.assertIn('no covariance-consistency', result.stdout)

    def test_invalid_analysis_limits_have_clear_cli_error(self):
        for value in ('nan', 'inf', '-1', '0'):
            result = subprocess.run([sys.executable, str(SCRIPT),
                                     '--max-pair-lag-s', value],
                                    text=True, capture_output=True, timeout=10)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('must be finite and positive', result.stderr)
            self.assertNotIn('Traceback', result.stderr)

    def test_legacy_csv_missing_truth_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            drive=root/'estimated_drive.csv'
            drive.write_text(
                'ros_time,state,reason,progress,lap,cmd_speed_mps,cte_m,amcl_pos_std_m\n'
                '1,DRIVE,OK,10,0,0.40,0.005,0.055\n'
                '2,STOP,LOCALIZATION PAUSE: high covariance,10,0,0,NaN,0.083\n')
            command=[sys.executable,str(SCRIPT),
                     str(root/'localization_truth_audit.csv'),
                     '--controller-csv',str(drive)]
            result=subprocess.run(command,text=True,capture_output=True,timeout=10)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertIn('NO TRUTH AUDIT IN THIS RUN',result.stdout)
            self.assertIn('LOCALIZATION PAUSE',result.stdout)
            self.assertIn('Maximum progress: 10',result.stdout)

    def test_missing_both_produces_clear_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            result=subprocess.run(
                [sys.executable,str(SCRIPT),str(Path(tmp)/'missing.csv')],
                text=True,capture_output=True,timeout=10)
            self.assertNotEqual(result.returncode,0)
            self.assertIn('No readable CSV found',result.stderr)
            self.assertNotIn('Traceback',result.stderr)

    def test_original_archived_controller_schema(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'drive.csv'
            path.write_text(
                'wall_time_s,state,reason,waypoint_index,commanded_speed_mps,estimated_cte_m\n'
                '100,DRIVE,OK,56,0.15,0.17\n')
            result = subprocess.run([sys.executable, str(SCRIPT),
                                     '--controller-csv', str(path)],
                                    text=True, capture_output=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('Maximum waypoint index (not lap progress): 56', result.stdout)
            self.assertIn('estimated_cte_m', result.stdout)
            self.assertIn('commanded_speed_mps', result.stdout)
            self.assertIn('NO TRUTH AUDIT IN THIS RUN', result.stdout)


if __name__ == '__main__':
    unittest.main()
