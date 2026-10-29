#!/usr/bin/env python3
"""Regression: older run without truth CSV must still give usable diagnostics."""
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / 'tools/localization/summarize_sim_localization_audit.py'


class SummaryTest(unittest.TestCase):
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


if __name__ == '__main__':
    unittest.main()
