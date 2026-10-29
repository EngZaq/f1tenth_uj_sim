#!/usr/bin/env python3
"""Regression: optimizer CLI must execute dense-footprint clearance, write only passing routes.

Uses the saved October 28 route and map as a reproducible integration fixture.
No ROS/Gym launch and no vehicle movement are involved.
"""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
MAP = ROOT / 'results/missions/20261028_112407/new_track.yaml'
ROUTE = ROOT / 'results/missions/20261028_112407/new_track_mission_forward_feasible.csv'
OPTIMIZER = ROOT / 'tools/planning/optimize_vehicle_route.py'


class OptimizerCLITest(unittest.TestCase):
    def run_candidate(self, margin, iterations):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / 'candidate.csv'
            command = [sys.executable, str(OPTIMIZER), str(MAP), str(ROUTE),
                       '--margin', str(margin), '--iterations', str(iterations),
                       '--out', str(output)]
            result = subprocess.run(command, capture_output=True, text=True,
                                    timeout=120)
            created = output.exists()
            rows = output.read_text().splitlines() if created else []
            return result, created, rows

    def test_accepted_route_checks_dense_footprint_and_writes_csv(self):
        result, created, rows = self.run_candidate(0.15, 40)
        self.assertEqual(result.returncode, 0, result.stdout + '\n' + result.stderr)
        self.assertIn('between waypoints', result.stdout)
        self.assertTrue(created)
        self.assertGreater(len(rows), 370)
        self.assertIn('s_m;x_m;y_m;psi_rad;kappa_1pm;vx_mps;ax_mps2', rows[1])

    def test_rejected_route_writes_nothing(self):
        result, created, _ = self.run_candidate(0.25, 3)
        self.assertEqual(result.returncode, 2, result.stdout + '\n' + result.stderr)
        self.assertIn('REJECTED', result.stdout)
        self.assertFalse(created)


if __name__ == '__main__':
    unittest.main()
