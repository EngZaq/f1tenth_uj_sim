"""Recorded open/closed map regression for the exploration completion gate."""
import base64
import json
import sys
import unittest
import zlib
from pathlib import Path
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/mission'))
from map_coverage import corridor_coverage


class MapCoverageTest(unittest.TestCase):
    def ring(self):
        grid=np.full((100,100),100,dtype=np.int8)
        grid[10:90,10:90]=0
        grid[35:65,35:65]=100
        return grid

    def check_grid(self,grid):
        return corridor_coverage(grid.ravel(),grid.shape[1],grid.shape[0],.05)

    def test_recorded_ci_gap_blocks_and_recorded_laptop_loop_passes(self):
        fixtures=json.loads((Path(__file__).parent/'fixtures/map_coverage_recordings.json').read_text())
        for item in fixtures:
            with self.subTest(item['name']):
                data=np.frombuffer(zlib.decompress(base64.b64decode(item['data'])),dtype=np.int8)
                result=corridor_coverage(data,item['width'],item['height'],item['resolution'])
                self.assertEqual(result['closed'],item['expected_closed'])

    def test_known_free_ring_closes_with_clearance(self):
        result=self.check_grid(self.ring())
        self.assertTrue(result['closed'])
        self.assertGreater(result['largest_interior_area_m2'],.5)

    def test_unknown_wedge_is_not_inferred_free(self):
        grid=self.ring()
        grid[10:35,46:54]=-1
        self.assertFalse(self.check_grid(grid)['closed'])
        # Only explicit new free observations can close this missing sector.
        grid[grid==-1]=0
        self.assertTrue(self.check_grid(grid)['closed'])

    def test_all_free_or_all_unknown_is_not_an_observed_loop(self):
        for value in (0,-1):
            self.assertFalse(self.check_grid(np.full((100,100),value))['closed'])

    def test_invalid_grid_is_rejected(self):
        with self.assertRaises(ValueError):
            corridor_coverage([0]*10,100,100,.05)
        with self.assertRaises(ValueError):
            corridor_coverage([0],1,1,float('nan'))
        with self.assertRaises(ValueError):
            corridor_coverage([float('nan')],1,1,.05)


if __name__=='__main__':
    unittest.main()
