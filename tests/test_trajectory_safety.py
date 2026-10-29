import math
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools' / 'navigation'))
from trajectory_safety import swept_path_collision


def test_path_obstacle_and_side_wall():
    route = np.array([[i * 0.1, i * 0.1, 0.0, 0.0]
                      for i in range(101)])
    n = 819
    minimum_angle = -2.356
    step = 4.712 / (n - 1)
    def scan(obstacle_x, obstacle_y):
        ranges = np.full(n, 10.0)
        a = math.atan2(obstacle_y, obstacle_x - 0.275)
        index = int(round((a - minimum_angle) / step))
        ranges[index] = math.hypot(obstacle_y, obstacle_x - 0.275)
        return ranges
    blocked, *_ = swept_path_collision(
        scan(0.8, 0.0), minimum_angle, step, .05, 25.,
        route, (0., 0., 0.), 0, .5, laser_x=.275)
    assert blocked
    blocked, *_ = swept_path_collision(
        scan(0.8, 0.8), minimum_angle, step, .05, 25.,
        route, (0., 0., 0.), 0, .5, laser_x=.275)
    assert not blocked


if __name__ == '__main__':
    test_path_obstacle_and_side_wall()
    print('PASS: trajectory collision test')
