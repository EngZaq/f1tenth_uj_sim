"""Pure route-following policy, without ROS dependencies.

Inputs are map-frame rear-axle pose and s,x,y,yaw,kappa,v,accel route rows.
No simulator ground truth is used here.
"""
import math
import numpy as np


def normalized_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


class ProfiledRoute:
    def __init__(self, csv_path, wheelbase=0.3302):
        self.data = np.loadtxt(csv_path, delimiter=';', comments='#')
        if (self.data.ndim != 2 or self.data.shape[0] < 30 or
                self.data.shape[1] < 7 or
                not np.isfinite(self.data).all()):
            raise ValueError('Required finite s;x;y;yaw;kappa;speed;accel route')
        if not np.all(np.diff(self.data[:, 0]) > 0):
            raise ValueError('Non-increasing route arc length')
        if not np.all(self.data[:, 5] > 0):
            raise ValueError('Invalid speed profile')
        self.xy = self.data[:, 1:3]
        self.n = len(self.data)
        self.length = float(self.data[-1, 0] + np.linalg.norm(self.xy[-1] - self.xy[0]))
        self.wheelbase = wheelbase

    def nearest(self, x, y, previous=None):
        dist = np.sum((self.xy - np.array([x, y]))**2, axis=1)
        if previous is None:
            return int(np.argmin(dist))
        # Prevent a spurious map-frame branch hop except for a start/restart.
        indices = np.array([(previous + j) % self.n for j in range(-15, 35)])
        candidate = int(indices[np.argmin(dist[indices])])
        if math.sqrt(float(dist[candidate])) > 0.4:
            raise ValueError('Outside local path tracking corridor')
        return candidate

    def error(self, x, y, i):
        # Segment-based cross track error near progress, including cyclic closure.
        best = float('inf')
        for offset in (-2, -1, 0, 1, 2):
            j = (i + offset) % self.n
            a, b = self.xy[j], self.xy[(j + 1) % self.n]
            v = b - a
            den = float(np.dot(v, v))
            t = max(0., min(1., float(np.dot(np.array([x, y]) - a, v) / max(den, 1e-10))))
            best = min(best, float(np.linalg.norm(np.array([x, y]) - (a + t * v))))
        return best

    def aim(self, x, y, yaw, i, lookahead, steer_limit=0.4189):
        s_target = (float(self.data[i, 0]) + lookahead) % self.length
        extended_s = np.r_[self.data[:, 0], self.length]
        px = np.interp(s_target, extended_s, np.r_[self.xy[:, 0], self.xy[0, 0]])
        py = np.interp(s_target, extended_s, np.r_[self.xy[:, 1], self.xy[0, 1]])
        dx, dy = px - x, py - y
        lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
        steering = math.atan2(2 * self.wheelbase * lateral, max(0.01, dx*dx + dy*dy))
        return max(-steer_limit, min(steer_limit, steering))

    def speed_at(self, i, lookahead_points=6):
        return float(np.min([self.data[(i + j) % self.n, 5]
                             for j in range(lookahead_points)]))


def ramp_velocity(previous, desired, dt, accel=0.5, decel=0.8):
    dt = max(0.0, min(dt, 0.2))
    upper = previous + accel * dt
    lower = max(0., previous - decel * dt)
    return max(lower, min(upper, max(0., desired)))


def route_progress_delta(previous, current, count, max_step=12):
    """Signed local progress; estimator jitter must not become almost a lap."""
    if count<3 or not (0<=previous<count and 0<=current<count):
        raise ValueError('Invalid route progress indices')
    delta=(current-previous)%count
    if delta>count//2:
        delta-=count
    if abs(delta)>max_step:
        raise ValueError('route progress discontinuity')
    return delta


def tracking_lookahead(speed, base=.30, gain=.20):
    """Low-speed technical-course target distance in metres."""
    if not all(math.isfinite(v) for v in (speed,base,gain)) or speed<0 or base<=0 or gain<0:
        raise ValueError('Invalid tracking lookahead inputs')
    return max(.25,min(.60,base+gain*speed))
