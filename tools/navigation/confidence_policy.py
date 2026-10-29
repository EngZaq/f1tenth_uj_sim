"""ROS-independent AMCL uncertainty gate for experimental simulator.

A temporary covariance increase holds the car at zero speed; it may resume
ONLY after multiple fresh, low-uncertainty estimates while stationary.
Severe/non-finite uncertainty and expired pauses produce a latched fault.
This never overrides LiDAR, TF, odometry, or path safety checks.
"""
import math


class ConfidenceGate:
    def __init__(self, soft_pos=0.08, soft_yaw_deg=10.0, hard_pos=0.16,
                 hard_yaw_deg=20.0, timeout=4.0, good_samples=4):
        self.soft_pos = float(soft_pos)
        self.soft_yaw = math.radians(soft_yaw_deg)
        self.hard_pos = float(hard_pos)
        self.hard_yaw = math.radians(hard_yaw_deg)
        self.timeout = float(timeout)
        self.good_samples = int(good_samples)
        self.paused_since = None
        self.good_count = 0
        self.last_stamp = None

    def check(self, now, stamp, position_std, yaw_std, vehicle_speed):
        if not all(map(math.isfinite, (position_std, yaw_std, vehicle_speed))):
            return 'FAULT', 'nonfinite AMCL covariance/speed'
        if position_std < 0 or yaw_std < 0:
            return 'FAULT', 'negative AMCL uncertainty'
        if position_std >= self.hard_pos or yaw_std >= self.hard_yaw:
            return 'FAULT', (
                f'AMCL severely uncertain: pos_std={position_std:.3f}m '
                f'yaw_std={math.degrees(yaw_std):.1f}deg')
        unsafe = position_std > self.soft_pos or yaw_std > self.soft_yaw
        if self.paused_since is None:
            if not unsafe:
                return 'GO', 'confidence OK'
            self.paused_since = now
            self.good_count = 0
            self.last_stamp = stamp
            return 'PAUSE', 'AMCL confidence temporarily low'

        if now - self.paused_since >= self.timeout:
            return 'FAULT', (
                f'AMCL did not relocalize in {self.timeout:.1f}s: '
                f'pos_std={position_std:.3f}m yaw_std={math.degrees(yaw_std):.1f}deg')
        recovered = (position_std <= 0.8 * self.soft_pos and
                     yaw_std <= 0.8 * self.soft_yaw and
                     abs(vehicle_speed) <= 0.08)
        if stamp != self.last_stamp:
            self.last_stamp = stamp
            self.good_count = self.good_count + 1 if recovered else 0
        if self.good_count >= self.good_samples:
            self.paused_since = None
            self.good_count = 0
            return 'GO', 'AMCL confidence recovered'
        return 'PAUSE', (
            f'AMCL confidence recovery {self.good_count}/{self.good_samples} '
            f'pos_std={position_std:.3f}m '
            f'yaw_std={math.degrees(yaw_std):.1f}deg')
