"""ROS-independent AMCL uncertainty gate for experimental simulator.

A temporary covariance increase holds the car at zero speed; it may resume
ONLY after multiple fresh, low-uncertainty estimates while stationary.
Severe/non-finite uncertainty and expired pauses produce a latched fault.
This never overrides LiDAR, TF, odometry, or path safety checks.
"""
import math


class ConfidenceGate:
    def __init__(self, soft_pos=0.08, soft_yaw_deg=10.0, hard_pos=0.16,
                 hard_yaw_deg=20.0, timeout=4.0, good_samples=4, soft_along=.12):
        self.soft_pos = float(soft_pos)
        self.soft_yaw = math.radians(soft_yaw_deg)
        self.hard_pos = float(hard_pos)
        self.hard_yaw = math.radians(hard_yaw_deg)
        self.timeout = float(timeout)
        self.good_samples = int(good_samples)
        self.soft_along=float(soft_along)
        self.paused_since = None
        self.good_count = 0
        self.last_stamp = None

    def check(self, now, stamp, position_std, yaw_std, vehicle_speed, *,
              along_std=None, major_std=None):
        # Legacy scalar callers remain supported. The driving node supplies
        # route-normal position sigma plus tangent and rotation-invariant major
        # sigmas. Collision prediction separately uses local body-frame motion.
        extra=tuple(v for v in (along_std,major_std) if v is not None)
        if not all(map(math.isfinite, (now, stamp, position_std, yaw_std, vehicle_speed)+extra)):
            return 'FAULT', 'nonfinite AMCL covariance/speed/time'
        if position_std < 0 or yaw_std < 0 or any(v<0 for v in extra):
            return 'FAULT', 'negative AMCL uncertainty'
        major=position_std if major_std is None else max(position_std,major_std)
        detail=(f'pos_std={position_std:.3f}m' if along_std is None else
                f'lateral_std={position_std:.3f}m along_std={along_std:.3f}m major_std={major:.3f}m')
        if major >= self.hard_pos or yaw_std >= self.hard_yaw:
            return 'FAULT', (
                f'AMCL severely uncertain: {detail} '
                f'yaw_std={math.degrees(yaw_std):.1f}deg')
        unsafe = (position_std > self.soft_pos or yaw_std > self.soft_yaw or
                  (along_std is not None and along_std > self.soft_along))
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
                f'{detail} yaw_std={math.degrees(yaw_std):.1f}deg')
        # Recovery uses the same admissible uncertainty as normal driving.
        # Four fresh stationary samples provide dwell hysteresis. A hidden
        # 20% tighter exit bound stranded the recorded run around 0.070 m even
        # though it was back inside the unchanged 0.080 m driving gate.
        recovered = (position_std <= self.soft_pos and
                     yaw_std <= self.soft_yaw and
                     (along_std is None or along_std <= self.soft_along) and
                     abs(vehicle_speed) <= 0.08)
        if self.last_stamp is None or stamp > self.last_stamp:
            self.last_stamp = stamp
            self.good_count = self.good_count + 1 if recovered else 0
        elif stamp < self.last_stamp or not recovered:
            self.good_count = 0
        if self.good_count >= self.good_samples:
            self.paused_since = None
            self.good_count = 0
            return 'GO', 'AMCL confidence recovered'
        return 'PAUSE', (
            f'AMCL confidence recovery {self.good_count}/{self.good_samples} '
            f'{detail} '
            f'yaw_std={math.degrees(yaw_std):.1f}deg')


def uncertainty_speed_cap(nominal, along_std, slowdown_start=.08, pause_limit=.12):
    if (not all(math.isfinite(v) for v in (nominal,along_std,slowdown_start,pause_limit)) or
            nominal<=0 or along_std<0 or not 0<slowdown_start<pause_limit):
        raise ValueError('Invalid uncertainty speed-cap inputs')
    fraction=max(.25,min(1.,(pause_limit-along_std)/(pause_limit-slowdown_start)))
    return nominal*fraction
