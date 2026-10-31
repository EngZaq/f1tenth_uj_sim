#!/usr/bin/env python3
"""Bounded, estimated-pose-only readiness check before automatic simulator arming."""
import argparse
import json
import math
import time
from pathlib import Path
from stationary_localization_probe import RefreshSchedule


class Readiness:
    def __init__(self):
        self.good = 0
        self.last_stamp = None
        self.reason = 'waiting for AMCL'

    def observe(self, stamp, age, xx, xy, yx, yy, yaw_variance, path_yaw=None):
        confidence = math.sqrt(max(0.,xx,yy)) <= .08
        if path_yaw is not None and all(math.isfinite(v) for v in (xx,xy,yx,yy,path_yaw)):
            c,s=math.cos(path_yaw),math.sin(path_yaw)
            across=s*s*xx-c*s*(xy+yx)+c*c*yy
            along=c*c*xx+c*s*(xy+yx)+s*s*yy
            major=.5*(xx+yy+math.hypot(xx-yy,xy+yx))
            confidence=(across<=.08**2 and along<=.12**2 and major<=.16**2)
        valid = (all(math.isfinite(v) for v in (stamp,age,xx,xy,yx,yy,yaw_variance)) and
                 xx >= 0 and yy >= 0 and yaw_variance >= 0 and
                 abs(xy-yx) <= 1e-9 and xx*yy >= (.5*(xy+yx))**2-1e-12 and
                 -.05 <= age <= .65 and confidence and
                 math.sqrt(yaw_variance) <= math.radians(10))
        if stamp != self.last_stamp:
            self.good = self.good+1 if valid and (self.last_stamp is None or stamp > self.last_stamp) else 0
            self.last_stamp = stamp
        elif not valid:
            self.good = 0
        self.reason = f'fresh confident AMCL observations {self.good}/4'
        return self.good >= 4


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--route', type=Path)
    args = parser.parse_args()
    route = None
    if args.route is not None:
        import numpy as np
        route=np.loadtxt(args.route,delimiter=';',comments='#')
        if route.ndim!=2 or route.shape[1]<4 or not np.isfinite(route).all():
            raise ValueError('Invalid readiness route')
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from std_srvs.srv import Empty
    rclpy.init(args=[])
    node = Node('automatic_amcl_readiness')
    gate, refresh = Readiness(), RefreshSchedule()
    client = node.create_client(Empty, '/request_nomotion_update')
    ready = False
    def callback(message):
        nonlocal ready
        if message.header.frame_id != 'slam_map':
            return
        c = message.pose.covariance
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        now = node.get_clock().now().nanoseconds*1e-9
        path_yaw=None
        if route is not None:
            p=message.pose.pose.position
            index=int(np.argmin((route[:,1]-p.x)**2+(route[:,2]-p.y)**2))
            path_yaw=float(route[index,3])
        ready = gate.observe(stamp, now-stamp, c[0], c[1], c[6], c[7], c[35],path_yaw)
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', callback, 10)
    deadline = time.monotonic()+45.
    try:
        while not ready and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            refresh.tick(time.monotonic(), client.service_is_ready(), lambda:client.call_async(Empty.Request()))
        record = {'ready':ready,'reason':gate.reason,'source':'AMCL only; no simulator truth'}
        args.output.write_text(json.dumps(record,indent=2)+'\n')
        if not ready:
            raise RuntimeError('Localization readiness timed out: '+gate.reason)
        print('Localization readiness passed; controller guards remain active.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
