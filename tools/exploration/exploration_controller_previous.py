#!/usr/bin/env python3
"""UJ F1TENTH exploration candidate. One algorithm for sim and Jetson.

IMPORTANT: Physical mode only outputs proposals. Actuation must be handled by an
independently tested safety arbiter with dead-man, timeout and E-stop.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from ackermann_msgs.msg import AckermannDriveStamped


def clamp(x, a, b):
    return max(a, min(b, x))


def points_and_coverage(msg, detection_limit):
    rr = np.asarray(msg.ranges, dtype=float)
    theta = msg.angle_min + np.arange(len(rr)) * msg.angle_increment
    finite = np.isfinite(rr) & (rr > max(0.05, msg.range_min)) & (rr <= msg.range_max)
    # +inf is no-return; no guaranteed obstacle-free extension is assumed.
    points = np.column_stack((rr[finite] * np.cos(theta[finite]), rr[finite] * np.sin(theta[finite])))
    keep = np.linalg.norm(points, axis=1) < detection_limit
    return theta, rr, finite, points[keep]


def pose_at(s, steer, wheelbase):
    k = math.tan(steer) / wheelbase
    if abs(k) < 1e-8:
        return s, 0.0, 0.0
    a = s * k
    return math.sin(a)/k, (1 - math.cos(a))/k, a


def arc_footprint_clearance(points_laser, steer, p):
    """Min signed separation in rear-axle coordinates; negative = overlap.

    Geometry: rear axle at (0,0); laser at (laser_x, laser_y).
    Body spans x=[-rear_overhang, wheelbase+front_overhang].
    """
    if not len(points_laser):
        return False, -float('inf')
    xobs = points_laser[:,0] + p['laser_x']
    yobs = points_laser[:,1] + p['laser_y']
    half = p['width']/2.0 + p['margin']
    rear = -p['rear_overhang'] - p['margin']
    front = p['wheelbase'] + p['front_overhang'] + p['margin']
    best = float('inf')
    samples = max(12, int(p['horizon']/0.04)+1)
    for s in np.linspace(0.0, p['horizon'], samples):
        x,y,heading = pose_at(s, steer, p['wheelbase'])
        dx, dy = xobs-x, yobs-y
        ch,sh = math.cos(heading),math.sin(heading)
        longitudinal = ch*dx + sh*dy
        lateral = -sh*dx + ch*dy
        ex = np.maximum(np.maximum(rear-longitudinal, longitudinal-front), 0.)
        ey = np.maximum(np.abs(lateral)-half, 0.)
        separation = np.hypot(ex,ey)
        d = float(np.min(separation))
        best = min(best,d)
        if d < 0.005:
            return False, best
    return True, best


def plan(msg, previous, p):
    angle, rr, valid, obstacles = points_and_coverage(msg, p['detection_limit'])
    forward = np.abs(angle) <= math.radians(20)
    if np.count_nonzero(forward) < 5:
        return 0.,0.,'no_forward_beams'
    # A finite hit is evidence of an obstacle, +inf is *unknown*, not an obstacle.
    # Reject malformed values separately. Require enough non-NaN/nonnegative beams.
    okay = valid | np.isposinf(rr)
    if np.mean(okay[forward]) < p['forward_coverage']:
        return 0.,0.,'bad_forward_scan'
    fw = rr[forward & valid]
    if len(fw) >= 5 and np.percentile(fw, 10) < p['hard_stop']:
        return 0.,0.,'front_obstacle'
    # Evidence of distant walls is useful for a direction preference, but unknown
    # beams are NOT converted to obstacle-free distances for collision checks.
    # Evaluate a local candidate bundle and prefer progress while retaining turn bias.
    best = None
    nclear = 0
    for steer in np.linspace(-p['max_steer'],p['max_steer'],41):
        steer=float(steer)
        safe, space = arc_footprint_clearance(obstacles,steer,p)
        if not safe:
            continue
        nclear += 1
        _,_,yaw = pose_at(p['horizon']*0.75,steer,p['wheelbase'])
        heading = yaw*0.5
        diff = np.arctan2(np.sin(angle-heading),np.cos(angle-heading))
        sector = np.abs(diff) < math.radians(12)
        measured = rr[sector & valid]
        if len(measured) < 3:
            # No finite directional evidence. Keep the command stopped rather than
            # encouraging navigation into entirely unobserved space.
            continue
        confidence=np.mean(okay[sector]) if np.count_nonzero(sector) else 0.
        if confidence < p['direction_coverage']:
            continue
        evidence=float(np.percentile(measured,20))
        if evidence < p['hard_stop']+0.10:
            continue
        # Forward progress + steering stability; safe arcs are checked above.
        score=(1.2*min(evidence,2.5)/2.5 + 0.20*min(space,0.40)/0.40
               - 0.12*abs(steer/p['max_steer'])
               - 0.20*abs(steer-previous)/p['max_steer'])
        if best is None or score > best[0]:
            best=(score,steer,evidence)
    if best is None:
        return 0.,0.,'blocked' if nclear==0 else 'insufficient_direction_evidence'
    desired=best[1]
    steer=clamp(desired, previous-p['steer_step'], previous+p['steer_step'])
    # Re-check the actual command after limiting; never assume interpolated arc is clear.
    safe,_=arc_footprint_clearance(obstacles,steer,p)
    if not safe:
        return 0.,0.,'limited_steer_not_clear'
    speed=p['speed']*max(0.65,1.0-0.45*abs(steer)/p['max_steer'])
    if best[2] < 1.0:
        speed=min(speed,0.09)
    return float(steer),float(speed),'ok'


class Explorer(Node):
    def __init__(self):
        super().__init__('uj_explorer_release')
        defaults={'platform':'physical','enable_drive':False,'scan_topic':'/scan',
                  'proposal_topic':'/exploration/drive','speed':0.14,
                  'wheelbase':0.26,'width':0.203,'rear_overhang':0.04,
                  'front_overhang':0.04,'laser_x':0.21,'laser_y':0.0,
                  'max_steer':0.40,'margin':0.08,'horizon':0.75,
                  'detection_limit':3.0,'hard_stop':0.55,
                  'forward_coverage':0.70,'direction_coverage':0.65,
                  'steer_step':0.045,'scan_timeout':0.25,'recovery_scans':3}
        for k,v in defaults.items():
            self.declare_parameter(k,v)
        self.p={k:self.get_parameter(k).value for k in defaults}
        self.platform=str(self.p['platform'])
        if self.platform not in ('physical','sim'):
            raise ValueError('platform must be sim or physical')
        if self.platform=='physical' and self.p['enable_drive']:
            raise ValueError('Physical actuation not allowed. Use a validated external safety arbiter.')
        # Reject malformed geometry before any command publication.
        for k in ['wheelbase','width','max_steer','horizon','detection_limit','hard_stop','scan_timeout']:
            if float(self.p[k])<=0:
                raise ValueError(k+' must be > 0')
        self.proposed=self.create_publisher(AckermannDriveStamped,str(self.p['proposal_topic']),10)
        self.cmd=(self.create_publisher(AckermannDriveStamped,'/drive',10)
                  if self.platform=='sim' and self.p['enable_drive'] else None)
        self.diag=self.create_publisher(String,'/exploration/status',10)
        self.scan=None
        self.received=None
        self.collided=False
        self.steer=0.0
        self.valid_streak=0
        self.last_reason=None
        self.create_subscription(LaserScan,str(self.p['scan_topic']),self.on_scan,qos_profile_sensor_data)
        if self.platform=='sim':
            self.create_subscription(Bool,'/ego_racecar/collision',self.on_collision,10)
        self.create_timer(0.05,self.tick)
        self.get_logger().info(f'Explorer ready: {self.platform}, actuation={self.cmd is not None}')

    def on_scan(self,msg):
        self.scan=msg
        self.received=self.get_clock().now()

    def on_collision(self,msg):
        self.collided=bool(msg.data)

    def tick(self):
        now=self.get_clock().now()
        steer,speed,reason=0.,0.,'waiting_for_scan'
        if self.collided:
            reason='collision'
        elif self.scan is not None:
            age=(now-self.received).nanoseconds/1e9
            if 0<=age<=self.p['scan_timeout']:
                steer,speed,reason=plan(self.scan,self.steer,self.p)
            else:
                reason='stale_scan'
        if reason=='ok':
            self.valid_streak+=1
            self.steer=steer
            if self.valid_streak < int(self.p['recovery_scans']):
                speed=0.
                reason='waiting_stable_clearance'
        else:
            self.valid_streak=0
            # Retain remembered steering through temporary stops.
        cmd=AckermannDriveStamped()
        cmd.header.stamp=now.to_msg()
        cmd.drive.steering_angle=float(steer if reason in ('ok','waiting_stable_clearance') else 0.)
        cmd.drive.speed=float(speed)
        self.proposed.publish(cmd)
        if self.cmd is not None:
            self.cmd.publish(cmd)
        self.diag.publish(String(data=reason))
        if reason!=self.last_reason:
            self.get_logger().info(f'{reason}: steer={cmd.drive.steering_angle:+.3f}, speed={speed:.2f}')
            self.last_reason=reason


def main():
    rclpy.init()
    node=Explorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=='__main__':
    main()
