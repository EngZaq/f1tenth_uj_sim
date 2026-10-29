#!/usr/bin/env python3
"""AMCL scan-pose versus map->odom->base TF shadow steering, ROS 2 Humble.

Only publishes diagnostic /drive_shadow_raw and /drive_shadow_tf. Never /drive.
/drive is observed for command comparison; no ground-truth pose is used.
"""
import csv
import math
import os
import statistics
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def segdist(px,py,ax,ay,bx,by):
    dx,dy=bx-ax,by-ay
    L=dx*dx+dy*dy
    if L<1e-12:
        return math.hypot(px-ax,py-ay)
    a=max(0.,min(1.,((px-ax)*dx+(py-ay)*dy)/L))
    return math.hypot(px-ax-a*dx,py-ay-a*dy)


def read_path(filename):
    path=[]
    with open(os.path.expanduser(filename)) as f:
        for line in f:
            line=line.strip()
            if line and not line.startswith('#'):
                v=[float(x) for x in line.split(';')]
                path.append((v[0],v[1],v[2]))
    if len(path)<3 or any(path[i][0]>path[i+1][0] for i in range(len(path)-1)):
        raise ValueError('Bad path: need increasing s;x;y rows')
    return path


def read_speeds(filename):
    with open(os.path.expanduser(filename), newline='') as f:
        return [float(r['planned_speed_mps']) for r in csv.DictReader(f)]


def describe(values):
    if not values:
        return 'n/a'
    arr=sorted(values)
    p95=arr[min(len(arr)-1,round(.95*(len(arr)-1)))]
    return f'mean={statistics.mean(arr):.2f}deg p95={p95:.2f}deg max={max(arr):.2f}deg'


class TFShadowCompare(Node):
    def __init__(self):
        super().__init__('amcl_tf_shadow_compare')
        self.declare_parameter('waypoints_file', '~/f1tenth_sim/reference/generated/uj_technical_slam_01_centerline.csv')
        self.declare_parameter('speed_profile_file', '~/f1tenth_sim/reference/generated/uj_technical_slam_01_speed_profile.csv')
        self.declare_parameter('results_file', '~/f1tenth_sim/results/localization/amcl_tf_shadow_compare.csv')
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('speed_scale', 1.0)
        self.declare_parameter('max_raw_pose_age_s', 0.4)
        self.declare_parameter('max_tf_pose_age_s', 0.25)
        self.declare_parameter('max_path_cte_m', 0.5)
        self.declare_parameter('max_position_std_m', 0.6)
        self.declare_parameter('max_heading_std_deg', 30.0)
        self.declare_parameter('map_frame', 'slam_map')
        self.declare_parameter('base_frame', 'fusion_base')
        P=lambda key:self.get_parameter(key).value
        self.path=read_path(P('waypoints_file'))
        self.speeds=read_speeds(P('speed_profile_file'))
        if len(self.path)!=len(self.speeds):
            raise ValueError('Speed profile/path length mismatch')
        self.lookahead=float(P('lookahead_distance'))
        self.wheelbase=float(P('wheelbase'))
        self.max_steer=float(P('max_steering_angle'))
        self.speed_scale=float(P('speed_scale'))
        self.raw_age_limit=float(P('max_raw_pose_age_s'))
        self.tf_age_limit=float(P('max_tf_pose_age_s'))
        self.cte_limit=float(P('max_path_cte_m'))
        self.pos_std_limit=float(P('max_position_std_m'))
        self.yaw_std_limit=math.radians(float(P('max_heading_std_deg')))
        self.map_frame=str(P('map_frame'))
        self.base_frame=str(P('base_frame'))
        self.raw=None
        self.raw_recv=0.
        self.oracle=None
        self.oracle_recv=0.
        self.tf_buffer=Buffer()
        self.tf_listener=TransformListener(self.tf_buffer,self)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.on_amcl,10)
        self.create_subscription(AckermannDriveStamped,'/drive',self.on_oracle,10)
        self.pub_raw=self.create_publisher(AckermannDriveStamped,'/drive_shadow_raw',10)
        self.pub_tf=self.create_publisher(AckermannDriveStamped,'/drive_shadow_tf',10)
        self.csv_path=Path(os.path.expanduser(P('results_file')))
        self.csv_path.parent.mkdir(parents=True,exist_ok=True)
        self.csv_file=self.csv_path.open('w',newline='')
        self.writer=csv.writer(self.csv_file)
        self.writer.writerow(['ros_time_s','raw_age_s','tf_age_s','raw_x','raw_y','raw_yaw_deg',
                              'tf_x','tf_y','tf_yaw_deg','raw_cte_m','tf_cte_m',
                              'raw_steering_deg','tf_steering_deg','oracle_steering_deg',
                              'abs_raw_oracle_diff_deg','abs_tf_oracle_diff_deg','raw_ok','tf_ok'])
        self.raw_diffs=[]
        self.tf_diffs=[]
        self.samples=0
        self.tf_fail=0
        self.last_log=time.monotonic()
        self.create_timer(0.05,self.timer)
        self.get_logger().info('SHADOW-ONLY: /drive_shadow_raw and /drive_shadow_tf; never publishes /drive')
        self.get_logger().info(f'Compare raw /amcl_pose vs latest {self.map_frame}->{self.base_frame}; CSV: {self.csv_path}')

    def on_amcl(self,msg):
        if msg.header.frame_id==self.map_frame:
            self.raw=msg
            self.raw_recv=time.monotonic()

    def on_oracle(self,msg):
        self.oracle=msg.drive
        self.oracle_recv=time.monotonic()

    def control(self,x,y,h):
        n=len(self.path)
        nearest=min(range(n),key=lambda i:(self.path[i][1]-x)**2+(self.path[i][2]-y)**2)
        target_s=self.path[nearest][0]+self.lookahead
        target=next((j for j in range(nearest,n) if self.path[j][0]>=target_s),None)
        if target is None:
            remainder=target_s-self.path[-1][0]
            target=next((j for j in range(n) if self.path[j][0]>=remainder),0)
        tx,ty=self.path[target][1:3]
        dx,dy=tx-x,ty-y
        r2=dx*dx+dy*dy
        steering=0. if r2<1e-6 else math.atan(self.wheelbase*2*(-math.sin(h)*dx+math.cos(h)*dy)/r2)
        steering=max(-self.max_steer,min(self.max_steer,steering))
        cte=min(segdist(x,y,self.path[i][1],self.path[i][2],self.path[(i+1)%n][1],self.path[(i+1)%n][2]) for i in range(n))
        speed=max(0.,self.speeds[nearest]*self.speed_scale)
        return steering,speed,cte

    def send(self, publisher, steer, speed):
        msg=AckermannDriveStamped()
        msg.header.stamp=self.get_clock().now().to_msg()
        msg.header.frame_id=self.map_frame
        msg.drive.steering_angle=float(steer)
        msg.drive.speed=float(speed)
        publisher.publish(msg)

    def timer(self):
        if self.raw is None:
            return
        self.samples+=1
        now=time.monotonic()
        rosnow=self.get_clock().now().nanoseconds*1e-9
        stamp=self.raw.header.stamp.sec+self.raw.header.stamp.nanosec*1e-9
        raw_age=rosnow-stamp
        p=self.raw.pose.pose.position
        h=yaw(self.raw.pose.pose.orientation)
        raw_steer,raw_speed,raw_cte=self.control(p.x,p.y,h)
        cov=self.raw.pose.covariance
        quality=(math.sqrt(max(0.,cov[0],cov[7]))<=self.pos_std_limit and
                 math.sqrt(max(0.,cov[35]))<=self.yaw_std_limit)
        raw_ok=(quality and math.isfinite(raw_steer) and raw_cte<=self.cte_limit and
                -.15<=raw_age<=self.raw_age_limit and now-self.raw_recv<=self.raw_age_limit)
        self.send(self.pub_raw,raw_steer if raw_ok else 0.,raw_speed if raw_ok else 0.)
        tx=ty=th=tf_age=tf_steer=tf_cte=float('nan')
        tf_ok=False
        try:
            tr=self.tf_buffer.lookup_transform(self.map_frame,self.base_frame,Time())
            t=tr.transform.translation
            tx,ty=float(t.x),float(t.y)
            th=yaw(tr.transform.rotation)
            ts=tr.header.stamp.sec+tr.header.stamp.nanosec*1e-9
            tf_age=rosnow-ts
            tf_steer,tf_speed,tf_cte=self.control(tx,ty,th)
            tf_ok=(quality and math.isfinite(tf_steer) and tf_cte<=self.cte_limit and
                   -.15<=tf_age<=self.tf_age_limit and
                   now-self.raw_recv<=self.raw_age_limit)
            self.send(self.pub_tf,tf_steer if tf_ok else 0.,tf_speed if tf_ok else 0.)
        except TransformException:
            self.tf_fail+=1
            self.send(self.pub_tf,0.,0.)
        oracle_ok=(self.oracle is not None and now-self.oracle_recv<=.2 and self.oracle.speed>.1)
        oracle_deg=math.degrees(self.oracle.steering_angle) if oracle_ok else float('nan')
        raw_diff=(abs(math.degrees(raw_steer)-oracle_deg) if raw_ok and oracle_ok else float('nan'))
        tf_diff=(abs(math.degrees(tf_steer)-oracle_deg) if tf_ok and oracle_ok else float('nan'))
        # Only include exactly matched samples in aggregate A/B comparison.
        if math.isfinite(raw_diff) and math.isfinite(tf_diff):
            self.raw_diffs.append(raw_diff)
            self.tf_diffs.append(tf_diff)
        self.writer.writerow([f'{rosnow:.3f}',f'{raw_age:.3f}',f'{tf_age:.3f}',
                              f'{p.x:.4f}',f'{p.y:.4f}',f'{math.degrees(h):.3f}',
                              f'{tx:.4f}',f'{ty:.4f}',f'{math.degrees(th):.3f}',
                              f'{raw_cte:.4f}',f'{tf_cte:.4f}',f'{math.degrees(raw_steer):.3f}',
                              f'{math.degrees(tf_steer):.3f}',f'{oracle_deg:.3f}',
                              f'{raw_diff:.3f}',f'{tf_diff:.3f}',int(raw_ok),int(tf_ok)])
        if now-self.last_log>=5.:
            self.csv_file.flush()
            self.get_logger().info(f'A/B paired={len(self.raw_diffs)} raw_diff={describe(self.raw_diffs)} | '
                                   f'TF_diff={describe(self.tf_diffs)} | raw_age={raw_age:.2f}s TF_age={tf_age:.2f}s '
                                   f'TF_lookup_failures={self.tf_fail}')
            self.last_log=now

    def close(self):
        self.csv_file.flush()
        self.csv_file.close()
        self.get_logger().info('=== SHADOW A/B SUMMARY ===')
        self.get_logger().info(f'Paired moving samples: {len(self.raw_diffs)}')
        self.get_logger().info(f'Raw AMCL -> Oracle: {describe(self.raw_diffs)}')
        self.get_logger().info(f'TF-fused  -> Oracle: {describe(self.tf_diffs)}')
        self.get_logger().info(f'TF lookup failures: {self.tf_fail} / {self.samples} total timer calls')
        if self.raw_diffs and self.tf_diffs:
            delta=statistics.mean(self.raw_diffs)-statistics.mean(self.tf_diffs)
            self.get_logger().info(f'Mean steering disagreement change: {delta:+.2f}deg (positive means TF improved)')
        self.get_logger().info(f'CSV saved: {self.csv_path}')


def main():
    rclpy.init()
    node=TFShadowCompare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    main()
