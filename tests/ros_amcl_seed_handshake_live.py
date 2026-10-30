#!/usr/bin/env python3
"""Real ROS regression: no initial reply; acknowledge only a no-motion update.

The fixture deliberately emits no pose on receiving the seed. It represents
the stationary first-reply loss seen in CI. No simulator or drive is involved.
"""
import math
import sys
import time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/localization'))
from initialize_amcl_pose import publish_and_confirm, INITIAL_POSE_TOPIC
from stationary_localization_probe import RefreshSchedule


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from std_srvs.srv import Empty
    rclpy.init(args=[])
    fixture=Node('stationary_amcl_handshake_fixture')
    helper=Node('one_shot_amcl_initializer_test')
    executor=SingleThreadedExecutor()
    executor.add_node(fixture);executor.add_node(helper)
    seeds,replies=[],[]
    pose_pub=fixture.create_publisher(PoseWithCovarianceStamped,'/amcl_pose',10)
    fixture.create_subscription(PoseWithCovarianceStamped,INITIAL_POSE_TOPIC,seeds.append,10)
    def update(request,response):
        if seeds:
            reply=PoseWithCovarianceStamped()
            reply.header.frame_id='slam_map'
            reply.header.stamp=fixture.get_clock().now().to_msg()
            reply.pose.pose.orientation.w=1.
            reply.pose.covariance[0]=reply.pose.covariance[7]=.01
            reply.pose.covariance[35]=math.radians(5)**2
            pose_pub.publish(reply)
        return response
    fixture.create_service(Empty,'/request_nomotion_update',update)
    helper.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',replies.append,10)
    seed_pub=helper.create_publisher(PoseWithCovarianceStamped,INITIAL_POSE_TOPIC,10)
    client=helper.create_client(Empty,'/request_nomotion_update')
    schedule=RefreshSchedule()
    def refresh():
        schedule.tick(time.monotonic(),client.service_is_ready(),lambda:client.call_async(Empty.Request()))
    seed=PoseWithCovarianceStamped()
    seed.header.frame_id='slam_map';seed.pose.pose.orientation.w=1.
    seed.pose.covariance[0]=seed.pose.covariance[7]=.01
    seed.pose.covariance[35]=math.radians(5)**2
    try:
        reply=publish_and_confirm(helper,seed_pub,seed,replies,10.,
            lambda node,timeout_sec:executor.spin_once(timeout_sec=timeout_sec),
            request_update=refresh)
        assert len(seeds)==1, f'Seed was received {len(seeds)} times'
        assert schedule.sent>=1, 'No no-motion update requested'
        assert reply.header.frame_id=='slam_map'
        assert helper.count_publishers('/drive')==0
        assert helper.count_publishers('/initialpose')==0
        print('PASS live stationary seed handshake: one seed, fresh service-driven reply, no drive/reset.')
    finally:
        executor.shutdown()
        helper.destroy_node();fixture.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':
    main()
