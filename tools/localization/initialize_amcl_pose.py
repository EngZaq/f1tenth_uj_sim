#!/usr/bin/env python3
"""Initialize simulation AMCL from scan matching with explicit uncertainty.

Publishes one /initialpose message, never /drive or TF. Parameter-only pose
initialization in Nav2 Humble constructs a message with zero covariance.
This helper uses a nonzero experimental prior, not a calibrated confidence
estimate derived from the scan-match score. Driving remains separately gated.
"""
import argparse
import json
import math
import time
from pathlib import Path


def pose_payload(seed, position_std_m=0.10, yaw_std_deg=5.0):
    """Return a ROS-message-shaped payload; covariance contains variances."""
    x, y, heading = (float(seed[k]) for k in ('x', 'y', 'yaw'))
    if not all(math.isfinite(v) for v in (x, y, heading)):
        raise ValueError('Scan-matched x, y and yaw must be finite')
    if not all(math.isfinite(v) and v > 0 for v in
               (position_std_m, yaw_std_deg)):
        raise ValueError('Initial standard deviations must be finite and positive')
    heading = math.atan2(math.sin(heading), math.cos(heading))
    position_variance = position_std_m*position_std_m
    yaw_sigma = math.radians(yaw_std_deg)
    yaw_variance = yaw_sigma*yaw_sigma
    if not all(math.isfinite(v) and v > 0 for v in
               (position_variance, yaw_variance)):
        raise ValueError('Initial variances must remain finite and strictly positive')
    covariance = [0.0]*36
    covariance[0] = covariance[7] = position_variance
    covariance[35] = yaw_variance
    return {
        'header': {'frame_id': 'slam_map'},
        'pose': {
            'pose': {'position': {'x': x, 'y': y, 'z': 0.0},
                     'orientation': {'x': 0.0, 'y': 0.0,
                                     'z': math.sin(heading/2),
                                     'w': math.cos(heading/2)}},
            'covariance': covariance}}


def stamp_s(message):
    return message.header.stamp.sec+message.header.stamp.nanosec*1e-9


def publish_and_confirm(node, publisher, message, observations, timeout,
                        spin_once, clock=time.monotonic):
    """Wait for a subscriber, publish ONCE, then require a fresh AMCL reply.

    A reply acknowledges processing; it does not establish pose correctness.
    Missing acknowledgement is an error, never a zero-covariance fallback.
    """
    deadline = clock()+timeout
    while publisher.get_subscription_count() == 0:
        if clock() >= deadline:
            raise RuntimeError('No AMCL /initialpose subscriber; initialization aborted')
        spin_once(node, timeout_sec=0.1)
    message.header.stamp = node.get_clock().now().to_msg()
    sent_stamp = stamp_s(message)
    publisher.publish(message)
    deadline = clock()+timeout
    while clock() < deadline:
        spin_once(node, timeout_sec=0.1)
        for reply in observations:
            if (reply.header.frame_id == message.header.frame_id and
                    stamp_s(reply) >= sent_stamp):
                covariance = reply.pose.covariance
                values = [reply.pose.pose.position.x, reply.pose.pose.position.y]
                if (all(math.isfinite(v) for v in values) and
                        all(math.isfinite(covariance[i]) and covariance[i] >= 0
                            for i in (0, 7, 35))):
                    return reply
    raise RuntimeError('No fresh AMCL reply after /initialpose; initialization unconfirmed')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed-json', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--position-std-m', type=float, default=0.10)
    parser.add_argument('--yaw-std-deg', type=float, default=5.0)
    parser.add_argument('--timeout', type=float, default=10.0)
    args = parser.parse_args()
    try:
        payload = pose_payload(json.loads(args.seed_json.read_text()),
                               args.position_std_m, args.yaw_std_deg)
        if not math.isfinite(args.timeout) or args.timeout <= 0:
            raise ValueError('Timeout must be finite and positive')
    except (ValueError, KeyError, TypeError, OSError) as exc:
        parser.error(str(exc))

    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.node import Node

    rclpy.init(args=[])
    node = Node('scan_seed_amcl_initializer')
    try:
        observations = []
        node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 observations.append, 10)
        publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        message = PoseWithCovarianceStamped()
        message.header.frame_id = payload['header']['frame_id']
        for key, value in payload['pose']['pose']['position'].items():
            setattr(message.pose.pose.position, key, value)
        for key, value in payload['pose']['pose']['orientation'].items():
            setattr(message.pose.pose.orientation, key, value)
        message.pose.covariance = payload['pose']['covariance']
        reply = publish_and_confirm(node, publisher, message, observations,
                                    args.timeout, rclpy.spin_once)
        record = {'initial_pose_message': payload,
                  'sent_stamp_s': stamp_s(message),
                  'amcl_reply_stamp_s': stamp_s(reply),
                  'position_std_m': args.position_std_m,
                  'yaw_std_deg': args.yaw_std_deg,
                  'status': 'fresh AMCL reply; accuracy not independently verified'}
        args.output.write_text(json.dumps(record, indent=2)+'\n')
        print(f'AMCL seed acknowledged: sigma_xy={args.position_std_m:.3f}m, '
              f'sigma_yaw={args.yaw_std_deg:.1f}deg; no motion published.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
