#!/usr/bin/env python3
"""Dedicated mapping TF chain; never launch AMCL concurrently with this node.

Optional: slam_config:=<params file> (default config/slam_sim.yaml, loop closing off)."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default=str(Path(__file__).resolve().parents[2]/'config/slam_sim.yaml')
    config=LaunchConfiguration('slam_config')
    return LaunchDescription([DeclareLaunchArgument('slam_config',default_value=default),Node(
        package='slam_toolbox',executable='async_slam_toolbox_node',
        name='slam_toolbox',output='screen',
        parameters=[config,{'use_sim_time':False,
                            'odom_frame':'fusion_odom',
                            'base_frame':'fusion_base',
                            'map_frame':'slam_map',
                            'scan_topic':'/fusion/scan',
                            'map_update_interval':3.0}],
        remappings=[('map','/mission/slam_map'),
                    ('map_metadata','/mission/slam_map_metadata')])])
