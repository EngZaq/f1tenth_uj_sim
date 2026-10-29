#!/usr/bin/env python3
"""Launch isolated Nav2 map_server + AMCL particle filter with lifecycle manager."""
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ws = Path.home() / 'f1tenth_sim'
    params = str(ws / 'config' / 'fusion_amcl.yaml')
    map_yaml = ws / 'results' / 'mapping' / 'uj_technical_slam_01.yaml'
    if not map_yaml.is_file():
        raise FileNotFoundError(f'Expected saved SLAM map YAML: {map_yaml}')
    if not (map_yaml.parent / 'uj_technical_slam_01.pgm').is_file():
        raise FileNotFoundError('Saved map PGM missing beside its YAML')

    return LaunchDescription([
        Node(
            package='nav2_map_server', executable='map_server', name='fusion_map_server',
            output='screen', parameters=[params, {'yaml_filename': str(map_yaml)}]),
        Node(
            package='nav2_amcl', executable='amcl', name='amcl',
            output='screen', parameters=[params]),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='fusion_localization_lifecycle_manager', output='screen',
            parameters=[{'use_sim_time': False, 'autostart': True,
                         'node_names': ['fusion_map_server', 'amcl']}]),
    ])
