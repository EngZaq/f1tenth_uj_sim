#!/usr/bin/env python3
"""Use the map and AMCL init pose generated during the same exploration mission."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params'),
        DeclareLaunchArgument('map_yaml'),
        Node(package='nav2_map_server',executable='map_server',
             name='fusion_map_server',output='screen',
             parameters=[LaunchConfiguration('params'),
                         {'yaml_filename':LaunchConfiguration('map_yaml')}]),
        Node(package='nav2_amcl',executable='amcl',name='amcl',
             output='screen',parameters=[LaunchConfiguration('params')],
             # Gym listens on global /initialpose to teleport the car.
             # AMCL's saved-map hypothesis must not reset simulator truth.
             remappings=[('initialpose','/fusion/initialpose')]),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
             name='fusion_localization_lifecycle_manager',output='screen',
             parameters=[{'use_sim_time':False,'autostart':True,
                          'node_names':['fusion_map_server','amcl']}]),
    ])
