"""VISUALIZATION ONLY: draw the sim car model at the ESTIMATED pose (estimation tree).

The truth model (ego_racecar/*) is attached to `map` by gym_bridge. This launch draws the
same URDF a second time, with the link names fusion_model/*, attached under fusion_base:

    slam_map -> fusion_odom -> fusion_base -> fusion_model/base_link (+0.17145 m in x) -> meshes

fusion_base is the rear axle. The URDF origin (base_link) is the CoG, lr = 0.17145 m ahead of it
(f1tenth_gym_ros/urdf/racecar_mesh.xacro, config/sim_laser_geometry.yaml).

It adds a leaf frame only. It does not publish fusion_base or anything in the truth tree, and it
uses its own topics (/fusion_model_description, /fusion_model/joint_states).

Start on top of a running sim and estimation stack:
    zsh -c 'source scripts/sim_env.zsh && ros2 launch tools/visualization/moatasem_fusion_car_model.launch.py'
RViz: RobotModel display, Description Topic = /fusion_model_description.

Arguments (defaults = original behaviour):
    accent_color:='1.0 0.45 0.0 1.0'
    tf_topic:=/tf  tf_static_topic:=/tf_static   (set to /fusion/tf, /fusion/tf_static when the estimation
                                                  tree runs on the separate fusion TF topics)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

REAR_AXLE_TO_COG_X = '0.17145'


def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory('f1tenth_gym_ros'),
                              'urdf', 'racecar_mesh.xacro')
    description = ParameterValue(Command([
        'xacro ', xacro_file,
        ' car_name:=fusion_model',
        ' accent_color:="', LaunchConfiguration('accent_color'), '"',
        ' vehicle:=f1tenth',
    ]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('accent_color', default_value='1.0 0.45 0.0 1.0'),
        DeclareLaunchArgument('tf_topic', default_value='/tf'),
        DeclareLaunchArgument('tf_static_topic', default_value='/tf_static'),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='fusion_model_state_publisher',
            parameters=[{'robot_description': description, 'use_sim_time': False}],
            remappings=[('/robot_description', '/fusion_model_description'),
                        ('/tf', LaunchConfiguration('tf_topic')),
                        ('/tf_static', LaunchConfiguration('tf_static_topic')),
                        ('/joint_states', '/fusion_model/joint_states')]),
        # Zero angles for the 4 continuous wheel joints (gym_bridge animates only the truth car).
        Node(
            package='joint_state_publisher', executable='joint_state_publisher',
            name='fusion_model_joint_state_publisher',
            parameters=[{'robot_description': description, 'use_sim_time': False}],
            remappings=[('/robot_description', '/fusion_model_description'),
                        ('/tf', LaunchConfiguration('tf_topic')),
                        ('/tf_static', LaunchConfiguration('tf_static_topic')),
                        ('/joint_states', '/fusion_model/joint_states')]),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='fusion_model_base_tf',
            arguments=['--x', REAR_AXLE_TO_COG_X, '--y', '0', '--z', '0',
                       '--frame-id', 'fusion_base',
                       '--child-frame-id', 'fusion_model/base_link'],
            remappings=[('/tf', LaunchConfiguration('tf_topic')),
                        ('/tf_static', LaunchConfiguration('tf_static_topic'))]),
    ])
