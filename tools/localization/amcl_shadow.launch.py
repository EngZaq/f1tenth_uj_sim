"""AMCL shadow test: map_server + AMCL on the saved SLAM map, fed by ONE odometry source.

Localization is graded only; the car is still driven by the simulator truth (Pure Pursuit),
so this launch never publishes /drive. Start the simulator first, then:

    zsh -c 'source scripts/sim_env.zsh && ros2 launch tools/localization/amcl_shadow.launch.py odom_source:=ekf'

Arguments:
    odom_source:=ekf | vesc     ekf  = three-sensor EKF (tools/fusion/fusion_three_sensors.launch.py)
                                vesc = raw VESC odometry (tools/localization/sim_vesc_to_odom.py):
                                       measured wheel speed + COMMANDED steering, no EKF/rf2o
    map_yaml:=results/missions/autonomous_20261103_114924_818822/new_track.yaml
                                (no-loop map; its slam_map origin = the car's start pose, so the
                                 initial pose is ~(0, 0, 0). The start/finish area is mapped twice
                                 (lap never closed, ~0.85 m / 15 deg apart): seed the START copy.)
    params:=config/fusion_amcl_moatasem.yaml               (team AMCL parameters, unchanged)
    sensors:=config/fusion_sim_sensors_calibrated.yaml     (sim VESC + IMU, both sources)
    steer_bias_deg:=0.0  steer_noise_deg:=0.0               (vesc only: servo / linkage play)

TF ownership (exactly one publisher per edge):
    slam_map    -> fusion_odom  : AMCL
    fusion_odom -> fusion_base  : EKF (ekf) or sim_vesc_to_odom (vesc)
    fusion_base -> fusion_laser : fusion_scan_relay (static)
AMCL's initial-pose topic is remapped to /fusion/initialpose. The global /initialpose
teleports the Gym car and must never be used for the localization hypothesis.
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[2]


def generate_launch_description():
    params = LaunchConfiguration('params')
    map_yaml = LaunchConfiguration('map_yaml')
    sensors = LaunchConfiguration('sensors')
    vesc = LaunchConfigurationEquals('odom_source', 'vesc')
    ekf = LaunchConfigurationEquals('odom_source', 'ekf')

    return LaunchDescription([
        DeclareLaunchArgument('odom_source', default_value='ekf', choices=['ekf', 'vesc']),
        DeclareLaunchArgument('map_yaml',
                              default_value=str(ROOT / 'results/missions/autonomous_20261103_114924_818822/new_track.yaml')),
        DeclareLaunchArgument('params', default_value=str(ROOT / 'config/fusion_amcl_moatasem.yaml')),
        DeclareLaunchArgument('sensors',
                              default_value=str(ROOT / 'config/fusion_sim_sensors_calibrated.yaml')),
        DeclareLaunchArgument('steer_bias_deg', default_value='0.0'),
        DeclareLaunchArgument('steer_noise_deg', default_value='0.0'),

        # ---------------- localization: map + AMCL ----------------
        Node(package='nav2_map_server', executable='map_server', name='fusion_map_server',
             output='screen', parameters=[params, {'yaml_filename': map_yaml}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[params],
             remappings=[('initialpose', '/fusion/initialpose')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='fusion_localization_lifecycle_manager', output='screen',
             parameters=[{'use_sim_time': False, 'autostart': True,
                          'node_names': ['fusion_map_server', 'amcl']}]),

        # ---------------- odometry source B: three-sensor EKF ----------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(ROOT / 'tools/fusion/fusion_three_sensors.launch.py')),
            launch_arguments={'sensors': sensors}.items(),
            condition=ekf),

        # ---------------- odometry source A: raw VESC odometry ----------------
        ExecuteProcess(
            name='sim_vesc_imu',
            cmd=['python3', '-u', str(ROOT / 'tools/fusion/sim_vesc_imu.py'),
                 '--ros-args', '--params-file', sensors],
            output='screen', condition=vesc),
        ExecuteProcess(
            name='fusion_scan_relay',
            cmd=['python3', '-u', str(ROOT / 'tools/localization/fusion_scan_relay.py')],
            output='screen', condition=vesc),
        ExecuteProcess(
            name='sim_vesc_to_odom',
            cmd=['python3', '-u', str(ROOT / 'tools/localization/sim_vesc_to_odom.py'),
                 '--ros-args',
                 '-p', ['steer_bias_deg:=', LaunchConfiguration('steer_bias_deg')],
                 '-p', ['steer_noise_deg:=', LaunchConfiguration('steer_noise_deg')]],
            output='screen', condition=vesc),
    ])
