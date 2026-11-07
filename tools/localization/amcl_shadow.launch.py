"""AMCL shadow test: map_server + AMCL on the saved SLAM map, fed by ONE odometry source.

Localization is graded only; the car is still driven by the simulator truth (Pure Pursuit),
so this launch never publishes /drive. Start the simulator first, then:

    zsh -c 'source scripts/sim_env.zsh && ros2 launch tools/localization/amcl_shadow.launch.py odom_source:=ekf2'

Arguments:
    odom_source:=ekf2 | ekf | vesc   (default ekf2 = main EKF)
                                ekf  = three-sensor EKF (tools/fusion/fusion_three_sensors.launch.py)
                                ekf2 = MAIN EKF (no rf2o, no cov_relay): sim VESC+IMU, scan relay and
                                       ekf_node with the yaml given by ekf:= (must not list odom1)
                                vesc = raw VESC odometry (tools/localization/sim_vesc_to_odom.py):
                                       measured wheel speed + COMMANDED steering, no EKF/rf2o
    map_yaml:=results/missions/autonomous_20261103_114924_818822/new_track.yaml
                                (no-loop map; its slam_map origin = the car's start pose, so the
                                 initial pose is ~(0, 0, 0). The start/finish area is mapped twice
                                 (lap never closed, ~0.85 m / 15 deg apart): seed the START copy.)
    params:=config/fusion_amcl_moatasem.yaml               (team AMCL parameters, unchanged)
    sensors:=config/fusion_sim_sensors_calibrated.yaml     (sim VESC + IMU, both sources)
    steer_bias_deg:=0.0  steer_noise_deg:=0.0               (vesc only: servo / linkage play)
    ekf:=config/fusion_ekf_moatasem.yaml                   (EKF parameters for ekf2, two sensors)
    ekf_rf2o:=config/fusion_ekf_moatasem_rf2o.yaml         (EKF parameters for ekf, three sensors)

TF ownership (exactly one publisher per edge):
    slam_map    -> fusion_odom  : AMCL
    fusion_odom -> fusion_base  : EKF (ekf, ekf2) or sim_vesc_to_odom (vesc)
    fusion_base -> fusion_laser : fusion_scan_relay (static)
AMCL's initial-pose topic is remapped to /fusion/initialpose. The global /initialpose
teleports the Gym car and must never be used for the localization hypothesis.
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[2]


def generate_launch_description():
    params = LaunchConfiguration('params')
    map_yaml = LaunchConfiguration('map_yaml')
    sensors = LaunchConfiguration('sensors')
    vesc = LaunchConfigurationEquals('odom_source', 'vesc')
    ekf = LaunchConfigurationEquals('odom_source', 'ekf')
    ekf2 = LaunchConfigurationEquals('odom_source', 'ekf2')
    ekf_yaml = LaunchConfiguration('ekf')
    # sensor emulator + scan relay started here unless the three-sensor launch provides them
    own_sensors = IfCondition(PythonExpression(["'", LaunchConfiguration('odom_source'), "' in ('vesc', 'ekf2')"]))

    return LaunchDescription([
        DeclareLaunchArgument('odom_source', default_value='ekf2', choices=['ekf', 'ekf2', 'vesc']),
        DeclareLaunchArgument('map_yaml',
                              default_value=str(ROOT / 'results/missions/autonomous_20261103_114924_818822/new_track.yaml')),
        DeclareLaunchArgument('params', default_value=str(ROOT / 'config/fusion_amcl_moatasem.yaml')),
        DeclareLaunchArgument('sensors',
                              default_value=str(ROOT / 'config/fusion_sim_sensors_calibrated.yaml')),
        DeclareLaunchArgument('steer_bias_deg', default_value='0.0'),
        DeclareLaunchArgument('steer_noise_deg', default_value='0.0'),
        DeclareLaunchArgument('ekf', default_value=str(ROOT / 'config/fusion_ekf_moatasem.yaml')),
        DeclareLaunchArgument('ekf_rf2o', default_value=str(ROOT / 'config/fusion_ekf_moatasem_rf2o.yaml')),

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
            launch_arguments={'sensors': sensors, 'ekf': LaunchConfiguration('ekf_rf2o')}.items(),
            condition=ekf),

        # ---------------- sensors for vesc / ekf2 (ekf gets them from its own launch) ----------------
        ExecuteProcess(
            name='sim_vesc_imu',
            cmd=['python3', '-u', str(ROOT / 'tools/fusion/sim_vesc_imu.py'),
                 '--ros-args', '--params-file', sensors],
            output='screen', condition=own_sensors),
        ExecuteProcess(
            name='fusion_scan_relay',
            cmd=['python3', '-u', str(ROOT / 'tools/localization/fusion_scan_relay.py')],
            output='screen', condition=own_sensors),

        # ---------------- odometry source: EKF with VESC + IMU only ----------------
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=[ekf_yaml], output='screen', condition=ekf2),

        # ---------------- odometry source A: raw VESC odometry ----------------
        ExecuteProcess(
            name='sim_vesc_to_odom',
            cmd=['python3', '-u', str(ROOT / 'tools/localization/sim_vesc_to_odom.py'),
                 '--ros-args',
                 '-p', ['steer_bias_deg:=', LaunchConfiguration('steer_bias_deg')],
                 '-p', ['steer_noise_deg:=', LaunchConfiguration('steer_noise_deg')]],
            output='screen', condition=vesc),
    ])
