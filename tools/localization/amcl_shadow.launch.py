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
    scan_min_period_s:=0.1                                 (vesc / ekf2 relay spacing: 0.1 = original ~8.4 Hz,
                                                            0.0 = every /scan, 30 Hz)
    tf_topic:=/tf  tf_static_topic:=/tf_static             (ekf2 only: topics the EKF and AMCL use for TF. Set to
                                                            /fusion/tf and /fusion/tf_static to keep the estimation
                                                            tree off the simulator's ~1450 msg/s /tf; the launch then
                                                            also publishes fusion_base -> fusion_laser there. The
                                                            relay still publishes it on /tf_static as well.)
    amcl_pose_topic:=/amcl_pose  amcl_tf_broadcast:=true    (Test 7 shadow runs: AMCL publishes its pose elsewhere and
                                                            no map->odom TF, while another node owns map->odom)

TF ownership (exactly one publisher per edge):
    slam_map    -> fusion_odom  : AMCL
    fusion_odom -> fusion_base  : EKF (ekf, ekf2) or sim_vesc_to_odom (vesc)
    fusion_base -> fusion_laser : fusion_scan_relay (static)
AMCL's initial-pose topic is remapped to /fusion/initialpose. The global /initialpose
teleports the Gym car and must never be used for the localization hypothesis.
"""
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
from sim_geometry import FUSION_LASER  # noqa: E402  same values as fusion_scan_relay.py


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
        DeclareLaunchArgument('scan_min_period_s', default_value='0.1'),
        DeclareLaunchArgument('tf_topic', default_value='/tf'),
        DeclareLaunchArgument('tf_static_topic', default_value='/tf_static'),
        DeclareLaunchArgument('amcl_pose_topic', default_value='/amcl_pose'),
        DeclareLaunchArgument('amcl_tf_broadcast', default_value='true'),

        # ---------------- localization: map + AMCL ----------------
        Node(package='nav2_map_server', executable='map_server', name='fusion_map_server',
             output='screen', parameters=[params, {'yaml_filename': map_yaml}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[params, {'tf_broadcast': ParameterValue(LaunchConfiguration('amcl_tf_broadcast'), value_type=bool)}],
             remappings=[('initialpose', '/fusion/initialpose'),
                         ('amcl_pose', LaunchConfiguration('amcl_pose_topic')),
                         ('/tf', LaunchConfiguration('tf_topic')),
                         ('/tf_static', LaunchConfiguration('tf_static_topic'))]),
        # fusion_base -> fusion_laser on a separate static TF topic (only when tf_static_topic is changed)
        Node(package='tf2_ros', executable='static_transform_publisher', name='fusion_laser_static_tf',
             arguments=['--x', str(FUSION_LASER[0]), '--y', str(FUSION_LASER[1]), '--z', '0',
                        '--yaw', str(FUSION_LASER[2]),
                        '--frame-id', 'fusion_base', '--child-frame-id', 'fusion_laser'],
             remappings=[('/tf_static', LaunchConfiguration('tf_static_topic'))],
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('tf_static_topic'), "' != '/tf_static'"]))),
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
            cmd=['python3', '-u', str(ROOT / 'tools/localization/fusion_scan_relay.py'),
                 '--ros-args', '-p', ['min_period_s:=', LaunchConfiguration('scan_min_period_s')]],
            output='screen', condition=own_sensors),

        # ---------------- odometry source: EKF with VESC + IMU only ----------------
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=[ekf_yaml], output='screen', condition=ekf2,
             remappings=[('/tf', LaunchConfiguration('tf_topic')),
                         ('/tf_static', LaunchConfiguration('tf_static_topic'))]),

        # ---------------- odometry source A: raw VESC odometry ----------------
        ExecuteProcess(
            name='sim_vesc_to_odom',
            cmd=['python3', '-u', str(ROOT / 'tools/localization/sim_vesc_to_odom.py'),
                 '--ros-args',
                 '-p', ['steer_bias_deg:=', LaunchConfiguration('steer_bias_deg')],
                 '-p', ['steer_noise_deg:=', LaunchConfiguration('steer_noise_deg')]],
            output='screen', condition=vesc),
    ])
