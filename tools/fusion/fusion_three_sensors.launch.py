"""Three-sensor EKF in the simulation: VESC (vx) + IMU (vyaw) + rf2o (vx, vyaw).

Start the simulator first (simrun), then:
    zsh -c 'source scripts/sim_env.zsh && ros2 launch tools/fusion/fusion_three_sensors.launch.py'

Optional arguments:
    sensors:=config/fusion_sim_sensors.yaml      (default: config/fusion_sim_sensors_real.yaml)
    ekf:=config/fusion_ekf_moatasem.yaml
    relay:=config/cov_relay_rf2o.yaml
    rf2o:=config/rf2o_fusion.yaml                (rf2o parameter file)
    scan_min_period_s:=0.1                       (fusion_scan_relay spacing; 0.0 = every scan)

Starts:  sim_vesc_imu.py -> fake VESC + IMU from truth velocities (the only truth reader)
         fusion_scan_relay.py -> /fusion/scan + TF fusion_base -> fusion_laser
         rf2o (publish_tf false) -> /fusion/rf2o/odom
         cov_relay.py -> /fusion/rf2o/odom_cov (adds R)
         robot_localization ekf_node -> /odometry/filtered + TF fusion_odom -> fusion_base
Do NOT run this together with run_full_autonomy.zsh (it starts its own copies).
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[2]


def generate_launch_description():
    sensors = LaunchConfiguration('sensors')
    ekf = LaunchConfiguration('ekf')
    relay = LaunchConfiguration('relay')
    rf2o = LaunchConfiguration('rf2o')
    scan_min_period = LaunchConfiguration('scan_min_period_s')

    return LaunchDescription([
        DeclareLaunchArgument('sensors', default_value=str(ROOT / 'config/fusion_sim_sensors_real.yaml')),
        DeclareLaunchArgument('ekf', default_value=str(ROOT / 'config/fusion_ekf_moatasem.yaml')),
        DeclareLaunchArgument('relay', default_value=str(ROOT / 'config/cov_relay_rf2o.yaml')),
        DeclareLaunchArgument('rf2o', default_value=str(ROOT / 'config/rf2o_fusion.yaml')),
        DeclareLaunchArgument('scan_min_period_s', default_value='0.1'),

        ExecuteProcess(
            name='sim_vesc_imu',
            cmd=['python3', '-u', str(ROOT / 'tools/fusion/sim_vesc_imu.py'),
                 '--ros-args', '--params-file', sensors],
            output='screen'),

        ExecuteProcess(
            name='fusion_scan_relay',
            cmd=['python3', '-u', str(ROOT / 'tools/localization/fusion_scan_relay.py'),
                 '--ros-args', '-p', ['min_period_s:=', scan_min_period]],
            output='screen'),

        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            parameters=[rf2o],
            output='screen'),

        ExecuteProcess(
            name='cov_relay',
            cmd=['python3', '-u', str(ROOT / 'tools/fusion/cov_relay.py'),
                 '--ros-args', '--params-file', relay],
            output='screen'),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf],
            output='screen'),
    ])
