"""TEST 4: F1TENTH / MIT particle filter on the saved SLAM map, fed by Moatasem's EKF "C".

Counterpart of tools/localization/amcl_shadow.launch.py (odom_source:=ekf2) with the particle filter
instead of AMCL. Start the simulator first, then:

    zsh -c 'source scripts/sim_env.zsh && ros2 launch tools/localization/pf_shadow.launch.py'

Then seed it once on /fusion/initialpose (tools/localization/initialize_amcl_pose.py; never /initialpose).

Particle filter: github.com/f1tenth/particle_filter (humble-devel) + github.com/f1tenth/range_libc
(humble-devel), built in ~/Data/pf_ws (pf_ws:= argument):
    pf_ws/install/particle_filter              colcon build of the package
    pf_ws/src/range_libc/pywrapper             RangeLibc Python module, built in place (Cython 3.0.12)
    pf_ws/.venv_pf                             transforms3d 0.4.2 only (the Ubuntu 0.3.1 breaks with numpy 1.26)
Particle filter and adapter run with OPENBLAS_NUM_THREADS=OMP_NUM_THREADS=MKL_NUM_THREADS=1 (see ONE_THREAD).
Parameters: config/particle_filter_moatasem.yaml (package values; see its header).

Arguments:
    map_yaml:=results/missions/autonomous_20261103_114924_818822/new_track.yaml
    sensors:=config/fusion_sim_sensors_calibrated.yaml    (sim VESC + IMU)
    ekf:=config/fusion_ekf_moatasem.yaml                  (EKF "C": VESC vx + gyro vyaw)
    pf_params:=config/particle_filter_moatasem.yaml
    pose_topic:=/amcl_pose                                (adapter output, PoseWithCovarianceStamped)
    adapter_log:=none                                     (or a path: CSV of every map->odom update)

TF ownership (exactly one publisher per edge):
    slam_map    -> fusion_odom  : pf_map_odom_adapter.py (from the particle filter pose)
    fusion_odom -> fusion_base  : EKF
    fusion_base -> fusion_laser : fusion_scan_relay (static)
    The particle filter's own '/map'->'/laser' TF is remapped to /pf/tf_raw and never reaches /tf.
Remaps of the particle filter: /tf -> /pf/tf_raw, /initialpose -> /fusion/initialpose,
/map_server/map -> /fusion_map_server/map, /clicked_point -> /pf/clicked_point_unused.
"""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[2]
PF_WS_DEFAULT = str(Path.home() / 'Data/pf_ws')
# numpy's bundled OpenBLAS starts one busy-waiting thread per core for every small np.cov / np.dot in the
# filter loop (first test run: 269 % of a core). One BLAS thread per node, as on the car computer.
ONE_THREAD = {'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1'}


def generate_launch_description():
    pf_ws = os.environ.get('PF_WS', PF_WS_DEFAULT)
    pf_path = os.pathsep.join([
        f'{pf_ws}/install/particle_filter/lib/python3.10/site-packages',
        f'{pf_ws}/src/range_libc/pywrapper',
        f'{pf_ws}/.venv_pf/lib/python3.10/site-packages',
        os.environ.get('PYTHONPATH', '')])
    map_yaml = LaunchConfiguration('map_yaml')
    return LaunchDescription([
        DeclareLaunchArgument('map_yaml',
                              default_value=str(ROOT / 'results/missions/autonomous_20261103_114924_818822/new_track.yaml')),
        DeclareLaunchArgument('sensors', default_value=str(ROOT / 'config/fusion_sim_sensors_calibrated.yaml')),
        DeclareLaunchArgument('ekf', default_value=str(ROOT / 'config/fusion_ekf_moatasem.yaml')),
        DeclareLaunchArgument('pf_params', default_value=str(ROOT / 'config/particle_filter_moatasem.yaml')),
        DeclareLaunchArgument('pose_topic', default_value='/amcl_pose'),
        DeclareLaunchArgument('adapter_log', default_value='none'),

        # ---------------- map (same server, topic and frame as amcl_shadow.launch.py) ----------------
        Node(package='nav2_map_server', executable='map_server', name='fusion_map_server', output='screen',
             parameters=[str(ROOT / 'config/fusion_amcl_moatasem.yaml'), {'yaml_filename': map_yaml}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='fusion_localization_lifecycle_manager', output='screen',
             parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': ['fusion_map_server']}]),

        # ---------------- odometry: EKF "C" (as odom_source:=ekf2) ----------------
        ExecuteProcess(name='sim_vesc_imu',
                       cmd=['python3', '-u', str(ROOT / 'tools/fusion/sim_vesc_imu.py'),
                            '--ros-args', '--params-file', LaunchConfiguration('sensors')], output='screen'),
        ExecuteProcess(name='fusion_scan_relay',
                       cmd=['python3', '-u', str(ROOT / 'tools/localization/fusion_scan_relay.py')], output='screen'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=[LaunchConfiguration('ekf')], output='screen'),

        # ---------------- particle filter + adapter ----------------
        ExecuteProcess(
            name='particle_filter',
            cmd=['/usr/bin/python3', '-u', f'{pf_ws}/install/particle_filter/lib/particle_filter/particle_filter',
                 '--ros-args', '-r', '__node:=particle_filter', '--params-file', LaunchConfiguration('pf_params'),
                 '-r', '/tf:=/pf/tf_raw', '-r', '/initialpose:=/fusion/initialpose',
                 '-r', '/map_server/map:=/fusion_map_server/map', '-r', '/clicked_point:=/pf/clicked_point_unused'],
            additional_env={'PYTHONPATH': pf_path, **ONE_THREAD}, output='screen'),
        ExecuteProcess(
            name='pf_map_odom_adapter',
            cmd=['python3', '-u', str(ROOT / 'tools/localization/pf_map_odom_adapter.py'), '--ros-args',
                 '-p', ['pose_topic:=', LaunchConfiguration('pose_topic')],
                 '-p', ['log_csv:=', LaunchConfiguration('adapter_log')]],
            additional_env=ONE_THREAD, output='screen'),
    ])
