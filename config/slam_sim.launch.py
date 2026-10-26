from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path

def generate_launch_description():
    params_file = str(Path(__file__).parent / 'slam_sim.yaml')

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': False},
        ],
        remappings=[
            ('map', '/slam_map'),
            ('map_metadata', '/slam_map_metadata'),
        ],
    )

    return LaunchDescription([slam])
