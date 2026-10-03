import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('caytu_nav_surcouche')
    return LaunchDescription([
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[
                os.path.join(pkg, 'config', 'collision_monitor_params.yaml'),
                {'use_sim_time': True},
            ],
        ),
        Node(
            package='caytu_nav_surcouche',
            executable='adaptive_inflation',
            name='adaptive_inflation',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
