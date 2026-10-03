import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='caytu_camera_perception',
            executable='height_filter_node',
            name='height_filter_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='caytu_camera_perception',
            executable='camera_perception_watchdog',
            name='camera_perception_watchdog',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
