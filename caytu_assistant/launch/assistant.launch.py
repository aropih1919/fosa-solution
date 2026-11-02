"""Lance la carte sémantique, le dialogue et les adaptateurs audio."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from caytu_config import load_config


def generate_launch_description():
    configuration = load_config()
    input_mode = LaunchConfiguration('input_mode')
    audio_file = LaunchConfiguration('audio_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument(
            'input_mode', default_value=configuration.dialogue.default_input_mode,
            choices=['microphone', 'audio_file', 'text']),
        DeclareLaunchArgument('audio_file', default_value=''),
        DeclareLaunchArgument(
            'use_sim_time', default_value=str(
                configuration.system.use_sim_time).lower()),
        Node(
            package='caytu_semantic_map', executable='semantic_map_node',
            output='screen', parameters=[{'use_sim_time': use_sim_time}]),
        Node(
            package='caytu_assistant', executable='dialogue_manager',
            output='screen', parameters=[{'use_sim_time': use_sim_time}]),
        Node(
            package='caytu_assistant', executable='speech_input',
            output='screen', parameters=[
                {'use_sim_time': use_sim_time},
                {'input_mode': input_mode},
                {'audio_file': audio_file},
            ]),
        Node(
            package='caytu_assistant', executable='speech_output',
            output='screen', parameters=[{'use_sim_time': use_sim_time}]),
        Node(
            package='caytu_perception', executable='tag_detector',
            output='screen', parameters=[{'use_sim_time': use_sim_time}]),
        Node(
            package='caytu_perception', executable='obstacle_classifier',
            output='screen', parameters=[{'use_sim_time': use_sim_time}]),
        Node(
            package='caytu_perception', executable='person_tracker',
            output='screen', parameters=[{'use_sim_time': use_sim_time}]),
    ])
