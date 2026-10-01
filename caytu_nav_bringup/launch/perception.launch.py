"""perception.launch.py  ->  à copier dans caytu_nav_bringup/launch/perception.launch.py

Lance la perception caméra (obstacles en hauteur) + son watchdog.
Le bridge de l'image RGB est OPTIONNEL (inutile tant que crowd_detector n'existe pas) :
  ros2 launch caytu_nav_bringup perception.launch.py enable_image_bridge:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("caytu_nav_bringup")
    cam_params = os.path.join(
        get_package_share_directory("caytu_camera_perception"), "config", "camera_obstacles.yaml"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("enable_image_bridge", default_value="false",
                              description="Bridge top_camera_color/image_raw (crowd_detector)"),

        Node(
            package="caytu_camera_perception",
            executable="camera_obstacle_node",
            name="camera_obstacle_node",
            output="screen",
            parameters=[cam_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="caytu_camera_perception",
            executable="camera_perception_watchdog",
            name="camera_perception_watchdog",
            output="screen",
            parameters=[cam_params, {"use_sim_time": use_sim_time}],
        ),
        # Bridge SUPPLÉMENTAIRE, lancé depuis NOTRE launch : gz_bridge.yaml des organisateurs reste intact.
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="camera_image_bridge",
            output="screen",
            parameters=[{
                "config_file": os.path.join(bringup_dir, "config", "camera_bridge.yaml"),
                "use_sim_time": use_sim_time,
            }],
            condition=IfCondition(LaunchConfiguration("enable_image_bridge")),
        ),
    ])
