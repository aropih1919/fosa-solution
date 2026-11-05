"""Adaptation au robot : repère au sol et filtres du lidar.

Le profil config/robots/<robot>.yaml décrit les interfaces du robot :
  sim   robot de la phase 1 dans Gazebo (défaut) ;
  real  CAYTU Sito-É réel de la phase finale (relevé officiel du 2 novembre).

Ce lancement démarre :
  - robot réel seulement : la TF statique base_link -> base_footprint (l'arbre
    TF réel s'arrête à base_link) ;
  - /scan -> laser_filters -> /fosa/scan_filtered. Le nom est à nous : sur le
    robot réel, /scan_filtered appartient aux organisateurs et alimente leur
    détecteur d'obstacles ;
  - /fosa/scan_filtered -> filtre_sol_lidar -> /scan_clean (marquage et
    cartographie), /scan_clear (effacement), plus la TF statique
    base_footprint -> base_footprint_level (repère horizontal des caméras).

Inclus par solution_bringup.launch.py et mapping.launch.py.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from caytu_config import load_config
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Repère horizontal publié par le filtre sol (caytu_config, comme les caméras).
LEVEL_FRAME = load_config().perception.top_scan.target_frame
FILTERED_SCAN = "/fosa/scan_filtered"


def load_robot_profile(name: str) -> dict:
    path = os.path.join(get_package_share_directory("caytu_nav_bringup"),
                        "config", "robots", f"{name}.yaml")
    if not os.path.isfile(path):
        raise RuntimeError(f"Profil de robot inconnu : {name} ({path} absent)")
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _setup(context):
    bringup_dir = get_package_share_directory("caytu_nav_bringup")
    robot = LaunchConfiguration("robot").perform(context)
    profile = load_robot_profile(robot)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    actions = [LogInfo(msg=f"Profil du robot : {robot}")]

    base_footprint = profile.get("base_footprint") or {}
    if base_footprint.get("publish"):
        height = float(base_footprint.get("base_link_height", 0.0))
        actions.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="fosa_base_footprint_tf",
            output="log",
            arguments=["--x", "0", "--y", "0", "--z", f"{-height:.4f}",
                       "--roll", "0", "--pitch", "0", "--yaw", "0",
                       "--frame-id", str(base_footprint.get("parent", "base_link")),
                       "--child-frame-id", "base_footprint"],
        ))

    # Retrait des retours où le lidar voit le robot lui-même.
    actions.append(Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="scan_to_scan_filter_chain",
        output="screen",
        parameters=[
            os.path.join(bringup_dir, "config", profile["laser_filter_file"]),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("scan", profile["scan_topic"]), ("scan_filtered", FILTERED_SCAN)],
    ))

    floor_params = {
        "use_sim_time": use_sim_time,
        "scan_in": FILTERED_SCAN,
        "level_frame": LEVEL_FRAME,
        "floor_line_distance": float(LaunchConfiguration("floor_line_distance").perform(context)),
    }
    # Les chaînes passent par ParameterValue : une chaîne vide (pas d'IMU)
    # doit rester une chaîne et non devenir « null ».
    floor_params.update({
        key: ParameterValue(value, value_type=str) if isinstance(value, str) else value
        for key, value in (profile.get("floor_filter") or {}).items()})
    actions.append(Node(
        package="caytu_nav_solution",
        executable="filtre_sol_lidar",
        name="filtre_sol_lidar",
        output="screen",
        parameters=[floor_params],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot", default_value="sim", choices=["sim", "parc", "real", "real_remote"],
            description="sim : Gazebo phase 1 ; real / real_remote : robot réel (config/robots/)."),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Utiliser /clock (Gazebo ou rejeu d'un enregistrement)."),
        DeclareLaunchArgument(
            "floor_line_distance", default_value="0.0",
            description="Forçage : distance (m) de la ligne de sol lue droit "
                        "devant dans /scan. 0.0 = calcul automatique."),
        OpaqueFunction(function=_setup),
    ])
