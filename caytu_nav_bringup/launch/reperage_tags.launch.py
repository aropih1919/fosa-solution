"""Repérage des tags sur une carte déjà faite (après mapping.launch.py).

Pendant la cartographie, la carte est corrigée après coup : les tags notés
alors peuvent être décalés de plusieurs dizaines de centimètres dans la carte
finale. Ici, le robot se localise sur la carte enregistrée (slam_toolbox),
et les tags sont notés dans ce repère, celui de la course.

    ros2 launch caytu_nav_bringup reperage_tags.launch.py robot:=real_remote \\
        carte:=$HOME/fosa_maps/arene_dakar
    ros2 run teleop_twist_keyboard teleop_twist_keyboard ...   # passer devant chaque tag

Le robot doit partir de la même position qu'au début de la cartographie.
<carte>/tags.yaml est réécrit (anciennes positions oubliées).
"""

import os

from ament_index_python.packages import get_package_share_directory
from caytu_config import load_config
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

CONFIG = load_config()


def _creer(context):
    dossier_bringup = get_package_share_directory("caytu_nav_bringup")
    robot = LaunchConfiguration("robot").perform(context)
    carte = os.path.abspath(os.path.expanduser(LaunchConfiguration("carte").perform(context)))
    if not os.path.isfile(os.path.join(carte, "fosa_map.yaml")):
        raise RuntimeError(f"carte:={carte} : fosa_map.yaml introuvable (mapping.launch.py)")
    temps = LaunchConfiguration("use_sim_time").perform(context).strip().lower()
    if temps not in ("true", "false"):
        temps = "true" if robot in ("sim", "parc") else "false"
    carte_nav = os.path.join(carte, "nav_map.yaml")
    if not os.path.isfile(carte_nav):
        carte_nav = os.path.join(carte, "map.yaml")
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(dossier_bringup, "launch", "solution_bringup.launch.py")),
            launch_arguments={
                "robot": robot, "use_sim_time": temps, "localization": "slam",
                "slam_map_dir": carte, "map": carte_nav,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(dossier_bringup, "launch", "tags.launch.py")),
            launch_arguments={
                "use_sim_time": temps,
                "fichier_tags": os.path.join(carte, CONFIG.course.tags_file),
                "recharger": "false",
                "detecteur": LaunchConfiguration("detecteur"),
                "image_topic": LaunchConfiguration("image_topic"),
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="sim",
                              choices=["sim", "parc", "real", "real_remote"]),
        DeclareLaunchArgument("use_sim_time", default_value=""),
        DeclareLaunchArgument("carte", description="Dossier de la carte enregistrée."),
        DeclareLaunchArgument("detecteur", default_value="",
                              choices=["", "opencv", "apriltag_ros"]),
        DeclareLaunchArgument("image_topic", default_value=""),
        OpaqueFunction(function=_creer),
    ])
