"""Bringup complet de la solution Fosa (lancé par task_solution.py).

Démarre, sans aucun délai ni intervention manuelle :
  1. le filtre d'auto-détection du lidar (/scan -> /scan_filtered) ;
  2. la mise à niveau (inclinaison du robot, retours du sol du lidar, repère
     horizontal base_footprint_level pour les caméras) ;
  3. la conversion des deux nuages de points caméra en LaserScan ;
  4. la localisation roues + IMU, qui publie map -> odom ;
  5. Nav2 (carte des murs, planificateur, contrôleur, behaviors, BT).

Utilisation directe (mise au point) :
    ros2 launch caytu_nav_bringup solution_bringup.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Repère horizontal publié par le filtre sol lidar (le robot penche de ~2,5°).
REPERE_HORIZONTAL = "base_footprint_level"


def _declarer_arguments(dossier_bringup):
    """Réglages modifiables en ligne de commande."""
    return [
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Utiliser /clock de Gazebo pour tous les nœuds."),
        DeclareLaunchArgument(
            "map", default_value=os.path.join(
                dossier_bringup, "maps", "cafe_map.yaml"),
            description="Carte statique (repère Gazebo). task_solution.py la "
                        "remplace par une carte vide si le monde n'est pas le café."),
        DeclareLaunchArgument(
            "controller", default_value="rpp", choices=["rpp", "dwb"],
            description="Contrôleur local : config/controller_<nom>.yaml."),
        DeclareLaunchArgument(
            "task_params_file", default_value="",
            description="task_params.yaml à utiliser (vide = package officiel)."),
        DeclareLaunchArgument(
            "top_cloud_topic", default_value="/top_camera_depth/points",
            description="Nuage de points de la caméra haute."),
        DeclareLaunchArgument(
            "bottom_cloud_topic", default_value="/bottom_camera_depth/points",
            description="Nuage de points de la caméra basse."),
        DeclareLaunchArgument(
            "floor_line_distance", default_value="0.0",
            description="Forçage : distance (m) de la ligne de sol lue droit "
                        "devant dans /scan. 0.0 = calcul automatique par l'IMU."),
    ]


def _creer_filtre_laser(dossier_bringup, utiliser_temps_simulation):
    """1. Le lidar est sous le châssis : quatre secteurs fixes voient les roues
    et la roulette. Nav2 ne consomme jamais /scan brut.
    """
    return Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="scan_to_scan_filter_chain",
        output="screen",
        parameters=[
            os.path.join(dossier_bringup, "config", "laser_filter_params.yaml"),
            {"use_sim_time": utiliser_temps_simulation},
        ],
        remappings=[("scan", "/scan"), ("scan_filtered", "/scan_filtered")],
    )


def _creer_filtre_sol(utiliser_temps_simulation):
    """/scan_filtered -> /scan_clean (marquage) et /scan_clear (effacement),
    plus la TF statique base_footprint -> base_footprint_level.
    """
    return Node(
        package="caytu_nav_solution",
        executable="filtre_sol_lidar",
        name="filtre_sol_lidar",
        output="screen",
        parameters=[{
            "use_sim_time": utiliser_temps_simulation,
            "level_frame": REPERE_HORIZONTAL,
            "floor_line_distance": ParameterValue(
                LaunchConfiguration("floor_line_distance"), value_type=float),
        }],
    )


def _creer_camera_haute(utiliser_temps_simulation):
    """3a. Caméra haute (horizontale, 1.01 m) : tables, chaises, personnes.

    Gazebo ajoute son bruit de profondeur (écart-type 0.10 m) directement sur
    x, y ET z de chaque point : le sol est donc bruité de ±0.10 m en hauteur.
    Dans le repère horizontal, min_height = 0.60 m représente 6 écarts-types
    (aucun faux obstacle dans nos simulations du capteur) et reste sous les
    plateaux de table (0.74 m) et les dossiers de chaise.
    """
    return Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="camera_haute_vers_scan",
        output="screen",
        remappings=[
            ("cloud_in", LaunchConfiguration("top_cloud_topic")),
            ("scan", "/top_camera_scan"),
        ],
        parameters=[{
            "use_sim_time": utiliser_temps_simulation,
            "target_frame": REPERE_HORIZONTAL,
            "transform_tolerance": 0.5,
            "min_height": 0.60,
            "max_height": 1.40,
            "angle_min": -0.785,
            "angle_max": 0.785,
            "angle_increment": 0.0087,
            "scan_time": 0.067,
            "range_min": 0.3,
            "range_max": 3.0,
            "use_inf": True,
            "inf_epsilon": 1.0,
        }],
    )


def _creer_camera_basse(utiliser_temps_simulation):
    """3b. Caméra basse (0.58 m, inclinée de 60° vers le sol) : obstacles bas et
    proches. Son bruit vertical est faible (±0.04 m) : 0.25 m suffit, ce que
    confirment nos journaux (0 point sur sol dégagé).
    """
    return Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="camera_basse_vers_scan",
        output="screen",
        remappings=[
            ("cloud_in", LaunchConfiguration("bottom_cloud_topic")),
            ("scan", "/bottom_camera_scan"),
        ],
        parameters=[{
            "use_sim_time": utiliser_temps_simulation,
            "target_frame": REPERE_HORIZONTAL,
            "transform_tolerance": 0.5,
            "min_height": 0.25,
            "max_height": 1.40,
            "angle_min": -0.785,
            "angle_max": 0.785,
            "angle_increment": 0.0087,
            "scan_time": 0.067,
            "range_min": 0.3,
            "range_max": 1.2,
            "use_inf": True,
            "inf_epsilon": 1.0,
        }],
    )


def _creer_localisateur(utiliser_temps_simulation):
    """4. Remplace AMCL : map -> odom depuis le départ de task_params.yaml, la
    distance des roues et le cap de l'IMU. `map` = repère Gazebo.
    """
    return Node(
        package="caytu_nav_solution",
        executable="localisateur_odom_imu",
        name="localisateur_odom_imu",
        output="screen",
        parameters=[{
            "use_sim_time": utiliser_temps_simulation,
            # value_type=str : une valeur vide doit rester une chaîne vide.
            "task_params_file": ParameterValue(
                LaunchConfiguration("task_params_file"), value_type=str),
        }],
    )


def _creer_navigation(dossier_bringup):
    """5. Nav2. Démarré tout de suite : le temps de la tâche court dès le
    lancement de la solution.
    """
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dossier_bringup, "launch", "navigation.launch.py")),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "map": LaunchConfiguration("map"),
            "controller": LaunchConfiguration("controller"),
        }.items(),
    )


def generate_launch_description():
    dossier_bringup = get_package_share_directory("caytu_nav_bringup")

    # Un argument de launch arrive toujours sous forme de texte ("true").
    # ParameterValue demande à ROS de le convertir en vrai booléen.
    utiliser_temps_simulation = ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool)

    return LaunchDescription(
        _declarer_arguments(dossier_bringup) + [
            _creer_filtre_laser(dossier_bringup, utiliser_temps_simulation),
            _creer_filtre_sol(utiliser_temps_simulation),
            _creer_camera_haute(utiliser_temps_simulation),
            _creer_camera_basse(utiliser_temps_simulation),
            _creer_localisateur(utiliser_temps_simulation),
            _creer_navigation(dossier_bringup),
        ])
