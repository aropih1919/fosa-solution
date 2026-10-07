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

# Repère horizontal publié par lidar_floor_filter (le robot penche de ~2,5°).
LEVEL_FRAME = "base_footprint_level"


def generate_launch_description():
    bringup_dir = get_package_share_directory("caytu_nav_bringup")

    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    arguments = [
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Utiliser /clock de Gazebo pour tous les nœuds."),
        DeclareLaunchArgument(
            "map", default_value=os.path.join(bringup_dir, "maps", "cafe_map.yaml"),
            description="Carte statique (repère Gazebo). task_solution.py la "
                        "remplace par une carte vide si le monde n'est pas le café."),
        DeclareLaunchArgument(
            "behavior_tree", default_value="if_invalid", choices=["if_invalid", "periodic"],
            description="Replanification : seulement si le chemin devient invalide "
                        "(défaut) ou périodique à 2 Hz."),
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
        DeclareLaunchArgument(
            "map_correction", default_value="false", choices=["true", "false"],
            description="Recalage sur les murs (defaut desactive)."),
    ]

    # 1. Le lidar est sous le châssis : quatre secteurs fixes voient les roues
    #    et la roulette. Nav2 ne consomme jamais /scan brut.
    laser_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="scan_to_scan_filter_chain",
        output="screen",
        parameters=[
            os.path.join(bringup_dir, "config", "laser_filter_params.yaml"),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("scan", "/scan"), ("scan_filtered", "/scan_filtered")],
    )

    # 2. /scan_filtered -> /scan_clean (marquage) et /scan_clear (effacement),
    #    plus la TF statique base_footprint -> base_footprint_level.
    floor_filter = Node(
        package="caytu_nav_solution",
        executable="lidar_floor_filter",
        name="lidar_floor_filter",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "level_frame": LEVEL_FRAME,
            "floor_line_distance": ParameterValue(
                LaunchConfiguration("floor_line_distance"), value_type=float),
        }],
    )

    # 3a. Caméra haute (horizontale, 1.01 m) : tables, chaises, personnes.
    #     Gazebo ajoute son bruit de profondeur (écart-type 0.10 m) directement
    #     sur x, y ET z de chaque point : le sol est donc bruité de ±0.10 m en
    #     hauteur. Dans le repère horizontal, min_height = 0.60 m représente
    #     6 écarts-types (aucun faux obstacle dans nos simulations du capteur)
    #     et reste sous les plateaux de table (0.74 m) et les dossiers de chaise.
    top_cam_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="top_cam_to_scan",
        output="screen",
        remappings=[
            ("cloud_in", LaunchConfiguration("top_cloud_topic")),
            ("scan", "/top_camera_scan"),
        ],
        parameters=[{
            "use_sim_time": use_sim_time,
            "target_frame": LEVEL_FRAME,
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

    # 3b. Caméra basse (0.58 m, inclinée de 60° vers le sol) : obstacles bas et
    #     proches. Son bruit vertical est faible (±0.04 m) : 0.25 m suffit, ce
    #     que confirment nos journaux (0 point sur sol dégagé).
    bottom_cam_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="bottom_cam_to_scan",
        output="screen",
        remappings=[
            ("cloud_in", LaunchConfiguration("bottom_cloud_topic")),
            ("scan", "/bottom_camera_scan"),
        ],
        parameters=[{
            "use_sim_time": use_sim_time,
            "target_frame": LEVEL_FRAME,
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

    # 4. Remplace AMCL : map -> odom depuis le spawn de task_params.yaml, la
    #    distance des roues et le cap de l'IMU. `map` = repère Gazebo.
    localizer = Node(
        package="caytu_nav_solution",
        executable="odom_imu_localizer",
        name="odom_imu_localizer",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            # value_type=str : une valeur vide doit rester une chaîne vide.
            "task_params_file": ParameterValue(
                LaunchConfiguration("task_params_file"), value_type=str),
            "enable_map_correction": ParameterValue(
                LaunchConfiguration("map_correction"), value_type=bool),
            "map_yaml": ParameterValue(
                LaunchConfiguration("map"), value_type=str),
        }],
    )

    # 5. Nav2. Démarré tout de suite : le temps de la tâche court dès le
    #    lancement de la solution.
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation.launch.py")),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "map": LaunchConfiguration("map"),
            "behavior_tree": LaunchConfiguration("behavior_tree"),
        }.items(),
    )

    return LaunchDescription(arguments + [
        laser_filter,
        floor_filter,
        top_cam_to_scan,
        bottom_cam_to_scan,
        localizer,
        navigation,
    ])
