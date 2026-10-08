"""Bringup complet de la solution Fosa (lancé par task_solution.py).

Démarre, sans aucun délai ni intervention manuelle :
  1. le filtre d'auto-détection du lidar (/scan -> /scan_filtered) ;
  2. la mise à niveau (inclinaison du robot, retours du sol du lidar, repère
     horizontal base_footprint_level pour les caméras) ;
  3. la conversion des deux nuages de points caméra en LaserScan ;
  4. la localisation roues + IMU, qui publie map -> odom ;
  5. le nœud qui stabilise le chemin (path_keeper), s'il est demandé ;
  6. Nav2 (carte des murs, planificateur, contrôleur, behaviors, BT).

Utilisation directe (mise au point) :
    ros2 launch caytu_nav_bringup solution_bringup.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Repère horizontal publié par lidar_floor_filter (le robot penche de ~2,5°).
LEVEL_FRAME = "base_footprint_level"

# Un nœud qui s'arrête par accident est relancé au bout de 2 s : sans lui, le
# robot perdrait un capteur jusqu'à la fin du trajet. La localisation fait
# exception (voir _localizer_node).
RESPAWN = {"respawn": True, "respawn_delay": 2.0}

# Portée de marquage de la caméra haute (m). Doit rester égale à
# obstacle_max_range de top_cam dans nav2_params.yaml.
TOP_CAMERA_RANGE = 2.5
# Le scan porte plus loin que le marquage : un rayon sans obstacle arrive à
# range_max et ne doit pas être marqué (voir nav2_params.yaml).
CAMERA_CLEAR_MARGIN = 0.5

# Réglages de conduite transmis tels quels à navigation.launch.py.
NAV2_DRIVE_ARGUMENTS = (
    "cruise_speed", "turn_speed", "approach_speed", "approach_distance",
    "curve_radius", "camera_range")


def _top_camera_node(context):
    """Caméra haute -> LaserScan, avec la portée demandée pour cet essai."""
    text = LaunchConfiguration("camera_range").perform(context).strip()
    reach = float(text) if text else TOP_CAMERA_RANGE
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    # Gazebo ajoute son bruit de profondeur (écart-type 0.10 m) directement
    # sur x, y ET z de chaque point : le sol est donc bruité de ±0.10 m en
    # hauteur. Dans le repère horizontal, min_height = 0.60 m représente
    # 6 écarts-types (aucun faux obstacle dans nos simulations du capteur)
    # et reste sous les plateaux de table (0.74 m) et les dossiers de chaise.
    return [Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        **RESPAWN,
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
            "range_max": reach + CAMERA_CLEAR_MARGIN,
            "use_inf": True,
            "inf_epsilon": 1.0,
        }],
    )]


def _path_keeper_node(context):
    """Chemin stabilisé : entre l'arbre de comportement et le planificateur."""
    if LaunchConfiguration("path_keeper").perform(context).lower() != "true":
        return []
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    parameters = {"use_sim_time": use_sim_time}
    text = LaunchConfiguration("keeper_gain").perform(context).strip()
    if text:
        parameters["min_gain"] = float(text)
    return [Node(
        package="caytu_nav_solution",
        executable="path_keeper",
        **RESPAWN,
        name="path_keeper",
        output="screen",
        parameters=[parameters],
    )]


def _localizer_node(context):
    """Localisation roues + IMU ; axle_offset n'est imposé que pour un essai."""
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    parameters = {
        "use_sim_time": use_sim_time,
        "task_params_file": LaunchConfiguration("task_params_file").perform(context),
    }
    text = LaunchConfiguration("axle_offset").perform(context).strip()
    if text:
        parameters["axle_offset"] = float(text)
    # Pas de relance automatique : relancé en cours de route, ce nœud repartirait
    # du point de départ avec l'odométrie du plugin, dont le cap est faux de 15 %,
    # et publierait une position fausse au lieu de s'arrêter.
    return [Node(
        package="caytu_nav_solution",
        executable="odom_imu_localizer",
        name="odom_imu_localizer",
        output="screen",
        parameters=[parameters],
    )]


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
            "path_keeper", default_value="true", choices=["true", "false"],
            description="Lancer le nœud qui stabilise le chemin (path_keeper)."),
        DeclareLaunchArgument(
            "keeper_gain", default_value="",
            description="Essai : gain (m) exigé pour changer de route. "
                        "Vide = valeur du nœud (1.5)."),
        DeclareLaunchArgument(
            "axle_offset", default_value="",
            description="Essai : distance base_footprint -> essieu (m). "
                        "Vide = valeur du nœud (0.095)."),
    ]
    arguments += [
        DeclareLaunchArgument(
            name, default_value="",
            description="Essai : réglage de conduite, voir navigation.launch.py.")
        for name in NAV2_DRIVE_ARGUMENTS
    ]

    # 1. Le lidar est sous le châssis : quatre secteurs fixes voient les roues
    #    et la roulette. Nav2 ne consomme jamais /scan brut.
    laser_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        **RESPAWN,
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
        **RESPAWN,
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
    top_cam_to_scan = OpaqueFunction(function=_top_camera_node)

    # 3b. Caméra basse (0.58 m, inclinée de 60° vers le sol) : obstacles bas et
    #     proches. Son bruit vertical est faible (±0.04 m) : 0.25 m suffit, ce
    #     que confirment nos journaux (0 point sur sol dégagé).
    bottom_cam_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        **RESPAWN,
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
    localizer = OpaqueFunction(function=_localizer_node)

    # 5. Chemin stabilisé (si demandé).
    path_keeper = OpaqueFunction(function=_path_keeper_node)

    # 6. Nav2. Démarré tout de suite : le temps de la tâche court dès le
    #    lancement de la solution.
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation.launch.py")),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "map": LaunchConfiguration("map"),
            **{name: LaunchConfiguration(name) for name in NAV2_DRIVE_ARGUMENTS},
        }.items(),
    )

    return LaunchDescription(arguments + [
        laser_filter,
        floor_filter,
        top_cam_to_scan,
        bottom_cam_to_scan,
        localizer,
        path_keeper,
        navigation,
    ])
