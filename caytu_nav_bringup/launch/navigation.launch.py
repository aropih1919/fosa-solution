"""Nav2 pour la solution Fosa : carte, planificateur, contrôleur, behaviors, BT."""

import atexit
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Topic de commande du robot officiel (Twist non horodaté, voir gz_bridge.yaml).
CMD_VEL_TOPIC = "/robot_base_controller/cmd_vel_unstamped"

# Un serveur Nav2 qui s'arrête par accident est relancé ; le lifecycle manager
# (attempt_respawn_reconnection) le réactive ensuite avec les autres.
RESPAWN = {"respawn": True, "respawn_delay": 2.0}

# map_server en premier : la costmap globale attend /map pour se dimensionner.
LIFECYCLE_NODES = [
    "map_server",
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
]

# Réglages de conduite que l'on peut imposer pour un essai (vide = valeur de
# nav2_params.yaml). Voir caytu_nav_solution/drive_settings.py.
DRIVE_ARGUMENTS = {
    "cruise_speed": "Vitesse de croisière (m/s).",
    "turn_speed": "Vitesse de pivot sur place (rad/s).",
    "approach_speed": "Vitesse minimale à l'arrivée (m/s).",
    "approach_distance": "Distance de ralentissement avant le but (m).",
    "curve_radius": "Rayon de virage sous lequel le robot ralentit (m).",
    "camera_range": "Portée de marquage de la caméra haute (m).",
}


def _params_file(context, bringup_dir):
    """nav2_params.yaml tel quel, ou une copie modifiée si un essai le demande."""
    source = os.path.join(bringup_dir, "config", "nav2_params.yaml")
    texts = {name: LaunchConfiguration(name).perform(context) for name in DRIVE_ARGUMENTS}
    if not any(text.strip() for text in texts.values()):
        return source
    # Import local : le cas normal (aucun essai) ne dépend pas de ce module.
    from caytu_nav_solution.drive_settings import apply_to_nav2_params, from_launch_text
    with open(source, encoding="utf-8") as stream:
        params = yaml.safe_load(stream)
    apply_to_nav2_params(params, from_launch_text(texts))
    handle, path = tempfile.mkstemp(prefix="fosa_nav2_", suffix=".yaml")
    with os.fdopen(handle, "w", encoding="utf-8") as stream:
        yaml.safe_dump(params, stream, allow_unicode=True)
    # Copie effacée quand ros2 launch se termine.
    atexit.register(lambda: os.path.exists(path) and os.remove(path))
    return path


def _nav2_nodes(context):
    bringup_dir = get_package_share_directory("caytu_nav_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    map_yaml = LaunchConfiguration("map").perform(context)

    params_file = _params_file(context, bringup_dir)
    # Chemin résolu dans le package installé : aucun chemin personnel en dur.
    bt_xml_file = os.path.join(
        bringup_dir, "behavior_trees", "navigate_bounded_recovery.xml")
    sim_time = {"use_sim_time": use_sim_time}

    return [
        Node(
            package="nav2_map_server",
            executable="map_server",
            **RESPAWN,
            name="map_server",
            output="screen",
            parameters=[sim_time, {"yaml_filename": map_yaml}],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            **RESPAWN,
            name="controller_server",
            output="screen",
            parameters=[params_file, sim_time],
            remappings=[("cmd_vel", CMD_VEL_TOPIC)],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            **RESPAWN,
            name="planner_server",
            output="screen",
            parameters=[params_file, sim_time],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            **RESPAWN,
            name="behavior_server",
            output="screen",
            parameters=[params_file, sim_time],
            remappings=[("cmd_vel", CMD_VEL_TOPIC)],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            **RESPAWN,
            name="bt_navigator",
            output="screen",
            parameters=[params_file, sim_time, {"default_nav_to_pose_bt_xml": bt_xml_file}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[sim_time, {
                "autostart": True,
                "node_names": LIFECYCLE_NODES,
                # La simulation tourne bien plus lentement que le temps réel :
                # on laisse aux nœuds le temps de répondre avant de les
                # déclarer morts.
                "bond_timeout": 20.0,
                "attempt_respawn_reconnection": True,
                "bond_respawn_max_duration": 20.0,
            }],
        ),
    ]


def generate_launch_description():
    bringup_dir = get_package_share_directory("caytu_nav_bringup")
    arguments = [
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Utiliser /clock de Gazebo."),
        DeclareLaunchArgument(
            "map", default_value=os.path.join(bringup_dir, "maps", "cafe_map.yaml"),
            description="Fichier YAML de la carte statique."),
    ]
    arguments += [
        DeclareLaunchArgument(
            name, default_value="",
            description=f"Essai : {text} Vide = valeur de nav2_params.yaml.")
        for name, text in DRIVE_ARGUMENTS.items()
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_nav2_nodes)])
