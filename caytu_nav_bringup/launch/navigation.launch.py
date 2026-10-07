"""Pile Nav2 de la solution Fosa : carte, planificateur, contrôleur, behaviors, BT."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Topic de commande du robot officiel (Twist non horodaté, voir gz_bridge.yaml).
CMD_VEL_TOPIC = "/robot_base_controller/cmd_vel_unstamped"

# map_server en premier : la costmap globale attend /map pour se dimensionner.
LIFECYCLE_NODES = [
    "map_server",
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
]


def _nav2_nodes(context):
    bringup_dir = get_package_share_directory("caytu_nav_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    map_yaml = LaunchConfiguration("map").perform(context)
    behavior_tree = LaunchConfiguration("behavior_tree").perform(context)

    params_file = os.path.join(bringup_dir, "config", "nav2_params.yaml")
    # Chemin résolu dans le package installé : aucun chemin personnel en dur.
    bt_xml_file = os.path.join(
        bringup_dir, "behavior_trees", f"navigate_replan_{behavior_tree}.xml")
    if not os.path.isfile(bt_xml_file):
        raise RuntimeError(f"Behavior tree inconnu : {behavior_tree} ({bt_xml_file} absent)")
    sim_time = {"use_sim_time": use_sim_time}

    return [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[sim_time, {"yaml_filename": map_yaml}],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params_file, sim_time],
            remappings=[("cmd_vel", CMD_VEL_TOPIC)],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params_file, sim_time],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[params_file, sim_time],
            remappings=[("cmd_vel", CMD_VEL_TOPIC)],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
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
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Utiliser l'horloge Gazebo."),
        DeclareLaunchArgument(
            "map", default_value=os.path.join(bringup_dir, "maps", "cafe_map.yaml"),
            description="Fichier YAML de la carte statique."),
        DeclareLaunchArgument(
            "behavior_tree", default_value="if_invalid", choices=["if_invalid", "periodic"],
            description="Replanification : seulement si le chemin devient invalide "
                        "(défaut) ou périodique à 2 Hz."),
        OpaqueFunction(function=_nav2_nodes),
    ])
