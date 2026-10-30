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
    controller = LaunchConfiguration("controller").perform(context)
    localization = LaunchConfiguration("localization").perform(context)
    params_file = LaunchConfiguration("params_file").perform(context)

    default_params_file = os.path.join(bringup_dir, "config", "nav2_params.yaml")
    controller_file = os.path.join(bringup_dir, "config", f"controller_{controller}.yaml")
    if not params_file:
        if controller == "mppi":
            raise RuntimeError(
                "MPPI exige le YAML complet params_file généré par task_solution.py.")
        if not os.path.isfile(controller_file):
            raise RuntimeError(
                f"Contrôleur inconnu : {controller} ({controller_file} absent)")
    elif not os.path.isfile(params_file):
        raise RuntimeError(f"Fichier de paramètres introuvable : {params_file}")
    if localization == "amcl" and not params_file:
        raise RuntimeError("AMCL exige un YAML complet avec pose initiale validée.")

    # Chemin résolu dans le package installé : aucun chemin personnel en dur.
    bt_xml_file = os.path.join(
        bringup_dir, "behavior_trees", "navigate_bounded_recovery.xml")
    sim_time = {"use_sim_time": use_sim_time}
    if params_file:
        common_parameters = [params_file, sim_time]
        controller_parameters = [params_file, sim_time]
    else:
        common_parameters = [default_params_file, sim_time]
        controller_parameters = [default_params_file, controller_file, sim_time]

    lifecycle_nodes = list(LIFECYCLE_NODES)
    if localization == "amcl":
        lifecycle_nodes.insert(1, "amcl")

    nodes = [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=common_parameters + [{"yaml_filename": map_yaml}],
        ),
    ]

    if localization == "amcl":
        initial_pose = {
            "x": float(LaunchConfiguration("initial_pose_x").perform(context)),
            "y": float(LaunchConfiguration("initial_pose_y").perform(context)),
            "z": 0.0,
            "yaw": float(LaunchConfiguration("initial_pose_yaw").perform(context)),
        }
        nodes.append(Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=common_parameters + [{
                "set_initial_pose": True,
                "always_reset_initial_pose": True,
                "initial_pose": initial_pose,
                "use_sim_time": use_sim_time,
            }],
        ))

    nodes.extend([
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            # Le fichier du contrôleur est chargé après nav2_params.yaml : il
            # complète controller_server avec le plugin FollowPath choisi.
            parameters=controller_parameters,
            remappings=[("cmd_vel", CMD_VEL_TOPIC)],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=common_parameters,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=common_parameters,
            remappings=[("cmd_vel", CMD_VEL_TOPIC)],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=common_parameters + [{"default_nav_to_pose_bt_xml": bt_xml_file}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[sim_time, {
                "autostart": True,
                "node_names": lifecycle_nodes,
                # La simulation tourne bien plus lentement que le temps réel :
                # on laisse aux nœuds le temps de répondre avant de les
                # déclarer morts.
                "bond_timeout": 20.0,
                "attempt_respawn_reconnection": True,
                "bond_respawn_max_duration": 20.0,
            }],
        ),
    ])
    return nodes


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
            "controller", default_value="rpp", choices=["rpp", "dwb", "mppi"],
            description="Contrôleur local sélectionné dans le YAML complet."),
        DeclareLaunchArgument(
            "localization", default_value="odom_imu",
            choices=["odom_imu", "odom_only", "amcl"],
            description="Localisation sélectionnée par solution_bringup.launch.py."),
        DeclareLaunchArgument("profile", default_value="balanced"),
        DeclareLaunchArgument("params_file", default_value=""),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
        OpaqueFunction(function=_nav2_nodes),
    ])
