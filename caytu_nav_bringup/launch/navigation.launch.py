"""Pile Nav2 de la solution Fosa : carte, planificateur, contrôleur, behaviors, BT."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Topic de commande du robot officiel (Twist non horodaté, voir gz_bridge.yaml).
TOPIC_VITESSE = "/robot_base_controller/cmd_vel_unstamped"

# map_server en premier : la costmap globale attend /map pour se dimensionner.
NOEUDS_CYCLE_VIE = [
    "map_server",
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
]


def _creer_noeuds_nav2(context):
    dossier_bringup = get_package_share_directory("caytu_nav_bringup")
    utiliser_temps_simulation = (
        LaunchConfiguration("use_sim_time").perform(context).lower() == "true")
    chemin_carte = LaunchConfiguration("map").perform(context)
    controleur = LaunchConfiguration("controller").perform(context)
    localisation = LaunchConfiguration("localization").perform(context)
    fichier_parametres_essai = LaunchConfiguration("params_file").perform(context)

    fichier_parametres = os.path.join(
        dossier_bringup, "config", "nav2_params.yaml")
    fichier_controleur = os.path.join(
        dossier_bringup, "config", f"controller_{controleur}.yaml")

    if not fichier_parametres_essai:
        if controleur == "mppi":
            raise RuntimeError(
                "MPPI exige le YAML complet params_file généré par task_solution.py.")
        if not os.path.isfile(fichier_controleur):
            raise RuntimeError(
                f"Contrôleur inconnu : {controleur} ({fichier_controleur} absent)")
    elif not os.path.isfile(fichier_parametres_essai):
        raise RuntimeError(
            f"Fichier de paramètres introuvable : {fichier_parametres_essai}")
    if localisation == "amcl" and not fichier_parametres_essai:
        raise RuntimeError(
            "AMCL exige un YAML complet avec pose initiale validée.")

    # Chemin résolu dans le package installé : aucun chemin personnel en dur.
    fichier_arbre_comportement = os.path.join(
        dossier_bringup, "behavior_trees", "navigate_bounded_recovery.xml")
    temps_simulation = {"use_sim_time": utiliser_temps_simulation}

    if fichier_parametres_essai:
        parametres_communs = [fichier_parametres_essai, temps_simulation]
        parametres_controleur = [fichier_parametres_essai, temps_simulation]
    else:
        parametres_communs = [fichier_parametres, temps_simulation]
        parametres_controleur = [
            fichier_parametres, fichier_controleur, temps_simulation]

    noeuds_cycle_vie = list(NOEUDS_CYCLE_VIE)
    if localisation == "amcl":
        noeuds_cycle_vie.insert(1, "amcl")

    noeuds = [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=parametres_communs + [{"yaml_filename": chemin_carte}],
        ),
    ]

    if localisation == "amcl":
        pose_initiale = {
            "x": float(LaunchConfiguration("initial_pose_x").perform(context)),
            "y": float(LaunchConfiguration("initial_pose_y").perform(context)),
            "z": 0.0,
            "yaw": float(LaunchConfiguration("initial_pose_yaw").perform(context)),
        }
        noeuds.append(Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=parametres_communs + [{
                "set_initial_pose": True,
                "always_reset_initial_pose": True,
                "initial_pose": pose_initiale,
                "use_sim_time": utiliser_temps_simulation,
            }],
        ))

    noeuds.extend([
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            # Le fichier du contrôleur est chargé après nav2_params.yaml : il
            # complète controller_server avec le plugin FollowPath choisi.
            parameters=parametres_controleur,
            remappings=[("cmd_vel", TOPIC_VITESSE)],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=parametres_communs,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=parametres_communs,
            remappings=[("cmd_vel", TOPIC_VITESSE)],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=parametres_communs + [{
                "default_nav_to_pose_bt_xml": fichier_arbre_comportement,
            }],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[temps_simulation, {
                "autostart": True,
                "node_names": noeuds_cycle_vie,
                # La simulation tourne bien plus lentement que le temps réel :
                # on laisse aux nœuds le temps de répondre avant de les
                # déclarer morts.
                "bond_timeout": 20.0,
                "attempt_respawn_reconnection": True,
                "bond_respawn_max_duration": 20.0,
            }],
        ),
    ])
    return noeuds


def generate_launch_description():
    dossier_bringup = get_package_share_directory("caytu_nav_bringup")
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Utiliser l'horloge Gazebo."),
        DeclareLaunchArgument(
            "map", default_value=os.path.join(
                dossier_bringup, "maps", "cafe_map.yaml"),
            description="Fichier YAML de la carte statique."),
        DeclareLaunchArgument(
            "controller", default_value="rpp", choices=["rpp", "dwb", "mppi"],
            description="Contrôleur local : fichier dédié ou YAML complet."),
        DeclareLaunchArgument(
            "localization", default_value="odom_imu",
            choices=["odom_imu", "odom_only", "amcl"],
            description="Localisation sélectionnée par solution_bringup.launch.py."),
        DeclareLaunchArgument(
            "profile", default_value="balanced",
            description="Profil de benchmark déjà fusionné dans params_file."),
        DeclareLaunchArgument(
            "params_file", default_value="",
            description="YAML complet généré par task_solution.py (essai benchmark)."),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
        OpaqueFunction(function=_creer_noeuds_nav2),
    ])
