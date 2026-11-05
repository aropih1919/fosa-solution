"""Course de la phase finale : tout ce qui tourne pendant les 10 minutes.

Lance :
  1. la navigation (solution_bringup.launch.py) :
       carte:=<dossier>  slam_toolbox se localise sur la carte enregistrée
                         pendant les essais (mapping.launch.py) ;
       carte:=''         cartographie en direct (si les cartes préparées sont
                         interdites vendredi) ;
  2. la détection et le registre des AprilTags (tags.launch.py), qui relit
     <carte>/tags.yaml ;
  3. le garde-fou nav2_collision_monitor (securite:=true) : arrêt devant
     tout objet, personnes comprises ;
  4. le démarrage vocal (voix:=true) : mot « allez » avec Whisper, hors ligne ;
  5. course_tags : ordre des tags lu sur le topic des organisateurs, départ
     au signal, visites à 0,5 m, statut.

Le robot reste immobile jusqu'au départ :
    voix      dire « allez » ;
    commande  ros2 run caytu_nav_solution depart

Exemples :
    Robot réel (portable)  ros2 launch caytu_nav_bringup course.launch.py robot:=real_remote \\
                               carte:=$HOME/fosa_maps/arene_dakar voix:=false
    Gazebo                 ros2 launch caytu_nav_bringup course.launch.py robot:=sim \\
                               carte:=$HOME/fosa_maps/cafe_test
"""

import os

from ament_index_python.packages import get_package_share_directory
from caytu_config import load_config
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

CONFIG = load_config()
TOPIC_VITESSE_BRUT = "/fosa/cmd_vel_brut"     # entrée du collision monitor


def _drapeau(context, nom):
    return LaunchConfiguration(nom).perform(context).strip().lower() in ("1", "true", "yes")


def _creer(context):
    dossier_bringup = get_package_share_directory("caytu_nav_bringup")
    robot = LaunchConfiguration("robot").perform(context)
    with open(os.path.join(dossier_bringup, "config", "robots", f"{robot}.yaml"),
              encoding="utf-8") as flux:
        profil = yaml.safe_load(flux)
    temps = LaunchConfiguration("use_sim_time").perform(context).strip().lower()
    if temps not in ("true", "false"):
        temps = "true" if robot in ("sim", "parc") else "false"
    temps_simulation = temps == "true"
    securite = _drapeau(context, "securite")
    topic_robot = (CONFIG.navigation.cmd_vel_topic if robot == "sim"
                   else str(profil["cmd_vel_topic"]))
    topic_commandes = TOPIC_VITESSE_BRUT if securite else topic_robot

    carte = os.path.abspath(os.path.expanduser(
        LaunchConfiguration("carte").perform(context).strip())) if (
        LaunchConfiguration("carte").perform(context).strip()) else ""
    actions = [LogInfo(msg=(
        f"COURSE : robot {robot}, "
        + (f"carte {carte}" if carte else "cartographie en direct")
        + f", garde-fou {'actif' if securite else 'désactivé'}, "
        f"vitesses -> {topic_robot}."))]

    # 1. Navigation
    arguments_navigation = {
        "robot": robot,
        "use_sim_time": temps,
        "controller": LaunchConfiguration("controller"),
        "imu_odom": LaunchConfiguration("imu_odom"),
        "nav_cmd_vel_topic": TOPIC_VITESSE_BRUT if securite else "",
    }
    if carte:
        carte_nav = os.path.join(carte, "nav_map.yaml")
        if not os.path.isfile(carte_nav):
            carte_nav = os.path.join(carte, "map.yaml")
        arguments_navigation.update({
            "localization": "slam", "slam_map_dir": carte, "map": carte_nav})
    else:
        arguments_navigation["localization"] = "slam_live"
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dossier_bringup, "launch", "solution_bringup.launch.py")),
        launch_arguments=arguments_navigation.items(),
    ))

    # 2. AprilTags
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dossier_bringup, "launch", "tags.launch.py")),
        launch_arguments={
            "use_sim_time": temps,
            "fichier_tags": os.path.join(carte, CONFIG.course.tags_file) if carte else "",
            "recharger": "true",
            "detecteur": LaunchConfiguration("detecteur"),
            "image_topic": LaunchConfiguration("image_topic"),
        }.items(),
    ))

    # 3. Garde-fou : tout objet devant le robot l'arrête.
    if securite:
        cameras_garde_fou = LaunchConfiguration("garde_fou_cameras").perform(context).strip().lower()
        cameras_garde_fou = (cameras_garde_fou in ("1", "true", "yes") if cameras_garde_fou
                             else bool(profil.get("collision_cameras", True)))
        cameras_garde_fou = cameras_garde_fou and bool(profil.get("cameras"))
        sources = ["lidar"] + (["camera_haute", "camera_basse"] if cameras_garde_fou else [])
        actions.append(LogInfo(msg=(
            "Garde-fou : " + ("lidar + caméras (une caméra muette plus de 1 s arrête le "
                              "robot ; secours : garde_fou_cameras:=false)."
                              if cameras_garde_fou else "lidar seul (garde_fou_cameras)."))))
        actions += [
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                output="screen",
                parameters=[
                    os.path.join(dossier_bringup, "config", "collision_monitor.yaml"),
                    {
                        "use_sim_time": temps_simulation,
                        "cmd_vel_in_topic": TOPIC_VITESSE_BRUT,
                        "cmd_vel_out_topic": topic_robot,
                        "observation_sources": sources,
                        "state_topic": CONFIG.course.safety_state_topic,
                        "camera_haute.topic": CONFIG.perception.top_scan.scan_topic,
                        "camera_basse.topic": CONFIG.perception.bottom_scan.scan_topic,
                    },
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_securite",
                output="screen",
                parameters=[{"use_sim_time": temps_simulation, "autostart": True,
                             "node_names": ["collision_monitor"], "bond_timeout": 20.0}],
            ),
        ]

    # 4. Démarrage vocal
    if _drapeau(context, "voix"):
        actions.append(Node(
            package="caytu_assistant",
            executable="mot_depart",
            name="mot_depart",
            output="screen",
            parameters=[{"use_sim_time": temps_simulation}],
        ))

    # 5. La course
    actions.append(Node(
        package="caytu_nav_solution",
        executable="course_tags",
        name="course_tags",
        output="screen",
        parameters=[{
            "use_sim_time": temps_simulation,
            "cmd_vel_topic": topic_commandes,
            "image_topic": LaunchConfiguration("image_topic").perform(context)
            or CONFIG.perception.top_color_topic,
            "ordre": ParameterValue(LaunchConfiguration("ordre"), value_type=str),
            "depart_immediat": _drapeau(context, "depart_immediat"),
        }],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot", default_value="sim", choices=["sim", "parc", "real", "real_remote"],
            description="Profil du robot (config/robots/)."),
        DeclareLaunchArgument(
            "use_sim_time", default_value="",
            description="Vide : true pour sim, false pour le robot réel."),
        DeclareLaunchArgument(
            "carte", default_value="",
            description="Dossier de la carte (map.yaml, posegraph, tags.yaml). Vide : "
                        "cartographie en direct pendant la course."),
        DeclareLaunchArgument("controller", default_value=CONFIG.navigation.controller,
                              choices=["rpp", "dwb"]),
        DeclareLaunchArgument("imu_odom", default_value="false"),
        DeclareLaunchArgument(
            "securite", default_value="true",
            description="nav2_collision_monitor entre les commandes et la base."),
        DeclareLaunchArgument(
            "garde_fou_cameras", default_value="", choices=["", "true", "false"],
            description="Caméras dans le garde-fou (vide = collision_cameras du profil "
                        "robot : true sur le vrai robot, false dans le simulateur officiel)."),
        DeclareLaunchArgument(
            "voix", default_value=str(CONFIG.course.voice.enabled).lower(),
            description="Démarrage vocal (Whisper hors ligne, sur le robot)."),
        DeclareLaunchArgument(
            "detecteur", default_value="", choices=["", "opencv", "apriltag_ros"],
            description="Détecteur d'AprilTags (vide = caytu_config)."),
        DeclareLaunchArgument(
            "image_topic", default_value="",
            description="Forçage de l'image couleur (vide = caytu_config)."),
        DeclareLaunchArgument(
            "ordre", default_value="",
            description="ESSAIS SEULEMENT : ordre imposé (ex. 7,1,9,4). En course, "
                        "l'ordre vient du topic des organisateurs."),
        DeclareLaunchArgument(
            "depart_immediat", default_value="false",
            description="ESSAIS SEULEMENT : partir sans attendre le signal."),
        OpaqueFunction(function=_creer),
    ])
