"""Cartographie manuelle : on pilote le robot, slam_toolbox dessine la carte.

Lance :
  1. l'adaptation au robot (robot_interfaces.launch.py : repère au sol,
     filtres du lidar, mêmes réglages que la navigation) ;
  2. slam_toolbox en mode cartographie, sur /scan_clean (config/slam_mapping.yaml) ;
  3. mapping_session : note la pose de départ, sauvegarde automatiquement la
     carte dans <maps_dir>/<map_name>/auto, offre le service /fosa/save_map ;
  4. avec tags:=true (défaut) : la détection des AprilTags et leur
     enregistrement dans <maps_dir>/<map_name>/tags.yaml (tags.launch.py) ;
  5. en option : l'enregistrement des capteurs, RViz, le relais TF du rejeu.

Le pilotage se fait dans un autre terminal (voir docs/CARTOGRAPHIE.md) :
    Gazebo      ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \\
                    -r cmd_vel:=/robot_base_controller/cmd_vel_unstamped -p speed:=0.2 -p turn:=0.6
    Vrai robot  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \\
                    -p speed:=0.2 -p turn:=0.6            (publie sur /cmd_vel)

Le profil du robot (config/robots/) vient de robot:=sim|real ; par défaut,
sim avec gazebo:=true, real sinon.

Exemples :
    Gazebo      ros2 launch caytu_nav_bringup mapping.launch.py gazebo:=true map_name:=cafe_test
    Vrai robot  ros2 launch caytu_nav_bringup mapping.launch.py map_name:=arene_dakar record:=true
    Rejeu       ros2 launch caytu_nav_bringup mapping.launch.py replay:=true robot:=real map_name:=rejeu_1
                ros2 bag play <enregistrement> --clock --remap /tf:=/tf_bag
    Sauvegarde  ros2 run caytu_nav_solution save_map
"""

import datetime
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue



def _flag(context, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).strip().lower() in ("1", "true", "yes", "on")


def _maps_dir(context) -> str:
    value = LaunchConfiguration("maps_dir").perform(context).strip()
    return os.path.expanduser(value or os.environ.get("FOSA_MAPS_DIR", "~/fosa_maps"))


def _slam_params(context, bringup_dir: str, odom_frame: str) -> str:
    """Fichier de réglages slam_toolbox : repère d'odométrie du profil, et
    portée forcée si demandée."""
    params_file = (LaunchConfiguration("slam_params_file").perform(context).strip()
                   or os.path.join(bringup_dir, "config", "slam_mapping.yaml"))
    max_range = LaunchConfiguration("max_laser_range").perform(context).strip()
    with open(params_file, encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    data["slam_toolbox"]["ros__parameters"]["odom_frame"] = odom_frame
    if max_range:
        data["slam_toolbox"]["ros__parameters"]["max_laser_range"] = float(max_range)
    path = os.path.join(tempfile.mkdtemp(prefix="fosa_slam_"), "slam_mapping.yaml")
    with open(path, "w", encoding="utf-8") as stream:
        yaml.safe_dump(data, stream, sort_keys=False)
    return path


def _robot(context, gazebo: bool) -> str:
    robot = LaunchConfiguration("robot").perform(context).strip()
    if robot == "auto":
        return "sim" if gazebo else "real"
    return robot


def _setup(context):
    bringup_dir = get_package_share_directory("caytu_nav_bringup")
    gazebo = _flag(context, "gazebo")
    replay = _flag(context, "replay")
    robot = _robot(context, gazebo)
    # Simulateur officiel (profil parc) : toujours en temps simulé.
    sim_time = "true" if (gazebo or replay or robot == "parc") else "false"
    with open(os.path.join(bringup_dir, "config", "robots", f"{robot}.yaml"),
              encoding="utf-8") as stream:
        profile = yaml.safe_load(stream)
    map_name = LaunchConfiguration("map_name").perform(context).strip()
    maps_dir = _maps_dir(context)
    imu_odom = _flag(context, "imu_odom")
    odom_frame = "fosa_odom" if imu_odom and profile.get("imu_topic") else "odom"
    params_file = _slam_params(context, bringup_dir, odom_frame)

    actions = [LogInfo(msg=(
        f"Cartographie « {map_name} » ({'Gazebo' if gazebo else 'rejeu' if replay else 'robot réel'}, "
        f"profil {robot}, use_sim_time={sim_time}) ; réglages slam_toolbox : {params_file}"))]

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "robot_interfaces.launch.py")),
        launch_arguments={
            "robot": robot,
            "use_sim_time": sim_time,
            "floor_line_distance": LaunchConfiguration("floor_line_distance"),
        }.items(),
    ))

    # Option imu_odom : odométrie corrigée par l'IMU pour slam_toolbox.
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "slam_odometry.launch.py")),
        launch_arguments={"robot": robot, "use_sim_time": sim_time,
                          "imu_odom": "true" if imu_odom else "false"}.items(),
    ))

    # Lancement officiel de slam_toolbox : il gère le cycle de vie du nœud
    # (configure puis active) quelle que soit sa version.
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")),
        launch_arguments={
            "slam_params_file": params_file,
            "use_sim_time": sim_time,
            "autostart": "true",
            "use_lifecycle_manager": "false",
        }.items(),
    ))

    actions.append(Node(
        package="caytu_nav_solution",
        executable="mapping_session",
        name="mapping_session",
        output="screen",
        parameters=[{
            "use_sim_time": sim_time == "true",
            # value_type=str : un nom comme « 2026 » ou un chemin vide doivent
            # rester des chaînes.
            "map_name": ParameterValue(map_name, value_type=str),
            "maps_dir": ParameterValue(maps_dir, value_type=str),
            "autosave_period_sec": float(LaunchConfiguration("autosave_sec").perform(context)),
            "record_world_frame": gazebo,
            "task_params_file": ParameterValue(
                LaunchConfiguration("task_params_file"), value_type=str),
        }],
    ))

    # Position des AprilTags, dans le repère de cette carte.
    if _flag(context, "tags"):
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, "launch", "tags.launch.py")),
            launch_arguments={
                "use_sim_time": sim_time,
                "fichier_tags": os.path.join(maps_dir, map_name, "tags.yaml"),
                "recharger": "false",
                "detecteur": LaunchConfiguration("detecteur"),
                "image_topic": LaunchConfiguration("image_topic"),
            }.items(),
        ))

    if replay:
        actions.append(Node(
            package="caytu_nav_solution",
            executable="tf_bag_relay",
            name="tf_bag_relay",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ))

    if _flag(context, "record") and not replay:
        stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        bags_dir = os.path.join(maps_dir, "enregistrements")
        os.makedirs(bags_dir, exist_ok=True)
        # Capteurs pour recartographier sans le robot ; image de la caméra
        # haute en option (lourd) pour mettre au point les AprilTag.
        topics = list(profile["record_topics"]) + (
            list(profile["record_camera_topics"]) if _flag(context, "record_camera") else [])
        output = os.path.join(bags_dir, f"{map_name}_{stamp}")
        actions.append(LogInfo(msg=f"Enregistrement des capteurs dans {output}"))
        cmd = ["ros2", "bag", "record", "-o", output, "-s", "mcap",
               "--storage-preset-profile", "zstd_fast"]
        if sim_time == "true":
            cmd.append("--use-sim-time")      # Gazebo : horodatage en temps simulé
        actions.append(ExecuteProcess(cmd=cmd + ["--topics"] + topics, output="screen"))

    if _flag(context, "rviz"):
        actions.append(Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_mapping",
            output="log",
            arguments=["-d", os.path.join(bringup_dir, "rviz", "mapping.rviz")],
            parameters=[{"use_sim_time": sim_time == "true"}],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "map_name", default_value="carte",
            description="Nom de la carte = nom du dossier dans maps_dir. Une carte "
                        "existante du même nom est déplacée, jamais écrasée."),
        DeclareLaunchArgument(
            "maps_dir", default_value="",
            description="Dossier des cartes (vide = $FOSA_MAPS_DIR ou ~/fosa_maps)."),
        DeclareLaunchArgument(
            "gazebo", default_value="false",
            description="true dans Gazebo : temps simulé, et lien map <-> monde noté "
                        "depuis task_params.yaml (pour viser le cercle vert)."),
        DeclareLaunchArgument(
            "robot", default_value="auto", choices=["auto", "sim", "parc", "real", "real_remote"],
            description="Profil du robot (config/robots/). auto : sim avec gazebo:=true, "
                        "real sinon."),
        DeclareLaunchArgument(
            "replay", default_value="false",
            description="true pour recartographier depuis un enregistrement "
                        "(ros2 bag play ... --clock --remap /tf:=/tf_bag)."),
        DeclareLaunchArgument(
            "record", default_value="false",
            description="Enregistrer lidar, odométrie, IMU et TF (topics du profil)."),
        DeclareLaunchArgument(
            "record_camera", default_value="false",
            description="Avec record:=true, ajouter l'image de la caméra haute (lourd)."),
        DeclareLaunchArgument(
            "imu_odom", default_value="false",
            description="Essai : slam_toolbox sur l'odométrie corrigée par l'IMU "
                        "(Gazebo ; ignoré sans IMU de base)."),
        DeclareLaunchArgument(
            "autosave_sec", default_value="30.0",
            description="Période de la sauvegarde automatique dans <carte>/auto (0 = jamais)."),
        DeclareLaunchArgument(
            "slam_params_file", default_value="",
            description="Réglages slam_toolbox (vide = config/slam_mapping.yaml)."),
        DeclareLaunchArgument(
            "max_laser_range", default_value="",
            description="Forçage de la portée utilisée par slam_toolbox (m)."),
        DeclareLaunchArgument(
            "floor_line_distance", default_value="0.0",
            description="Forçage de la ligne de sol du lidar (voir robot_interfaces.launch.py)."),
        DeclareLaunchArgument(
            "task_params_file", default_value="",
            description="Gazebo : task_params.yaml à lire (vide = package officiel)."),
        DeclareLaunchArgument(
            "rviz", default_value="false",
            description="Ouvrir RViz avec la carte, le scan et le robot."),
        DeclareLaunchArgument(
            "tags", default_value="true",
            description="Détecter les AprilTags et noter leur position dans "
                        "<carte>/tags.yaml pendant la cartographie."),
        DeclareLaunchArgument(
            "detecteur", default_value="", choices=["", "opencv", "apriltag_ros"],
            description="Détecteur d'AprilTags (vide = caytu_config)."),
        DeclareLaunchArgument(
            "image_topic", default_value="",
            description="Forçage de l'image couleur pour les tags (vide = caytu_config)."),
        OpaqueFunction(function=_setup),
    ])
