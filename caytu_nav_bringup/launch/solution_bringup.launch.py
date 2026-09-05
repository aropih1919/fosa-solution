"""Bringup autonome de la navigation PARC.

Ce launch remplace le démarrage manuel en plusieurs terminaux : il garantit que le
filtre LiDAR, la carte et AMCL existent avant l'activation de la pile Nav2.
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("caytu_nav_bringup")
    task_bringup_dir = get_package_share_directory("parc_robot_bringup")

    # task.launch.py utilise ce même fichier pour spawner le robot. Initialiser
    # AMCL avec ces coordonnées évite qu'un clic RViz approximatif fasse calculer
    # un plan depuis une position fausse (le goal reste, lui aussi, dans map).
    task_params_path = os.path.join(task_bringup_dir, "config", "task_params.yaml")
    with open(task_params_path, encoding="utf-8") as task_params_file:
        task_params = yaml.safe_load(task_params_file)["/**"]["ros__parameters"]

    initial_pose = {
        "x": float(task_params["x"]),
        "y": float(task_params["y"]),
        "z": float(task_params["z"]),
        "yaw": float(task_params["yaw"]),
    }

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Utiliser /clock de Gazebo pour tous les nœuds de navigation.",
    )
    declare_map = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(bringup_dir, "maps", "stadium_map.yaml"),
        description="Fichier YAML de carte ; valeur résolue depuis le package installé.",
    )

    laser_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="scan_to_scan_filter_chain",
        output="screen",
        parameters=[
            os.path.join(bringup_dir, "config", "laser_filter_params.yaml"),
            {"use_sim_time": use_sim_time},
        ],
        # Le robot publie /scan ; Nav2 et AMCL consomment exclusivement le scan filtré.
        remappings=[("scan", "/scan"), ("scan_filtered", "/scan_filtered")],
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_yaml, "use_sim_time": use_sim_time}],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            os.path.join(bringup_dir, "config", "amcl_params.yaml"),
            {
                "use_sim_time": use_sim_time,
                # Surcharge dynamique : aucun couple x/y propre à une machine ou
                # à une tâche n'est figé dans le YAML de configuration AMCL.
                "set_initial_pose": True,
                "initial_pose": initial_pose,
            },
        ],
    )

    localization_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                # map_server est activé avant AMCL : AMCL peut alors souscrire à /map.
                "node_names": ["map_server", "amcl"],
            }
        ],
    )

    localization_watchdog = Node(
        package="caytu_nav_solution",
        executable="localization_watchdog",
        name="localization_watchdog",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(bringup_dir, "launch", "navigation.launch.py")]
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # Laisser le lifecycle manager localiser/configurer la carte et AMCL avant
    # que les costmaps Nav2 ne demandent map -> odom. Le task_solution attendra
    # toujours /localization_ready avant d'envoyer un goal.
    delayed_navigation = TimerAction(period=3.0, actions=[navigation])

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_map,
            laser_filter,
            map_server,
            amcl,
            localization_manager,
            localization_watchdog,
            delayed_navigation,
        ]
    )
