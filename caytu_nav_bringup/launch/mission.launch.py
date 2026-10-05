# mission.launch.py — TOUT le stack PARC 2026 en UNE commande.
#
# Un "launch file" est un script Python qui decrit QUOI lancer et COMMENT :
# quels noeuds, avec quels parametres/remaps, dans quel ordre et avec quels
# delais. `ros2 launch` l'execute : plus de 6 terminaux a gerer a la main.
#
# Usage (UN terminal, domaine isole) :
#   export ROS_DOMAIN_ID=42
#   ros2 launch caytu_nav_bringup mission.launch.py
#
# Options :
#   perception:=false      ne pas lancer la perception camera
#   monitor:=false         ne pas lancer nav_monitor
#   set_initial_pose:=false  ne pas initialiser AMCL (faire 2D Pose Estimate a la main)
#   Goal officiel envoye automatiquement a +60 s (auto_goal:=false pour desactiver)
#   rviz seul : task.launch.py lance toujours RViz (fourni par parc_robot_bringup).
#
# Ordre interne : gazebo+bridges -> filtre -> map/amcl (autostart) ->
# Nav2 (delai 15 s, le temps du spawn) -> perception/monitor/watchdog.

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory("caytu_nav_bringup")
    parc_dir = get_package_share_directory("parc_robot_bringup")
    perception_dir = get_package_share_directory("caytu_camera_perception")

    use_sim_time = LaunchConfiguration("use_sim_time")
    perception = LaunchConfiguration("perception")
    monitor = LaunchConfiguration("monitor")
    set_initial_pose = LaunchConfiguration("set_initial_pose")
    auto_goal = LaunchConfiguration("auto_goal")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")

    # 1. Gazebo + spawn robot/goal + bridges + RViz (parc_robot_bringup).
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(parc_dir, "launch", "task.launch.py")]
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # 2. Filtre LiDAR (coupe le chassis vu par le lidar a z=0).
    lidar_filter = Node(
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

    # 3. Map + AMCL, actives automatiquement par leur lifecycle manager.
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "yaml_filename": os.path.join(bringup_dir, "maps", "stadium_map.yaml"),
                "use_sim_time": use_sim_time,
            }
        ],
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
                "set_initial_pose": set_initial_pose,
                "initial_pose.x": initial_x,
                "initial_pose.y": initial_y,
                "initial_pose.yaw": initial_yaw,
            },
        ],
    )

    lifecycle_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["map_server", "amcl"],
            }
        ],
    )

    # 4. Pile Nav2 (planner + controller + BT, cmd_vel direct sans surcouche).
    nav2 = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(bringup_dir, "launch", "navigation.launch.py")]
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            )
        ],
    )

    # 5. Perception camera (optionnelle).
    perception_launch = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(perception_dir, "launch", "perception.launch.py")]
                ),
                condition=IfCondition(perception),
            )
        ],
    )

    # 6. Monitoring + watchdog (logs temps reel + /localization_ready).
    nav_monitor = TimerAction(
        period=20.0,
        actions=[
            Node(
                package="caytu_nav_solution",
                executable="nav_monitor",
                name="nav_monitor",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(monitor),
            )
        ],
    )

    watchdog = TimerAction(
        period=20.0,
        actions=[
            Node(
                package="caytu_nav_solution",
                executable="localization_watchdog",
                name="localization_watchdog",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ],
    )

    # 7. Goal automatique (optionnel, 60 s : laisse AMCL converger).
    goal = TimerAction(
        period=60.0,
        actions=[
            Node(
                package="caytu_nav_solution",
                executable="task_solution",
                name="task_solution",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time, "use_test_goal": False}
                ],
                condition=IfCondition(auto_goal),
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("perception", default_value="true"),
            DeclareLaunchArgument("monitor", default_value="true"),
            DeclareLaunchArgument("set_initial_pose", default_value="true"),
            DeclareLaunchArgument("auto_goal", default_value="true"),
            # Spawn de task_params.yaml : x=-0.200546, y=-7.485170, yaw=1.571.
            DeclareLaunchArgument("initial_x", default_value="-0.200546"),
            DeclareLaunchArgument("initial_y", default_value="-7.485170"),
            DeclareLaunchArgument("initial_yaw", default_value="1.571"),
            gazebo,
            lidar_filter,
            map_server,
            amcl,
            lifecycle_localization,
            nav2,
            perception_launch,
            nav_monitor,
            watchdog,
            goal,
        ]
    )
