import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # PACKAGE

    bringup_dir = get_package_share_directory("caytu_nav_bringup")

    params_file = os.path.join(
        bringup_dir,
        "config",
        "nav2_params.yaml",
    )

    # LAUNCH ARGUMENTS

    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Utiliser l'horloge Gazebo",
    )

    # CONTROLLER SERVER

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",

        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
            },
        ],

        remappings=[
            (
                "cmd_vel",
                "/robot_base_controller/cmd_vel_unstamped",
            ),
        ],
    )

    # PLANNER SERVER

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",

        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    # BEHAVIOR SERVER

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",

        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
            },
        ],

        remappings=[
            (
                "cmd_vel",
                "/robot_base_controller/cmd_vel_unstamped",
            ),
        ],
    )

    # BT NAVIGATOR

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",

        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    # LIFECYCLE MANAGER

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",

        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "autostart": True,

                "node_names": [
                    "controller_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                ],
            },
        ],
    )

    # LAUNCH DESCRIPTION

    return LaunchDescription(
        [

            declare_use_sim_time,

            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            lifecycle_manager,

        ]
    )