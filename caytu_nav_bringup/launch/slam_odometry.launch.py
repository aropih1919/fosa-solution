"""Option imu_odom:=true : odométrie corrigée par l'IMU pour slam_toolbox.

Par défaut, slam_toolbox lit l'odométrie des roues (repère odom). Dans Gazebo,
son cap est faux d'environ 15 % en rotation (entraxe déclaré 0,341 m pour
0,393 m réels). slam_toolbox l'a rattrapé dans nos essais, mais l'option permet
de le vérifier : localisateur_odom_imu, en mode correcteur, publie fosa_odom ->
odom, de sorte que fosa_odom -> base_footprint soit l'odométrie corrigée
(distance des roues, cap de l'IMU, comme la localisation de la phase 1).
slam_toolbox prend alors fosa_odom comme repère d'odométrie. L'arbre officiel
(odom -> base_footprint) n'est pas touché.

Sans IMU de base (robot réel), l'option est ignorée.

Inclus par mapping.launch.py et solution_bringup.launch.py (localization:=slam).
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


IMU_ODOM_FRAME = "fosa_odom"


def slam_odom_frame(profile: dict, imu_odom: bool) -> str:
    """Repère d'odométrie à donner à slam_toolbox (odom = odométrie des roues)."""
    return IMU_ODOM_FRAME if imu_odom and profile.get("imu_topic") else "odom"


def _setup(context):
    robot = LaunchConfiguration("robot").perform(context)
    with open(os.path.join(get_package_share_directory("caytu_nav_bringup"),
                           "config", "robots", f"{robot}.yaml"), encoding="utf-8") as stream:
        profile = yaml.safe_load(stream)
    imu_odom = LaunchConfiguration("imu_odom").perform(context).lower() == "true"
    frame = slam_odom_frame(profile, imu_odom)
    if frame == "odom":
        if imu_odom:
            return [LogInfo(msg="imu_odom ignoré : pas d'IMU de base dans ce profil.")]
        return [LogInfo(msg="slam_toolbox : odométrie des roues (odom).")]
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    return [
        LogInfo(msg=f"slam_toolbox : odométrie corrigée par l'IMU ({frame} -> odom)."),
        Node(
            package="caytu_nav_solution",
            executable="localisateur_odom_imu",
            name="correcteur_odom_imu",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "map_frame": ParameterValue(frame, value_type=str),
                "odom_frame": "odom",
                "odom_topic": ParameterValue(profile["odom_topic"], value_type=str),
                "imu_topic": ParameterValue(profile["imu_topic"], value_type=str),
                "use_task_spawn": False,
                "pose_topic": ParameterValue("", value_type=str),
                # Pas de post-datation : slam_toolbox attend la TF à l'heure
                # exacte de chaque scan, la correction ne doit pas être en retard.
                "transform_tolerance": 0.0,
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="sim", choices=["sim", "parc", "real", "real_remote"]),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("imu_odom", default_value="false"),
        OpaqueFunction(function=_setup),
    ])
