"""Détection et enregistrement des AprilTags (cartographie et course).

Lance :
  - le détecteur choisi par perception.apriltag.detector (caytu_config), ou
    par detecteur:=opencv|apriltag_ros :
      opencv        tag_detector de caytu_perception (OpenCV, solvePnP) ;
      apriltag_ros  apriltag_node du paquet ROS apriltag_ros (pose en TF) ;
  - registre_tags : position filtrée de chaque tag dans `map`, enregistrée
    dans fichier_tags (tags.yaml du dossier de la carte).

Les topics de la caméra viennent de caytu_config (perception.top_color_topic,
top_info_topic) ; robot réel : FOSA_CONFIG_OVERRIDE=.../robot_reel.yaml.
Inclus par mapping.launch.py (tags:=true) et course.launch.py.
"""

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from caytu_config import load_config
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

CONFIG = load_config()


def _drapeau(context, nom):
    return LaunchConfiguration(nom).perform(context).strip().lower() in ("1", "true", "yes")


def _creer_noeuds(context):
    apriltag = CONFIG.perception.apriltag
    detecteur = LaunchConfiguration("detecteur").perform(context) or apriltag.detector
    temps_simulation = _drapeau(context, "use_sim_time")
    image = LaunchConfiguration("image_topic").perform(context) or CONFIG.perception.top_color_topic
    info = (LaunchConfiguration("camera_info_topic").perform(context)
            or CONFIG.perception.top_info_topic)
    actions = [LogInfo(msg=f"AprilTags : détecteur {detecteur}, image {image}.")]

    if detecteur == "apriltag_ros":
        # Sans détecteur, aucune visite ne serait déclarée : on s'arrête tout de
        # suite, avec la marche à suivre, plutôt que de partir à l'aveugle.
        for paquet in ("apriltag_ros", "apriltag_msgs"):
            try:
                get_package_share_directory(paquet)
            except PackageNotFoundError as erreur:
                raise RuntimeError(
                    f"{paquet} absent : sudo apt install ros-jazzy-apriltag-ros "
                    "(robot : bash $FOSA_ROOT/installer_robot.sh, ou les organisateurs). "
                    "En secours seulement : detecteur:=opencv.") from erreur
        actions.append(Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name="apriltag",
            output="screen",
            remappings=[("image_rect", image), ("camera_info", info)],
            parameters=[{
                "use_sim_time": temps_simulation,
                "family": apriltag.family,
                "size": apriltag.size_m,
                "max_hamming": 0,
                "pose_estimation_method": "pnp",
                "detector.threads": 2,
                "detector.decimate": 1.0,
            }],
        ))
    else:
        actions.append(Node(
            package="caytu_perception",
            executable="tag_detector",
            name="tag_detector",
            output="screen",
            parameters=[{
                "use_sim_time": temps_simulation,
                "publish_map_pose": True,
                "image_topic": image,
                "camera_info_topic": info,
                "annotated_directory": ParameterValue(
                    LaunchConfiguration("images_annotees"), value_type=str),
            }],
        ))

    actions.append(Node(
        package="caytu_perception",
        executable="registre_tags",
        name="registre_tags",
        output="screen",
        parameters=[{
            "use_sim_time": temps_simulation,
            "detector": detecteur,
            "fichier_tags": ParameterValue(
                LaunchConfiguration("fichier_tags"), value_type=str),
            "recharger": _drapeau(context, "recharger"),
        }],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "detecteur", default_value="", choices=["", "opencv", "apriltag_ros"],
            description="Vide = perception.apriltag.detector de caytu_config."),
        DeclareLaunchArgument(
            "fichier_tags", default_value="",
            description="tags.yaml du dossier de la carte (vide = rien n'est écrit)."),
        DeclareLaunchArgument(
            "recharger", default_value="true",
            description="Relire fichier_tags au démarrage (false en cartographie : "
                        "nouvelle carte, nouveau repère)."),
        DeclareLaunchArgument(
            "image_topic", default_value="",
            description="Forçage de l'image couleur (vide = caytu_config). Portable : "
                        "<image>/compressed pour soulager le Wi-Fi (détecteur opencv)."),
        DeclareLaunchArgument("camera_info_topic", default_value=""),
        DeclareLaunchArgument(
            "images_annotees", default_value="",
            description="Dossier où garder des images annotées (vide = aucune)."),
        OpaqueFunction(function=_creer_noeuds),
    ])
