"""Bringup complet de la solution Fosa (lancé par task_solution.py).

Démarre, sans aucun délai ni intervention manuelle :
  1-2. l'adaptation au robot (robot_interfaces.launch.py) : repère au sol
     (robot réel), filtre d'auto-détection du lidar (/scan -> /fosa/scan_filtered),
     mise à niveau (inclinaison du robot, retours du sol du lidar, repère
     horizontal base_footprint_level pour les caméras) ;
  3. la conversion des deux caméras de profondeur en LaserScan (robot réel :
     images de profondeur -> nuages avec depth_image_proc, s'il est installé ;
     profil real_remote : aucune caméra, navigation au lidar seul) ;
  4. la localisation, qui publie map -> odom :
       odom_imu / odom_only  localisateur roues (+ IMU) depuis le départ ;
       amcl                  AMCL, démarré par navigation.launch.py ;
       slam                  slam_toolbox sur une carte enregistrée par
                             mapping.launch.py (slam_map_dir:=<dossier>) ;
       slam_live             slam_toolbox cartographie en direct (pas de carte
                             préalable : Nav2 lit la carte en construction) ;
  5. Nav2 (carte des murs, planificateur, contrôleur, behaviors, BT), dès que la
     localisation publie la position (wait_for_tf). Avec AMCL, Nav2 démarre
     tout de suite : c'est lui qui lance AMCL.

Le profil robot:=sim|real|real_remote (config/robots/) donne les topics du
robot. En simulation, les topics et réglages restent ceux de caytu_config.

Utilisation directe (mise au point) :
    ros2 launch caytu_nav_bringup solution_bringup.launch.py
    ros2 launch caytu_nav_bringup solution_bringup.launch.py robot:=real \\
        use_sim_time:=false localization:=slam \\
        slam_map_dir:=$FOSA_MAPS_DIR/arene_dakar \\
        map:=$FOSA_MAPS_DIR/arene_dakar/map.yaml
"""

import os
import tempfile

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from caytu_config import load_config
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
import yaml

CONFIG = load_config()

# Repère horizontal publié par le filtre sol lidar (le robot penche de ~2,5°).
REPERE_HORIZONTAL = CONFIG.perception.top_scan.target_frame

ROBOTS = ["sim", "parc", "real", "real_remote"]


def _valider_arguments_essai(context):
    """Refuse une combinaison benchmark qui exige un YAML complet absent."""
    fichier_parametres = LaunchConfiguration("params_file").perform(context)
    controleur = LaunchConfiguration("controller").perform(context)
    localisation = LaunchConfiguration("localization").perform(context)
    profil = LaunchConfiguration("profile").perform(context)
    if not fichier_parametres and (
            controleur == "mppi" or profil != "balanced"
            or localisation == "amcl"):
        raise RuntimeError(
            "Cette combinaison exige un YAML complet params_file généré par "
            "task_solution.py ou fosa_benchmark.")
    return []


def _charger_profil(context):
    """Profil du robot : config/robots/<robot>.yaml."""
    robot = LaunchConfiguration("robot").perform(context)
    chemin = os.path.join(
        get_package_share_directory("caytu_nav_bringup"),
        "config", "robots", f"{robot}.yaml")
    if not os.path.isfile(chemin):
        raise RuntimeError(f"Profil de robot inconnu : {robot} ({chemin} absent)")
    with open(chemin, encoding="utf-8") as flux:
        return yaml.safe_load(flux)


def _temps_simulation(context):
    return LaunchConfiguration("use_sim_time").perform(context).lower() == "true"


def _topic_vitesse(context, profil):
    """Forçage (nav_cmd_vel_topic, ex. entrée du collision monitor), sinon
    fosa.yaml en simulation, sinon le profil du robot réel."""
    force = LaunchConfiguration("nav_cmd_vel_topic").perform(context).strip()
    if force:
        return force
    if LaunchConfiguration("robot").perform(context) == "sim":
        return CONFIG.navigation.cmd_vel_topic
    return str(profil["cmd_vel_topic"])


def _declarer_arguments(dossier_bringup):
    """Réglages modifiables en ligne de commande."""
    return [
        DeclareLaunchArgument(
            "robot", default_value="sim", choices=ROBOTS,
            description="sim : Gazebo ; real : robot réel, solution sur le robot ; "
                        "real_remote : robot réel, solution sur le portable, sans "
                        "caméras (config/robots/)."),
        DeclareLaunchArgument(
            "use_sim_time", default_value=str(CONFIG.system.use_sim_time).lower(),
            description="Utiliser /clock de Gazebo pour tous les nœuds "
                        "(false sur le robot réel)."),
        DeclareLaunchArgument(
            "map", default_value=CONFIG.navigation.map_yaml or os.path.join(
                dossier_bringup, CONFIG.navigation.default_map),
            description="Carte statique (repère Gazebo, ou carte enregistrée). "
                        "task_solution.py la remplace par une carte vide si le "
                        "monde n'est pas le café."),
        DeclareLaunchArgument(
            "controller", default_value=CONFIG.navigation.controller,
            choices=["rpp", "dwb", "mppi"],
            description="Contrôleur local sélectionné dans params_file."),
        DeclareLaunchArgument(
            "localization", default_value=CONFIG.navigation.localization,
            choices=["odom_imu", "odom_only", "amcl", "slam", "slam_live"],
            description="Localisation : une seule source map -> odom. slam : "
                        "slam_toolbox sur la carte de slam_map_dir ; slam_live : "
                        "cartographie en direct."),
        DeclareLaunchArgument(
            "profile", default_value=CONFIG.navigation.profile,
            choices=[
                "ultra_safe", "safe", "balanced", "fast", "ultra_fast",
                "tight_clearance"],
            description="Profil chargé dans params_file."),
        DeclareLaunchArgument(
            "params_file", default_value="",
            description="YAML fusionné complet généré par task_solution/fosa_benchmark."),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "task_params_file", default_value="",
            description="task_params.yaml à utiliser (vide = package officiel)."),
        DeclareLaunchArgument(
            "top_cloud_topic", default_value="",
            description="Forçage du nuage de la caméra haute (vide = profil du "
                        "robot, puis caytu_config)."),
        DeclareLaunchArgument(
            "bottom_cloud_topic", default_value="",
            description="Forçage du nuage de la caméra basse (vide = profil du "
                        "robot, puis caytu_config)."),
        DeclareLaunchArgument(
            "floor_line_distance",
            default_value=str(CONFIG.navigation.floor_line_distance),
            description="Forçage : distance (m) de la ligne de sol lue droit "
                        "devant dans /scan. 0.0 = calcul automatique par l'IMU."),
        DeclareLaunchArgument(
            "slam_map_dir", default_value="",
            description="localization:=slam : dossier de la carte (fosa_map.yaml)."),
        DeclareLaunchArgument(
            "nav_cmd_vel_topic", default_value="",
            description="Topic où Nav2 publie ses vitesses (vide = celui du robot). "
                        "course.launch.py y met l'entrée du collision monitor."),
        DeclareLaunchArgument(
            "imu_odom", default_value="false",
            description="Essai, avec localization:=slam : odométrie corrigée par "
                        "l'IMU (Gazebo ; ignoré sans IMU de base)."),
    ]


# --------------------------------------------------------------------------- #
# 1-2. Adaptation au robot (partagée avec la cartographie)
# --------------------------------------------------------------------------- #
def _creer_interfaces_robot(dossier_bringup):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dossier_bringup, "launch", "robot_interfaces.launch.py")),
        launch_arguments={
            "robot": LaunchConfiguration("robot"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "floor_line_distance": LaunchConfiguration("floor_line_distance"),
        }.items(),
    )


# --------------------------------------------------------------------------- #
# 3. Caméras
# --------------------------------------------------------------------------- #
def _parametres_scan(scan, utiliser_temps_simulation):
    """Paramètres pointcloud_to_laserscan lus dans caytu_config."""
    return {
        "use_sim_time": utiliser_temps_simulation,
        "target_frame": scan.target_frame,
        "transform_tolerance": scan.transform_tolerance,
        "min_height": scan.min_height,
        "max_height": scan.max_height,
        "angle_min": scan.angle_min,
        "angle_max": scan.angle_max,
        "angle_increment": scan.angle_increment,
        "scan_time": scan.scan_time,
        "range_min": scan.range_min,
        "range_max": scan.range_max,
        "use_inf": scan.use_inf,
        "inf_epsilon": scan.inf_epsilon,
    }


def _creer_cameras(context):
    """3a. Caméra haute (horizontale, 1.01 m) : tables, chaises, personnes.

    Gazebo ajoute son bruit de profondeur (écart-type 0.10 m) directement sur
    x, y ET z de chaque point : le sol est donc bruité de ±0.10 m en hauteur.
    Dans le repère horizontal, min_height = 0.60 m représente 6 écarts-types
    (aucun faux obstacle dans nos simulations du capteur) et reste sous les
    plateaux de table (0.74 m) et les dossiers de chaise.

    3b. Caméra basse (0.58 m, inclinée de 60° vers le sol) : obstacles bas et
    proches. Son bruit vertical est faible (±0.04 m) : 0.25 m suffit, ce que
    confirment nos journaux (0 point sur sol dégagé).
    """
    profil = _charger_profil(context)
    temps_simulation = _temps_simulation(context)
    cameras = profil.get("cameras") or {}
    if not cameras:
        # Profil real_remote : les images de profondeur ne passent pas par le
        # Wi-Fi ; Nav2 navigue au lidar seul (les couches caméra restent vides).
        return [LogInfo(msg="Caméras désactivées par le profil : navigation au lidar seul.")]

    haute = cameras.get("top") or {}
    basse = cameras.get("bottom") or {}
    actions = []

    # Robot réel : les caméras publient des images de profondeur ; on en tire
    # des nuages de points (depth_image_proc, composant C++).
    sources_profondeur = [(nom, camera) for nom, camera in (("haute", haute), ("basse", basse))
                          if camera.get("depth_image")]
    if sources_profondeur:
        try:
            get_package_share_directory("depth_image_proc")
        except PackageNotFoundError:
            return [LogInfo(msg=(
                "depth_image_proc absent : caméras ignorées, navigation au lidar seul "
                "(sudo apt install ros-jazzy-depth-image-proc)."))]
        for nom, camera in sources_profondeur:
            image = camera["depth_image"]
            actions.append(Node(
                package="depth_image_proc",
                executable="point_cloud_xyz_node",
                name=f"camera_{nom}_profondeur_vers_nuage",
                output="log",
                remappings=[
                    ("image_rect", image),
                    ("camera_info", image.rsplit("/", 1)[0] + "/camera_info"),
                    ("points", camera["cloud"]),
                ],
                parameters=[{"use_sim_time": temps_simulation}],
            ))

    for nom, camera, scan, argument in (
            ("haute", haute, CONFIG.perception.top_scan, "top_cloud_topic"),
            ("basse", basse, CONFIG.perception.bottom_scan, "bottom_cloud_topic")):
        nuage = (LaunchConfiguration(argument).perform(context)
                 or camera.get("cloud") or scan.cloud_topic)
        actions.append(Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name=f"camera_{nom}_vers_scan",
            output="screen",
            remappings=[("cloud_in", nuage), ("scan", scan.scan_topic)],
            parameters=[_parametres_scan(scan, temps_simulation)],
        ))
    return actions


# --------------------------------------------------------------------------- #
# 4-5. Localisation puis Nav2
# --------------------------------------------------------------------------- #
def _creer_navigation(dossier_bringup, topic_vitesse, topic_odom):
    """5. Nav2 (navigation.launch.py)."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dossier_bringup, "launch", "navigation.launch.py")),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "map": LaunchConfiguration("map"),
            "controller": LaunchConfiguration("controller"),
            "localization": LaunchConfiguration("localization"),
            "profile": LaunchConfiguration("profile"),
            "params_file": LaunchConfiguration("params_file"),
            "initial_pose_x": LaunchConfiguration("initial_pose_x"),
            "initial_pose_y": LaunchConfiguration("initial_pose_y"),
            "initial_pose_yaw": LaunchConfiguration("initial_pose_yaw"),
            "cmd_vel_topic": topic_vitesse,
            # Odométrie du robot pour le contrôleur (vitesse mesurée) : sans
            # elle, RPP limite la rotation sur place à max_angular_accel x dt
            # (0,08 rad/s) et le progress checker déclenche le recovery.
            "odom_topic": topic_odom,
        }.items(),
    )


def _creer_localisateur(context, profil):
    """4. Localisation roues + IMU (odom_imu) ou roues seules (odom_only)."""
    localisation = LaunchConfiguration("localization").perform(context)
    fichier_parametres = LaunchConfiguration("params_file").perform(context)
    parametres = [fichier_parametres] if fichier_parametres else []
    parametres.append({
        "use_sim_time": ParameterValue(
            LaunchConfiguration("use_sim_time"), value_type=bool),
        # value_type=str : une valeur vide doit rester une chaîne vide.
        "task_params_file": ParameterValue(
            LaunchConfiguration("task_params_file"), value_type=str),
        "use_imu_yaw": localisation == "odom_imu",
        # Topics du robot (robot réel : /botwheel_explorer/odom, pas d'IMU).
        "odom_topic": ParameterValue(profil["odom_topic"], value_type=str),
        "imu_topic": ParameterValue(profil["imu_topic"], value_type=str),
    })
    return [Node(
        package="caytu_nav_solution",
        executable="localisateur_odom_imu",
        name="localisateur_odom_imu",
        output="screen",
        parameters=parametres,
    )]


def _valeurs_pose_depart(meta):
    """Pose de départ notée pendant la cartographie, en flottants (slam_toolbox
    refuse une liste d'entiers)."""
    pose = meta.get("start_pose") or [0.0, 0.0, 0.0]
    return [float(valeur) for valeur in pose]


def _creer_localisation_slam(context, profil, dossier_bringup):
    """4. slam_toolbox se localise sur le graphe enregistré, en partant de la
    pose notée au début de la cartographie : le robot doit repartir de là."""
    temps_simulation = _temps_simulation(context)
    dossier = os.path.abspath(os.path.expanduser(
        LaunchConfiguration("slam_map_dir").perform(context)))
    chemin_meta = os.path.join(dossier, "fosa_map.yaml")
    if not os.path.isfile(chemin_meta):
        raise RuntimeError(f"localization:=slam : {chemin_meta} introuvable")
    with open(chemin_meta, encoding="utf-8") as flux:
        meta = yaml.safe_load(flux) or {}
    prefixe = os.path.join(dossier, str(meta.get("posegraph", "map")))
    for extension in (".posegraph", ".data"):
        if not os.path.isfile(prefixe + extension):
            raise RuntimeError(
                f"localization:=slam : graphe {prefixe + extension} introuvable")

    with open(os.path.join(dossier_bringup, "config", "slam_localization.yaml"),
              encoding="utf-8") as flux:
        parametres = yaml.safe_load(flux)
    parametres_ros = parametres["slam_toolbox"]["ros__parameters"]
    parametres_ros["map_file_name"] = prefixe
    parametres_ros["map_start_pose"] = _valeurs_pose_depart(meta)
    imu_odom = LaunchConfiguration("imu_odom").perform(context).lower() == "true"
    parametres_ros["odom_frame"] = (
        "fosa_odom" if imu_odom and profil.get("imu_topic") else "odom")
    fichier_slam = os.path.join(
        tempfile.mkdtemp(prefix="fosa_slam_"), "slam_localization.yaml")
    with open(fichier_slam, "w", encoding="utf-8") as flux:
        yaml.safe_dump(parametres, flux, sort_keys=False)

    texte_temps = "true" if temps_simulation else "false"
    # Option imu_odom : odométrie corrigée par l'IMU pour slam_toolbox.
    odometrie_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dossier_bringup, "launch", "slam_odometry.launch.py")),
        launch_arguments={
            "robot": LaunchConfiguration("robot"),
            "use_sim_time": texte_temps,
            "imu_odom": "true" if imu_odom else "false",
        }.items(),
    )

    # Nav2 lit la carte (éventuellement retouchée) publiée par map_server sur
    # /map ; celle de slam_toolbox part sur /slam_map pour ne pas la remplacer.
    return [odometrie_slam, GroupAction([
        SetRemap(src="/map", dst="/slam_map"),
        SetRemap(src="/map_metadata", dst="/slam_map_metadata"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("slam_toolbox"),
                "launch", "localization_launch.py")),
            launch_arguments={
                "slam_params_file": fichier_slam,
                "use_sim_time": texte_temps,
                "autostart": "true",
                "use_lifecycle_manager": "false",
            }.items(),
        ),
    ])]


def _creer_cartographie_directe(context, profil, dossier_bringup):
    """4. slam_toolbox en cartographie pendant la course (carte préalable
    interdite) : il publie map -> odom et /map, que la costmap globale lit."""
    temps_simulation = _temps_simulation(context)
    with open(os.path.join(dossier_bringup, "config", "slam_mapping.yaml"),
              encoding="utf-8") as flux:
        parametres = yaml.safe_load(flux)
    imu_odom = LaunchConfiguration("imu_odom").perform(context).lower() == "true"
    parametres["slam_toolbox"]["ros__parameters"]["odom_frame"] = (
        "fosa_odom" if imu_odom and profil.get("imu_topic") else "odom")
    fichier_slam = os.path.join(
        tempfile.mkdtemp(prefix="fosa_slam_"), "slam_mapping.yaml")
    with open(fichier_slam, "w", encoding="utf-8") as flux:
        yaml.safe_dump(parametres, flux, sort_keys=False)
    texte_temps = "true" if temps_simulation else "false"
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(dossier_bringup, "launch", "slam_odometry.launch.py")),
            launch_arguments={
                "robot": LaunchConfiguration("robot"),
                "use_sim_time": texte_temps,
                "imu_odom": "true" if imu_odom else "false",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("slam_toolbox"),
                "launch", "online_async_launch.py")),
            launch_arguments={
                "slam_params_file": fichier_slam,
                "use_sim_time": texte_temps,
                "autostart": "true",
                "use_lifecycle_manager": "false",
            }.items(),
        ),
    ]


def _creer_localisation_et_navigation(context):
    dossier_bringup = get_package_share_directory("caytu_nav_bringup")
    profil = _charger_profil(context)
    navigation = _creer_navigation(dossier_bringup, _topic_vitesse(context, profil),
                                   str(profil["odom_topic"]))
    localisation = LaunchConfiguration("localization").perform(context)

    # En mode AMCL, navigation.launch.py démarre lui-même le nœud et applique la
    # pose initiale validée : aucun localisateur Fosa ne doit publier en parallèle.
    if localisation == "amcl":
        return [navigation]

    if localisation == "slam":
        localisateur = _creer_localisation_slam(context, profil, dossier_bringup)
    elif localisation == "slam_live":
        localisateur = _creer_cartographie_directe(context, profil, dossier_bringup)
    else:
        localisateur = _creer_localisateur(context, profil)

    # Nav2 ne démarre qu'une fois la position connue (map -> base_footprint),
    # sur événement et non après un délai fixe. La costmap globale calcule son
    # délai d'attente de la TF avec now() : en temps simulé, now() vaut 0 tant
    # que /clock n'est pas arrivé, puis saute à l'heure de Gazebo, et
    # l'activation échoue si la TF n'existait pas encore. Tout Nav2 reste alors
    # arrêté (constaté avec slam_toolbox comme avec le localisateur roues + IMU).
    attente_localisation = Node(
        package="caytu_nav_solution",
        executable="wait_for_tf",
        name="attente_localisation",
        output="screen",
        parameters=[{
            "use_sim_time": _temps_simulation(context),
            "target_frame": CONFIG.system.global_frame,
            "source_frame": CONFIG.system.base_frame,
        }],
    )

    def _demarrer_nav2(evenement, _contexte):
        if evenement.returncode == 0:
            return [navigation]
        return [LogInfo(msg="Localisation absente : Nav2 n'est pas démarré.")]

    return localisateur + [
        attente_localisation,
        RegisterEventHandler(OnProcessExit(
            target_action=attente_localisation, on_exit=_demarrer_nav2)),
    ]


def generate_launch_description():
    dossier_bringup = get_package_share_directory("caytu_nav_bringup")

    return LaunchDescription(
        _declarer_arguments(dossier_bringup) + [
            OpaqueFunction(function=_valider_arguments_essai),
            _creer_interfaces_robot(dossier_bringup),
            OpaqueFunction(function=_creer_cameras),
            OpaqueFunction(function=_creer_localisation_et_navigation),
        ])
