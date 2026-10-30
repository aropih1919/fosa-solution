"""Sélection, validation et fusion déterministe des paramètres d'expérience."""

from copy import deepcopy
import math
from pathlib import Path

import yaml


CONTROLLERS = ("rpp", "dwb", "mppi")
LOCALIZATIONS = ("odom_imu", "odom_only", "amcl", "amcl_ekf")
PROFILES = (
    "ultra_safe", "safe", "balanced", "fast", "ultra_fast", "tight_clearance")
SCENARIOS = ("full_course",)
SIM_MAX_LINEAR_SPEED = 1.5
SIM_MAX_ANGULAR_SPEED = 1.0

AMCL_EKF_UNAVAILABLE = (
    "amcl_ekf est indisponible : le paquet parc_robot_bringup/URDF n'est pas "
    "installé ici, donc il est impossible de confirmer que le publisher actuel "
    "de odom -> base_footprint peut être désactivé sans doublon TF."
)


class ConfigurationError(ValueError):
    """Configuration inconnue, invalide ou indisponible."""


def _read_yaml(path):
    with Path(path).open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ConfigurationError(f"Le YAML doit contenir un mapping : {path}")
    return value


def deep_merge(base, override):
    """Fusionne les mappings récursivement; listes et valeurs sont remplacées."""
    result = deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def package_is_installed(package_name):
    """Retourne True si ament peut résoudre le package ROS demandé."""
    try:
        from ament_index_python.packages import get_package_prefix
        get_package_prefix(package_name)
        return True
    except (ImportError, LookupError):
        return False


def localization_availability():
    """Décrit les modes réellement sélectionnables dans cet environnement."""
    has_amcl = package_is_installed("nav2_amcl")
    return {
        "odom_imu": {"available": True, "reason": "localisateur Fosa existant"},
        "odom_only": {"available": True, "reason": "odom_imu avec use_imu_yaw=false"},
        "amcl": {
            "available": has_amcl,
            "reason": "Nav2 AMCL installé" if has_amcl else "paquet nav2_amcl absent",
        },
        "amcl_ekf": {"available": False, "reason": AMCL_EKF_UNAVAILABLE},
    }


def validate_selection(controller, localization, profile, scenario="full_course",
                       environment="simulation", check_packages=True):
    """Refuse les choix sans plugin, scénario, environnement ou TF validés."""
    if controller not in CONTROLLERS:
        raise ConfigurationError(
            f"Contrôleur inconnu : {controller}. Choix : {', '.join(CONTROLLERS)}")
    if localization not in LOCALIZATIONS:
        raise ConfigurationError(
            f"Localisation inconnue : {localization}. Choix : {', '.join(LOCALIZATIONS)}")
    if profile not in PROFILES:
        raise ConfigurationError(
            f"Profil inconnu : {profile}. Choix : {', '.join(PROFILES)}")
    if scenario not in SCENARIOS:
        raise ConfigurationError(
            f"Scénario indisponible : {scenario}. Seul full_course possède une fixture.")
    if environment != "simulation":
        raise ConfigurationError(
            "L'environnement robot physique est désactivé : aucune configuration "
            "de vitesse et d'arrêt d'urgence n'a été validée sur le robot réel.")
    if localization == "amcl_ekf":
        raise ConfigurationError(AMCL_EKF_UNAVAILABLE)
    if check_packages and controller == "mppi" and not package_is_installed(
            "nav2_mppi_controller"):
        raise ConfigurationError(
            "MPPI est demandé mais nav2_mppi_controller n'est pas installé. "
            "Installez les dépendances du package puis relancez.")
    if check_packages and localization == "amcl" and not package_is_installed("nav2_amcl"):
        raise ConfigurationError(
            "AMCL est demandé mais le package nav2_amcl n'est pas installé.")


def _set_sim_time(config, use_sim_time):
    """Applique l'horloge à chaque bloc de paramètres ROS de la configuration."""
    for value in config.values():
        if not isinstance(value, dict):
            continue
        if isinstance(value.get("ros__parameters"), dict):
            value["ros__parameters"]["use_sim_time"] = bool(use_sim_time)
        _set_sim_time(value, use_sim_time)


def build_experiment_config(controller, localization, profile, initial_pose,
                            use_sim_time=True, config_root=None, check_packages=True):
    """Construit le YAML complet qui sera chargé par les nœuds ROS de l'essai."""
    validate_selection(
        controller, localization, profile, check_packages=check_packages)

    if config_root is None:
        from ament_index_python.packages import get_package_share_directory
        config_root = Path(get_package_share_directory("caytu_nav_bringup")) / "config"
    config_root = Path(config_root)

    config = _read_yaml(config_root / "nav2_params.yaml")
    if controller == "mppi":
        controller_file = config_root / "controllers" / "mppi.yaml"
    else:
        controller_file = config_root / f"controller_{controller}.yaml"
    config = deep_merge(config, _read_yaml(controller_file))
    profile_file = config_root / "profiles" / profile / f"{controller}.yaml"
    config = deep_merge(config, _read_yaml(profile_file))

    if localization in ("odom_imu", "odom_only"):
        localizer = _read_yaml(config_root / "localization" / "odom_imu.yaml")
        params = localizer["odom_imu_localizer"]["ros__parameters"]
        params["use_imu_yaw"] = localization == "odom_imu"
        config = deep_merge(config, localizer)
    else:
        amcl = _read_yaml(config_root / "localization" / "amcl.yaml")
        amcl_params = amcl["amcl"]["ros__parameters"]
        amcl_params["initial_pose"] = {
            "x": float(initial_pose.x),
            "y": float(initial_pose.y),
            "z": 0.0,
            "yaw": float(initial_pose.yaw),
        }
        config = deep_merge(config, amcl)

    _set_sim_time(config, use_sim_time)
    validate_experiment_config(config, controller, localization, profile)
    return config


def write_experiment_config(config, output_path):
    """Écrit le snapshot YAML complet réellement fourni aux nodes Nav2."""
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream, sort_keys=False, allow_unicode=True)
    return output_path


def validate_localization_map(map_yaml, spawn, goal, clearance_m=0.5):
    """Vérifie carte PGM, couverture et zone libre autour du spawn et du but."""
    from caytu_nav_solution.nav_math import _read_pgm_tokens

    map_yaml = Path(map_yaml).expanduser().resolve()
    metadata = _read_yaml(map_yaml)
    try:
        image_name = metadata["image"]
        resolution = float(metadata["resolution"])
        origin = metadata["origin"]
        origin_x, origin_y = float(origin[0]), float(origin[1])
        origin_yaw = float(origin[2])
        occupied_threshold = float(metadata["occupied_thresh"])
        free_threshold = float(metadata["free_thresh"])
        negate = int(metadata.get("negate", 0))
    except (KeyError, IndexError, TypeError, ValueError) as error:
        raise ConfigurationError(f"Métadonnées de carte incomplètes ({map_yaml}): {error}") from error

    if resolution <= 0.0 or origin_yaw != 0.0 or negate not in (0, 1):
        raise ConfigurationError(
            "AMCL exige ici une carte à résolution positive, sans rotation d'origine, "
            "et avec negate égal à 0 ou 1.")
    image_path = Path(image_name)
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path
    if not image_path.is_file():
        raise ConfigurationError(f"Image de carte absente : {image_path}")

    image_data = image_path.read_bytes()
    tokens, data_offset = _read_pgm_tokens(image_data, 4)
    if tokens[0] != b"P5" or int(tokens[3]) > 255:
        raise ConfigurationError("La validation AMCL accepte une carte PGM P5 8 bits.")
    width, height = int(tokens[1]), int(tokens[2])
    pixels = image_data[data_offset:data_offset + width * height]
    if width <= 0 or height <= 0 or len(pixels) != width * height:
        raise ConfigurationError(f"Image PGM invalide ou tronquée : {image_path}")

    radius_cells = int((clearance_m + resolution - 1e-12) / resolution)
    for label, pose in (("spawn", spawn), ("but", goal)):
        col = int((float(pose.x) - origin_x) / resolution)
        row_bottom = int((float(pose.y) - origin_y) / resolution)
        if not (0 <= col < width and 0 <= row_bottom < height):
            raise ConfigurationError(
                f"Le {label} ({pose.x:.3f}, {pose.y:.3f}) est hors de la carte AMCL.")
        for row in range(row_bottom - radius_cells, row_bottom + radius_cells + 1):
            for column in range(col - radius_cells, col + radius_cells + 1):
                if not (0 <= column < width and 0 <= row < height):
                    raise ConfigurationError(
                        f"La marge libre de {clearance_m:.2f} m autour du {label} "
                        "sort de la carte AMCL.")
                pixel = pixels[(height - 1 - row) * width + column] / 255.0
                occupancy = pixel if negate else 1.0 - pixel
                if occupancy >= free_threshold or occupancy > occupied_threshold:
                    raise ConfigurationError(
                        f"La zone de {clearance_m:.2f} m autour du {label} n'est pas "
                        "entièrement libre sur la carte AMCL.")
    return str(map_yaml)


def validate_scan_map_agreement(agreement, used_rays, minimum_agreement=0.35):
    """Refuse AMCL si le scan initial ne confirme pas suffisamment les murs."""
    if used_rays < 30 or agreement < minimum_agreement:
        raise ConfigurationError(
            f"Carte/scan non validés ({used_rays} rayons, accord {agreement:.0%}, "
            f"minimum {minimum_agreement:.0%}); AMCL ne démarre pas.")


def validate_experiment_config(config, controller, localization, profile):
    """Valide un YAML complet avant qu'un launch puisse charger les nœuds."""
    _validate_final_config(config, controller, profile)
    if localization == "amcl":
        if "amcl" not in config or "odom_imu_localizer" in config:
            raise ConfigurationError(
                "AMCL exige son bloc de paramètres et l'absence du localisateur Fosa.")
        amcl = config["amcl"].get("ros__parameters", {})
        if amcl.get("scan_topic") != "/scan_clean":
            raise ConfigurationError("AMCL doit consommer le scan filtré /scan_clean.")
        if not amcl.get("set_initial_pose") or not isinstance(amcl.get("initial_pose"), dict):
            raise ConfigurationError("AMCL exige une pose initiale explicite.")
    elif localization in ("odom_imu", "odom_only"):
        if "odom_imu_localizer" not in config or "amcl" in config:
            raise ConfigurationError(
                "Le mode odométrie exige le localisateur Fosa et ne doit pas charger AMCL.")
        params = config["odom_imu_localizer"].get("ros__parameters", {})
        if params.get("use_imu_yaw") is not (localization == "odom_imu"):
            raise ConfigurationError(
                f"use_imu_yaw ne correspond pas au mode {localization}.")
    else:
        raise ConfigurationError(f"Localisation non prise en charge : {localization}")


def _validate_final_config(config, controller, profile):
    controller_params = config["controller_server"]["ros__parameters"]
    if controller_params.get("controller_plugins") != ["FollowPath"]:
        raise ConfigurationError("Le controller_server doit avoir un seul plugin FollowPath.")
    follow_path = controller_params["FollowPath"]
    expected = {
        "rpp": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
        "dwb": "dwb_core::DWBLocalPlanner",
        "mppi": "nav2_mppi_controller::MPPIController",
    }[controller]
    if follow_path.get("plugin") != expected:
        raise ConfigurationError(
            f"Plugin {controller} invalide : {follow_path.get('plugin')!r}")

    for key in ("global_costmap", "local_costmap"):
        params = config[key][key]["ros__parameters"]
        if "inflation_layer" not in params["plugins"]:
            raise ConfigurationError(f"{key}: la couche inflation_layer est obligatoire.")
    if controller == "mppi" and follow_path.get("motion_model") != "DiffDrive":
        raise ConfigurationError("MPPI doit utiliser le modèle différentiel DiffDrive.")

    local_costmap = config["local_costmap"]["local_costmap"]["ros__parameters"]
    inflation = local_costmap["inflation_layer"]
    if controller == "rpp":
        if follow_path.get("cost_scaling_dist", 0.0) > inflation["inflation_radius"]:
            raise ConfigurationError(
                "RPP cost_scaling_dist dépasse le rayon d'inflation local.")
        if not math.isclose(
                follow_path.get("inflation_cost_scaling_factor", -1.0),
                inflation["cost_scaling_factor"], rel_tol=1e-6):
            raise ConfigurationError(
                "Le facteur d'inflation de RPP doit correspondre à la costmap locale.")
    elif controller == "dwb":
        if follow_path.get("min_vel_x", 0.0) > follow_path.get("max_vel_x", 0.0):
            raise ConfigurationError("DWB min_vel_x dépasse max_vel_x.")
        if follow_path.get("max_vel_y", 0.0) != 0.0 or follow_path.get("vy_samples", 0) < 1:
            raise ConfigurationError("DWB doit rester différentiel avec vy_samples positif.")
        for sample_name in ("vx_samples", "vtheta_samples"):
            if follow_path.get(sample_name, 0) < 2:
                raise ConfigurationError(f"DWB {sample_name} doit échantillonner plusieurs vitesses.")
        if follow_path.get("acc_lim_x", 0.0) < 0.0 or follow_path.get("decel_lim_x", 0.0) > 0.0:
            raise ConfigurationError("DWB accélération/décélération linéaires incohérentes.")
    elif controller == "mppi":
        if follow_path.get("batch_size", 0) < 1 or follow_path.get("time_steps", 0) < 1:
            raise ConfigurationError("MPPI batch_size et time_steps doivent être positifs.")

    max_linear = _max_linear_speed(controller, follow_path)
    if max_linear > SIM_MAX_LINEAR_SPEED:
        raise ConfigurationError(
            f"Vitesse {max_linear} m/s supérieure à la limite simulée documentée "
            f"({SIM_MAX_LINEAR_SPEED} m/s).")
    if controller == "dwb":
        max_angular = follow_path.get("max_vel_theta", 0.0)
    elif controller == "mppi":
        max_angular = follow_path.get("wz_max", 0.0)
    else:
        max_angular = follow_path.get("rotate_to_heading_angular_vel", 0.0)
    if max_angular > SIM_MAX_ANGULAR_SPEED:
        raise ConfigurationError(
            f"Vitesse angulaire {max_angular} rad/s supérieure à la limite "
            f"simulée documentée ({SIM_MAX_ANGULAR_SPEED} rad/s).")

    # Les profils ne doivent pas supprimer la détection d'obstacles ni réduire
    # l'empreinte du robot pour obtenir des temps artificiellement meilleurs.
    if profile in PROFILES:
        for key in ("global_costmap", "local_costmap"):
            params = config[key][key]["ros__parameters"]
            if params.get("footprint_padding", 0.0) < 0.02:
                raise ConfigurationError(f"{key}: marge de footprint réduite.")
            if not any("camera_layer" in name or "lidar_layer" in name
                       for name in params["plugins"]):
                raise ConfigurationError(f"{key}: couches capteurs manquantes.")


def _max_linear_speed(controller, params):
    if controller == "rpp":
        return float(params["desired_linear_vel"])
    if controller == "dwb":
        return float(params["max_speed_xy"])
    return float(params["vx_max"])
