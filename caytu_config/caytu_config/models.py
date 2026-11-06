"""Schémas typés de la configuration Fosa.

Chaque section est validée au chargement : une clé inconnue, une valeur vide
ou un nombre hors bornes arrête le démarrage avec un message explicite. Les
valeurs par défaut vivent dans `config/fosa.yaml`, jamais dans les nœuds.
"""

from dataclasses import dataclass

from caytu_config.errors import ConfigurationError
from caytu_config.modeles_course import (
    _as_int_list,
    CourseConfig,
    DepthScanConfig,
    ObstacleSourceConfig,
    StartWordConfig,
    TagCameraConfig,
)
from caytu_config.validation import (
    _as_bool,
    _as_int,
    _as_list,
    _as_number,
    _as_numbers,
    _as_str,
    _check_keys,
    _choice,
    _get,
    _section,
)

DEPLOYMENTS = ("robot", "local", "remote_pc", "simulation")


# -------------------------------------------------------------------- système
@dataclass(frozen=True)
class SystemConfig:
    deployment: str
    use_sim_time: bool
    global_frame: str
    base_frame: str
    ros_domain_id: int
    clock_topic: str

    @classmethod
    def from_mapping(cls, data, path="system"):
        data = _section(data, path)
        _check_keys(data, (
            "deployment", "use_sim_time", "global_frame", "base_frame",
            "ros_domain_id", "clock_topic"), path)
        return cls(
            deployment=_choice(
                _get(data, "deployment", path), DEPLOYMENTS,
                f"{path}.deployment"),
            use_sim_time=_as_bool(
                _get(data, "use_sim_time", path), f"{path}.use_sim_time"),
            global_frame=_as_str(
                _get(data, "global_frame", path), f"{path}.global_frame"),
            base_frame=_as_str(
                _get(data, "base_frame", path), f"{path}.base_frame"),
            ros_domain_id=_as_int(
                _get(data, "ros_domain_id", path), f"{path}.ros_domain_id",
                0, 232),
            clock_topic=_as_str(
                _get(data, "clock_topic", path), f"{path}.clock_topic"),
        )


# ---------------------------------------------------------------- navigation
def _as_footprint(value, path):
    """Polygone [[x, y], ...] d'au moins trois sommets, dans base_frame."""
    sommets = _as_list(value, path)
    if len(sommets) < 3:
        raise ConfigurationError(f"{path} doit contenir au moins 3 sommets.")
    polygone = []
    for index, sommet in enumerate(sommets):
        if len(_as_list(sommet, f"{path}[{index}]")) != 2:
            raise ConfigurationError(f"{path}[{index}] doit être un couple [x, y].")
        polygone.append(_as_numbers(sommet, f"{path}[{index}]"))
    return tuple(polygone)


def _as_obstacle_sources(value, path):
    sources = tuple(
        ObstacleSourceConfig.from_mapping(source, f"{path}[{index}]")
        for index, source in enumerate(_as_list(value, path)))
    if not any(source.enabled for source in sources):
        raise ConfigurationError(f"{path}: au moins une source doit être active.")
    noms = [source.name for source in sources]
    if len(set(noms)) != len(noms):
        raise ConfigurationError(f"{path}: noms de sources en double.")
    return sources


@dataclass(frozen=True)
class ConfigurationSecurite:
    """Garde commune aux commandes Nav2 et à l'asservissement visuel."""

    topic_entree: str
    age_scan_max_sec: float
    age_commande_max_sec: float
    horizon_sec: float
    pas_sec: float
    periode_sec: float
    marge_m: float

    @classmethod
    def from_mapping(cls, donnees, chemin="navigation.securite"):
        donnees = _section(donnees, chemin)
        nombres = ("age_scan_max_sec", "age_commande_max_sec", "horizon_sec",
                   "pas_sec", "periode_sec", "marge_m")
        _check_keys(donnees, ("topic_entree",) + nombres, chemin)
        securite = cls(
            topic_entree=_as_str(_get(donnees, "topic_entree", chemin),
                                f"{chemin}.topic_entree"),
            **{cle: _as_number(_get(donnees, cle, chemin), f"{chemin}.{cle}",
                               positive=True) for cle in nombres})
        if securite.pas_sec > securite.horizon_sec:
            raise ConfigurationError(f"{chemin}: pas_sec dépasse horizon_sec.")
        return securite


@dataclass(frozen=True)
class NavigationConfig:
    mode: str
    rolling_window_size_m: float
    securite: ConfigurationSecurite
    profile: str
    success_radius: float
    arrival_recheck_radius: float
    scan_topic: str
    cmd_vel_topic: str
    odom_topic: str
    odom_frame: str
    footprint: tuple
    footprint_padding: float
    center_offset_x: float
    max_linear_speed: float
    max_angular_speed: float
    laser_self_filter: bool
    obstacle_sources: tuple
    global_costmap_clear_service: str
    local_costmap_clear_service: str
    cancel_timeout_sec: float
    feedback_log_period_sec: float

    @classmethod
    def from_mapping(cls, data, path="navigation"):
        data = _section(data, path)
        _check_keys(data, (
            "mode", "rolling_window_size_m", "securite",
            "profile", "success_radius", "arrival_recheck_radius",
            "scan_topic", "cmd_vel_topic", "odom_topic", "odom_frame",
            "footprint", "footprint_padding", "center_offset_x",
            "max_linear_speed", "max_angular_speed", "laser_self_filter",
            "obstacle_sources", "global_costmap_clear_service",
            "local_costmap_clear_service", "cancel_timeout_sec",
            "feedback_log_period_sec"), path)
        return cls(
            mode=_choice(_get(data, "mode", path), ("carte", "sans_carte"),
                         f"{path}.mode"),
            rolling_window_size_m=_as_number(
                _get(data, "rolling_window_size_m", path),
                f"{path}.rolling_window_size_m", positive=True),
            securite=ConfigurationSecurite.from_mapping(
                _get(data, "securite", path), f"{path}.securite"),
            profile=_as_str(_get(data, "profile", path), f"{path}.profile"),
            success_radius=_as_number(
                _get(data, "success_radius", path), f"{path}.success_radius",
                positive=True),
            arrival_recheck_radius=_as_number(
                _get(data, "arrival_recheck_radius", path),
                f"{path}.arrival_recheck_radius", positive=True),
            scan_topic=_as_str(
                _get(data, "scan_topic", path), f"{path}.scan_topic"),
            cmd_vel_topic=_as_str(
                _get(data, "cmd_vel_topic", path), f"{path}.cmd_vel_topic"),
            odom_topic=_as_str(
                _get(data, "odom_topic", path), f"{path}.odom_topic"),
            odom_frame=_as_str(
                _get(data, "odom_frame", path), f"{path}.odom_frame"),
            footprint=_as_footprint(
                _get(data, "footprint", path), f"{path}.footprint"),
            footprint_padding=_as_number(
                _get(data, "footprint_padding", path),
                f"{path}.footprint_padding", minimum=0.0),
            center_offset_x=_as_number(
                _get(data, "center_offset_x", path), f"{path}.center_offset_x"),
            max_linear_speed=_as_number(
                _get(data, "max_linear_speed", path),
                f"{path}.max_linear_speed", positive=True, maximum=1.5),
            max_angular_speed=_as_number(
                _get(data, "max_angular_speed", path),
                f"{path}.max_angular_speed", positive=True, maximum=2.0),
            laser_self_filter=_as_bool(
                _get(data, "laser_self_filter", path),
                f"{path}.laser_self_filter"),
            obstacle_sources=_as_obstacle_sources(
                _get(data, "obstacle_sources", path),
                f"{path}.obstacle_sources"),
            global_costmap_clear_service=_as_str(
                _get(data, "global_costmap_clear_service", path),
                f"{path}.global_costmap_clear_service"),
            local_costmap_clear_service=_as_str(
                _get(data, "local_costmap_clear_service", path),
                f"{path}.local_costmap_clear_service"),
            cancel_timeout_sec=_as_number(
                _get(data, "cancel_timeout_sec", path),
                f"{path}.cancel_timeout_sec", positive=True),
            feedback_log_period_sec=_as_number(
                _get(data, "feedback_log_period_sec", path),
                f"{path}.feedback_log_period_sec", positive=True),
        )


# ------------------------------------------------------------------ perception
@dataclass(frozen=True)
class AprilTagConfig:
    family: str
    size_m: float
    max_distance_m: float
    ids: tuple
    min_marker_distance_rate: float
    max_rate_hz: float
    detection_image_topic: str
    detection_image_rate_hz: float
    tf_timeout_sec: float
    tf_max_lag_sec: float
    cameras: tuple

    @classmethod
    def from_mapping(cls, data, path="perception.apriltag"):
        data = _section(data, path)
        _check_keys(data, (
            "family", "size_m", "max_distance_m", "ids",
            "min_marker_distance_rate", "max_rate_hz",
            "detection_image_topic", "detection_image_rate_hz",
            "tf_timeout_sec", "tf_max_lag_sec", "cameras"), path)
        cameras = tuple(
            TagCameraConfig.from_mapping(camera, f"{path}.cameras[{index}]")
            for index, camera in enumerate(
                _as_list(_get(data, "cameras", path), f"{path}.cameras")))
        if not cameras:
            raise ConfigurationError(f"{path}.cameras ne doit pas être vide.")
        if len({camera.name for camera in cameras}) != len(cameras):
            raise ConfigurationError(f"{path}.cameras: noms en double.")
        return cls(
            ids=_as_int_list(_get(data, "ids", path), f"{path}.ids"),
            min_marker_distance_rate=_as_number(
                _get(data, "min_marker_distance_rate", path),
                f"{path}.min_marker_distance_rate", 0.0, 1.0),
            max_rate_hz=_as_number(
                _get(data, "max_rate_hz", path), f"{path}.max_rate_hz",
                positive=True),
            detection_image_topic=_as_str(
                _get(data, "detection_image_topic", path),
                f"{path}.detection_image_topic"),
            detection_image_rate_hz=_as_number(
                _get(data, "detection_image_rate_hz", path),
                f"{path}.detection_image_rate_hz", positive=True),
            tf_timeout_sec=_as_number(
                _get(data, "tf_timeout_sec", path), f"{path}.tf_timeout_sec",
                positive=True),
            tf_max_lag_sec=_as_number(
                _get(data, "tf_max_lag_sec", path), f"{path}.tf_max_lag_sec",
                minimum=0.0),
            cameras=cameras,
            family=_as_str(_get(data, "family", path), f"{path}.family"),
            size_m=_as_number(
                _get(data, "size_m", path), f"{path}.size_m", positive=True),
            max_distance_m=_as_number(
                _get(data, "max_distance_m", path),
                f"{path}.max_distance_m", positive=True),
        )


@dataclass(frozen=True)
class PerceptionConfig:
    tags_topic: str
    apriltag: AprilTagConfig
    depth_scans: tuple

    @classmethod
    def from_mapping(cls, data, path="perception"):
        data = _section(data, path)
        _check_keys(data, (
            "tags_topic", "apriltag", "depth_scans"), path)
        depth_scans = tuple(
            DepthScanConfig.from_mapping(scan, f"{path}.depth_scans[{index}]")
            for index, scan in enumerate(
                _as_list(_get(data, "depth_scans", path), f"{path}.depth_scans")))
        if len({scan.name for scan in depth_scans}) != len(depth_scans):
            raise ConfigurationError(f"{path}.depth_scans: noms en double.")
        return cls(
            depth_scans=depth_scans,
            tags_topic=_as_str(
                _get(data, "tags_topic", path), f"{path}.tags_topic"),
            apriltag=AprilTagConfig.from_mapping(
                _get(data, "apriltag", path), f"{path}.apriltag"),
        )


# ------------------------------------------------------------------ racine
@dataclass(frozen=True)
class FosaConfig:
    system: SystemConfig
    navigation: NavigationConfig
    perception: PerceptionConfig
    course: CourseConfig
    start_word: StartWordConfig
