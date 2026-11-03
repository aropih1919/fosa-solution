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
    ObstacleSourceConfig,
    StartWordConfig,
    TagCameraConfig,
)
from caytu_config.validation import (
    _MISSING,
    _as_bool,
    _as_int,
    _as_list,
    _as_number,
    _as_numbers,
    _as_str,
    _check_keys,
    _choice,
    _get,
    _normaliser,
    _section,
)

DEPLOYMENTS = ("robot", "local", "remote_pc", "simulation")
PROVIDERS = ("local", "api", "auto")
MAP_MODES = ("auto", "always", "never")
APRILTAG_DETECTORS = ("opencv", "apriltag_ros")


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
class NavigationConfig:
    map_yaml: str
    default_map: str
    static_map: str
    profile: str
    controller: str
    localization: str
    fallback_profile: str
    fallback_controller: str
    fallback_localization: str
    time_limit_sec: float
    stop_margin_sec: float
    success_radius: float
    arrival_recheck_radius: float
    retry_pause_sec: float
    nav2_startup_timeout_sec: float
    map_check_timeout_sec: float
    map_check_min_agreement: float
    launch_bringup: bool
    bringup_package: str
    bringup_launch: str
    scan_topic: str
    amcl_pose_topic: str
    cmd_vel_topic: str
    global_costmap_clear_service: str
    local_costmap_clear_service: str
    guiding_speed_mps: float
    guiding_wait_sec: float
    blank_map_resolution: float
    blank_map_margin: float
    clock_wait_sec: float
    cancel_timeout_sec: float
    bringup_stop_timeouts: tuple
    stop_repeat_count: int
    feedback_log_period_sec: float
    floor_line_distance: float
    odom_topic: str
    odom_frame: str
    footprint: tuple
    footprint_padding: float
    center_offset_x: float
    max_linear_speed: float
    max_angular_speed: float
    laser_self_filter: bool
    obstacle_sources: tuple

    @classmethod
    def from_mapping(cls, data, path="navigation"):
        data = _section(data, path)
        _check_keys(data, (
            "map_yaml", "default_map", "static_map", "profile", "controller",
            "localization", "fallback_profile", "fallback_controller",
            "fallback_localization", "time_limit_sec", "stop_margin_sec",
            "success_radius", "arrival_recheck_radius", "retry_pause_sec",
            "nav2_startup_timeout_sec", "map_check_timeout_sec",
            "map_check_min_agreement", "launch_bringup", "bringup_package",
            "bringup_launch", "scan_topic", "amcl_pose_topic",
            "cmd_vel_topic", "global_costmap_clear_service",
            "local_costmap_clear_service", "guiding_speed_mps",
            "guiding_wait_sec", "blank_map_resolution", "blank_map_margin",
            "clock_wait_sec", "cancel_timeout_sec", "bringup_stop_timeouts",
            "stop_repeat_count", "feedback_log_period_sec",
            "floor_line_distance", "odom_topic", "odom_frame", "footprint",
            "footprint_padding", "center_offset_x", "max_linear_speed",
            "max_angular_speed", "laser_self_filter", "obstacle_sources"), path)
        return cls(
            map_yaml=_as_str(
                _get(data, "map_yaml", path, ""), f"{path}.map_yaml",
                allow_empty=True),
            default_map=_as_str(
                _get(data, "default_map", path), f"{path}.default_map"),
            static_map=_choice(
                _get(data, "static_map", path), MAP_MODES,
                f"{path}.static_map"),
            profile=_as_str(_get(data, "profile", path), f"{path}.profile"),
            controller=_as_str(
                _get(data, "controller", path), f"{path}.controller"),
            localization=_as_str(
                _get(data, "localization", path), f"{path}.localization"),
            fallback_profile=_as_str(
                _get(data, "fallback_profile", path),
                f"{path}.fallback_profile"),
            fallback_controller=_as_str(
                _get(data, "fallback_controller", path),
                f"{path}.fallback_controller"),
            fallback_localization=_as_str(
                _get(data, "fallback_localization", path),
                f"{path}.fallback_localization"),
            time_limit_sec=_as_number(
                _get(data, "time_limit_sec", path), f"{path}.time_limit_sec",
                positive=True),
            stop_margin_sec=_as_number(
                _get(data, "stop_margin_sec", path), f"{path}.stop_margin_sec",
                minimum=0.0),
            success_radius=_as_number(
                _get(data, "success_radius", path), f"{path}.success_radius",
                positive=True),
            arrival_recheck_radius=_as_number(
                _get(data, "arrival_recheck_radius", path),
                f"{path}.arrival_recheck_radius", positive=True),
            retry_pause_sec=_as_number(
                _get(data, "retry_pause_sec", path), f"{path}.retry_pause_sec",
                minimum=0.0),
            nav2_startup_timeout_sec=_as_number(
                _get(data, "nav2_startup_timeout_sec", path),
                f"{path}.nav2_startup_timeout_sec", positive=True),
            map_check_timeout_sec=_as_number(
                _get(data, "map_check_timeout_sec", path),
                f"{path}.map_check_timeout_sec", positive=True),
            map_check_min_agreement=_as_number(
                _get(data, "map_check_min_agreement", path),
                f"{path}.map_check_min_agreement", 0.0, 1.0),
            launch_bringup=_as_bool(
                _get(data, "launch_bringup", path), f"{path}.launch_bringup"),
            bringup_package=_as_str(
                _get(data, "bringup_package", path),
                f"{path}.bringup_package"),
            bringup_launch=_as_str(
                _get(data, "bringup_launch", path),
                f"{path}.bringup_launch"),
            scan_topic=_as_str(
                _get(data, "scan_topic", path), f"{path}.scan_topic"),
            amcl_pose_topic=_as_str(
                _get(data, "amcl_pose_topic", path),
                f"{path}.amcl_pose_topic"),
            cmd_vel_topic=_as_str(
                _get(data, "cmd_vel_topic", path), f"{path}.cmd_vel_topic"),
            global_costmap_clear_service=_as_str(
                _get(data, "global_costmap_clear_service", path),
                f"{path}.global_costmap_clear_service"),
            local_costmap_clear_service=_as_str(
                _get(data, "local_costmap_clear_service", path),
                f"{path}.local_costmap_clear_service"),
            guiding_speed_mps=_as_number(
                _get(data, "guiding_speed_mps", path),
                f"{path}.guiding_speed_mps", positive=True),
            guiding_wait_sec=_as_number(
                _get(data, "guiding_wait_sec", path),
                f"{path}.guiding_wait_sec", minimum=0.0),
            blank_map_resolution=_as_number(
                _get(data, "blank_map_resolution", path),
                f"{path}.blank_map_resolution", positive=True),
            blank_map_margin=_as_number(
                _get(data, "blank_map_margin", path),
                f"{path}.blank_map_margin", minimum=0.0),
            clock_wait_sec=_as_number(
                _get(data, "clock_wait_sec", path),
                f"{path}.clock_wait_sec", positive=True),
            cancel_timeout_sec=_as_number(
                _get(data, "cancel_timeout_sec", path),
                f"{path}.cancel_timeout_sec", positive=True),
            bringup_stop_timeouts=_as_numbers(
                _get(data, "bringup_stop_timeouts", path),
                f"{path}.bringup_stop_timeouts", positive=True),
            stop_repeat_count=_as_int(
                _get(data, "stop_repeat_count", path),
                f"{path}.stop_repeat_count", 1),
            feedback_log_period_sec=_as_number(
                _get(data, "feedback_log_period_sec", path),
                f"{path}.feedback_log_period_sec", positive=True),
            floor_line_distance=_as_number(
                _get(data, "floor_line_distance", path),
                f"{path}.floor_line_distance", minimum=0.0),
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
        )


# ------------------------------------------------------------------ perception
@dataclass(frozen=True)
class AprilTagConfig:
    family: str
    size_m: float
    max_distance_m: float
    detector: str
    calibration_samples: int
    calibration_file: str
    ids: tuple
    min_marker_distance_rate: float
    max_rate_hz: float
    detection_image_topic: str
    tf_timeout_sec: float
    cameras: tuple

    @classmethod
    def from_mapping(cls, data, path="perception.apriltag"):
        data = _section(data, path)
        _check_keys(data, (
            "family", "size_m", "max_distance_m", "detector",
            "calibration_samples", "calibration_file", "ids",
            "min_marker_distance_rate", "max_rate_hz",
            "detection_image_topic", "tf_timeout_sec", "cameras"), path)
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
            tf_timeout_sec=_as_number(
                _get(data, "tf_timeout_sec", path), f"{path}.tf_timeout_sec",
                positive=True),
            cameras=cameras,
            family=_as_str(_get(data, "family", path), f"{path}.family"),
            size_m=_as_number(
                _get(data, "size_m", path), f"{path}.size_m", positive=True),
            max_distance_m=_as_number(
                _get(data, "max_distance_m", path),
                f"{path}.max_distance_m", positive=True),
            detector=_choice(
                _get(data, "detector", path), APRILTAG_DETECTORS,
                f"{path}.detector"),
            calibration_samples=_as_int(
                _get(data, "calibration_samples", path),
                f"{path}.calibration_samples", 1),
            calibration_file=_as_str(
                _get(data, "calibration_file", path),
                f"{path}.calibration_file"),
        )


@dataclass(frozen=True)
class YoloConfig:
    enabled: bool
    model_path: str
    confidence: float
    input_size: int
    person_class_index: int
    max_fps: float
    device: str

    @classmethod
    def from_mapping(cls, data, path="perception.yolo"):
        data = _section(data, path)
        _check_keys(data, (
            "enabled", "model_path", "confidence", "input_size",
            "person_class_index", "max_fps", "device"), path)
        return cls(
            enabled=_as_bool(_get(data, "enabled", path), f"{path}.enabled"),
            model_path=_as_str(
                _get(data, "model_path", path), f"{path}.model_path",
                allow_empty=True),
            confidence=_as_number(
                _get(data, "confidence", path), f"{path}.confidence", 0.0, 1.0),
            input_size=_as_int(
                _get(data, "input_size", path), f"{path}.input_size", 1),
            person_class_index=_as_int(
                _get(data, "person_class_index", path),
                f"{path}.person_class_index", 0),
            max_fps=_as_number(
                _get(data, "max_fps", path), f"{path}.max_fps", positive=True),
            device=_as_str(_get(data, "device", path), f"{path}.device"),
        )


@dataclass(frozen=True)
class PersonConfig:
    min_distance_m: float
    max_distance_m: float
    min_confidence: float
    consecutive_frames: int
    cooldown_sec: float
    dynamic_speed_mps: float
    min_height_m: float
    max_width_m: float
    cluster_tolerance_m: float
    min_cluster_points: int
    tracker_match_m: float
    bearing_tolerance_rad: float
    front_fov_rad: float
    robot_stop_threshold_mps: float

    @classmethod
    def from_mapping(cls, data, path="perception.person"):
        data = _section(data, path)
        _check_keys(data, (
            "min_distance_m", "max_distance_m", "min_confidence",
            "consecutive_frames", "cooldown_sec", "dynamic_speed_mps",
            "min_height_m", "max_width_m", "cluster_tolerance_m",
            "min_cluster_points", "tracker_match_m",
            "bearing_tolerance_rad", "front_fov_rad",
            "robot_stop_threshold_mps"), path)
        config = cls(
            min_distance_m=_as_number(
                _get(data, "min_distance_m", path),
                f"{path}.min_distance_m", positive=True),
            max_distance_m=_as_number(
                _get(data, "max_distance_m", path),
                f"{path}.max_distance_m", positive=True),
            min_confidence=_as_number(
                _get(data, "min_confidence", path),
                f"{path}.min_confidence", 0.0, 1.0),
            consecutive_frames=_as_int(
                _get(data, "consecutive_frames", path),
                f"{path}.consecutive_frames", 1),
            cooldown_sec=_as_number(
                _get(data, "cooldown_sec", path), f"{path}.cooldown_sec",
                minimum=0.0),
            dynamic_speed_mps=_as_number(
                _get(data, "dynamic_speed_mps", path),
                f"{path}.dynamic_speed_mps", minimum=0.0),
            min_height_m=_as_number(
                _get(data, "min_height_m", path),
                f"{path}.min_height_m", positive=True),
            max_width_m=_as_number(
                _get(data, "max_width_m", path),
                f"{path}.max_width_m", positive=True),
            cluster_tolerance_m=_as_number(
                _get(data, "cluster_tolerance_m", path),
                f"{path}.cluster_tolerance_m", positive=True),
            min_cluster_points=_as_int(
                _get(data, "min_cluster_points", path),
                f"{path}.min_cluster_points", 1),
            tracker_match_m=_as_number(
                _get(data, "tracker_match_m", path),
                f"{path}.tracker_match_m", positive=True),
            bearing_tolerance_rad=_as_number(
                _get(data, "bearing_tolerance_rad", path),
                f"{path}.bearing_tolerance_rad", positive=True),
            front_fov_rad=_as_number(
                _get(data, "front_fov_rad", path),
                f"{path}.front_fov_rad", positive=True),
            robot_stop_threshold_mps=_as_number(
                _get(data, "robot_stop_threshold_mps", path),
                f"{path}.robot_stop_threshold_mps", minimum=0.0),
        )
        if config.min_distance_m >= config.max_distance_m:
            raise ConfigurationError(
                f"{path}: min_distance_m doit être inférieur à max_distance_m.")
        return config


@dataclass(frozen=True)
class ScanConversionConfig:
    cloud_topic: str
    scan_topic: str
    target_frame: str
    transform_tolerance: float
    min_height: float
    max_height: float
    angle_min: float
    angle_max: float
    angle_increment: float
    scan_time: float
    range_min: float
    range_max: float
    use_inf: bool
    inf_epsilon: float

    @classmethod
    def from_mapping(cls, data, path):
        data = _section(data, path)
        _check_keys(data, (
            "cloud_topic", "scan_topic", "target_frame", "transform_tolerance",
            "min_height", "max_height", "angle_min", "angle_max",
            "angle_increment", "scan_time", "range_min", "range_max",
            "use_inf", "inf_epsilon"), path)
        config = cls(
            cloud_topic=_as_str(
                _get(data, "cloud_topic", path), f"{path}.cloud_topic"),
            scan_topic=_as_str(
                _get(data, "scan_topic", path), f"{path}.scan_topic"),
            target_frame=_as_str(
                _get(data, "target_frame", path), f"{path}.target_frame"),
            transform_tolerance=_as_number(
                _get(data, "transform_tolerance", path),
                f"{path}.transform_tolerance", positive=True),
            min_height=_as_number(
                _get(data, "min_height", path), f"{path}.min_height"),
            max_height=_as_number(
                _get(data, "max_height", path), f"{path}.max_height"),
            angle_min=_as_number(
                _get(data, "angle_min", path), f"{path}.angle_min"),
            angle_max=_as_number(
                _get(data, "angle_max", path), f"{path}.angle_max"),
            angle_increment=_as_number(
                _get(data, "angle_increment", path),
                f"{path}.angle_increment", positive=True),
            scan_time=_as_number(
                _get(data, "scan_time", path), f"{path}.scan_time",
                positive=True),
            range_min=_as_number(
                _get(data, "range_min", path), f"{path}.range_min",
                positive=True),
            range_max=_as_number(
                _get(data, "range_max", path), f"{path}.range_max",
                positive=True),
            use_inf=_as_bool(
                _get(data, "use_inf", path), f"{path}.use_inf"),
            inf_epsilon=_as_number(
                _get(data, "inf_epsilon", path), f"{path}.inf_epsilon",
                positive=True),
        )
        if config.min_height >= config.max_height:
            raise ConfigurationError(
                f"{path}: min_height doit être inférieur à max_height.")
        if config.angle_min >= config.angle_max:
            raise ConfigurationError(
                f"{path}: angle_min doit être inférieur à angle_max.")
        if config.range_min >= config.range_max:
            raise ConfigurationError(
                f"{path}: range_min doit être inférieur à range_max.")
        return config


@dataclass(frozen=True)
class PerceptionConfig:
    top_color_topic: str
    top_depth_topic: str
    top_info_topic: str
    bottom_color_topic: str
    bottom_depth_topic: str
    entities_topic: str
    tags_topic: str
    person_events_topic: str
    apriltag: AprilTagConfig
    yolo: YoloConfig
    person: PersonConfig
    top_scan: ScanConversionConfig
    bottom_scan: ScanConversionConfig

    @classmethod
    def from_mapping(cls, data, path="perception"):
        data = _section(data, path)
        _check_keys(data, (
            "top_color_topic", "top_depth_topic", "top_info_topic",
            "bottom_color_topic", "bottom_depth_topic", "entities_topic",
            "tags_topic", "person_events_topic", "apriltag", "yolo",
            "person", "top_scan", "bottom_scan"), path)
        return cls(
            top_color_topic=_as_str(
                _get(data, "top_color_topic", path),
                f"{path}.top_color_topic"),
            top_depth_topic=_as_str(
                _get(data, "top_depth_topic", path),
                f"{path}.top_depth_topic"),
            top_info_topic=_as_str(
                _get(data, "top_info_topic", path),
                f"{path}.top_info_topic"),
            bottom_color_topic=_as_str(
                _get(data, "bottom_color_topic", path),
                f"{path}.bottom_color_topic"),
            bottom_depth_topic=_as_str(
                _get(data, "bottom_depth_topic", path),
                f"{path}.bottom_depth_topic"),
            entities_topic=_as_str(
                _get(data, "entities_topic", path),
                f"{path}.entities_topic"),
            tags_topic=_as_str(
                _get(data, "tags_topic", path), f"{path}.tags_topic"),
            person_events_topic=_as_str(
                _get(data, "person_events_topic", path),
                f"{path}.person_events_topic"),
            apriltag=AprilTagConfig.from_mapping(
                _get(data, "apriltag", path), f"{path}.apriltag"),
            yolo=YoloConfig.from_mapping(
                _get(data, "yolo", path), f"{path}.yolo"),
            person=PersonConfig.from_mapping(
                _get(data, "person", path), f"{path}.person"),
            top_scan=ScanConversionConfig.from_mapping(
                _get(data, "top_scan", path), f"{path}.top_scan"),
            bottom_scan=ScanConversionConfig.from_mapping(
                _get(data, "bottom_scan", path), f"{path}.bottom_scan"),
        )


# ----------------------------------------------------------------------- audio
@dataclass(frozen=True)
class AudioConfig:
    input_device: str
    output_device: str
    sample_rate: int
    vad_threshold: float
    vad_rms_scale: float
    vad_frame_ms: int
    silence_ms: int
    piper_executable: str
    audio_player: str

    @classmethod
    def from_mapping(cls, data, path="audio"):
        data = _section(data, path)
        _check_keys(data, (
            "input_device", "output_device", "sample_rate", "vad_threshold",
            "vad_rms_scale",
            "vad_frame_ms", "silence_ms", "piper_executable",
            "audio_player"), path)
        return cls(
            input_device=_as_str(
                _get(data, "input_device", path), f"{path}.input_device"),
            output_device=_as_str(
                _get(data, "output_device", path), f"{path}.output_device"),
            sample_rate=_as_int(
                _get(data, "sample_rate", path), f"{path}.sample_rate", 1),
            vad_threshold=_as_number(
                _get(data, "vad_threshold", path), f"{path}.vad_threshold",
                0.0, 1.0),
            vad_rms_scale=_as_number(
                _get(data, "vad_rms_scale", path),
                f"{path}.vad_rms_scale", positive=True),
            vad_frame_ms=_as_int(
                _get(data, "vad_frame_ms", path),
                f"{path}.vad_frame_ms", 1),
            silence_ms=_as_int(
                _get(data, "silence_ms", path), f"{path}.silence_ms", 0),
            piper_executable=_as_str(
                _get(data, "piper_executable", path),
                f"{path}.piper_executable"),
            audio_player=_as_str(
                _get(data, "audio_player", path), f"{path}.audio_player"),
        )


# -------------------------------------------------------------------- dialogue
@dataclass(frozen=True)
class SttConfig:
    provider: str
    local_model: str
    local_device: str
    local_compute_type: str
    api_model: str
    api_base: str
    timeout_sec: float

    @classmethod
    def from_mapping(cls, data, path="dialogue.stt"):
        data = _section(data, path)
        _check_keys(data, (
            "provider", "local_model", "local_device", "local_compute_type",
            "api_model", "api_base",
            "timeout_sec"), path)
        return cls(
            provider=_choice(
                _get(data, "provider", path), PROVIDERS, f"{path}.provider"),
            local_model=_as_str(
                _get(data, "local_model", path), f"{path}.local_model"),
            local_device=_as_str(
                _get(data, "local_device", path), f"{path}.local_device"),
            local_compute_type=_as_str(
                _get(data, "local_compute_type", path),
                f"{path}.local_compute_type"),
            api_model=_as_str(
                _get(data, "api_model", path), f"{path}.api_model",
                allow_empty=True),
            api_base=_as_str(
                _get(data, "api_base", path), f"{path}.api_base",
                allow_empty=True),
            timeout_sec=_as_number(
                _get(data, "timeout_sec", path), f"{path}.timeout_sec",
                positive=True),
        )


@dataclass(frozen=True)
class TtsConfig:
    provider: str
    local_voice: str
    api_voice: str
    api_model: str
    api_base: str
    timeout_sec: float

    @classmethod
    def from_mapping(cls, data, path="dialogue.tts"):
        data = _section(data, path)
        _check_keys(data, (
            "provider", "local_voice", "api_voice", "api_model", "api_base",
            "timeout_sec"), path)
        return cls(
            provider=_choice(
                _get(data, "provider", path), PROVIDERS, f"{path}.provider"),
            local_voice=_as_str(
                _get(data, "local_voice", path), f"{path}.local_voice"),
            api_voice=_as_str(
                _get(data, "api_voice", path), f"{path}.api_voice",
                allow_empty=True),
            api_model=_as_str(
                _get(data, "api_model", path), f"{path}.api_model",
                allow_empty=True),
            api_base=_as_str(
                _get(data, "api_base", path), f"{path}.api_base",
                allow_empty=True),
            timeout_sec=_as_number(
                _get(data, "timeout_sec", path), f"{path}.timeout_sec",
                positive=True),
        )


@dataclass(frozen=True)
class LlmConfig:
    provider: str
    local_model: str
    local_base: str
    api_model: str
    api_base: str
    timeout_sec: float
    max_retries: int
    temperature: float

    @classmethod
    def from_mapping(cls, data, path="dialogue.llm"):
        data = _section(data, path)
        _check_keys(data, (
            "provider", "local_model", "local_base", "api_model", "api_base", "timeout_sec",
            "max_retries", "temperature"), path)
        return cls(
            provider=_choice(
                _get(data, "provider", path), PROVIDERS, f"{path}.provider"),
            local_model=_as_str(
                _get(data, "local_model", path), f"{path}.local_model"),
            local_base=_as_str(
                _get(data, "local_base", path), f"{path}.local_base"),
            api_model=_as_str(
                _get(data, "api_model", path), f"{path}.api_model",
                allow_empty=True),
            api_base=_as_str(
                _get(data, "api_base", path), f"{path}.api_base",
                allow_empty=True),
            timeout_sec=_as_number(
                _get(data, "timeout_sec", path), f"{path}.timeout_sec",
                positive=True),
            max_retries=_as_int(
                _get(data, "max_retries", path), f"{path}.max_retries", 0),
            temperature=_as_number(
                _get(data, "temperature", path), f"{path}.temperature",
                0.0, 2.0),
        )


@dataclass(frozen=True)
class DialogueConfig:
    languages: tuple
    default_input_mode: str
    greeting_reply: str
    greeting_en_reply: str
    stt: SttConfig
    tts: TtsConfig
    llm: LlmConfig
    confirmation_required: bool
    max_turns_memory: int
    listening_timeout_sec: float
    unknown_location_reply: str
    unknown_information_reply: str
    transcript_topic: str
    response_topic: str
    state_topic: str

    @classmethod
    def from_mapping(cls, data, path="dialogue"):
        data = _section(data, path)
        _check_keys(data, (
            "languages", "default_input_mode", "greeting_reply", "greeting_en_reply",
            "stt", "tts", "llm", "confirmation_required",
            "max_turns_memory", "listening_timeout_sec",
            "unknown_location_reply", "unknown_information_reply",
            "transcript_topic", "response_topic", "state_topic"), path)
        languages = tuple(
            _as_str(langue, f"{path}.languages[{index}]")
            for index, langue in enumerate(
                _as_list(_get(data, "languages", path), f"{path}.languages")))
        if not languages:
            raise ConfigurationError(f"{path}.languages ne doit pas être vide.")
        return cls(
            languages=languages,
            default_input_mode=_choice(
                _get(data, "default_input_mode", path),
                ("microphone", "audio_file", "text"),
                f"{path}.default_input_mode"),
            greeting_reply=_as_str(
                _get(data, "greeting_reply", path),
                f"{path}.greeting_reply"),
            greeting_en_reply=_as_str(
                _get(data, "greeting_en_reply", path),
                f"{path}.greeting_en_reply"),
            stt=SttConfig.from_mapping(
                _get(data, "stt", path), f"{path}.stt"),
            tts=TtsConfig.from_mapping(
                _get(data, "tts", path), f"{path}.tts"),
            llm=LlmConfig.from_mapping(
                _get(data, "llm", path), f"{path}.llm"),
            confirmation_required=_as_bool(
                _get(data, "confirmation_required", path),
                f"{path}.confirmation_required"),
            max_turns_memory=_as_int(
                _get(data, "max_turns_memory", path),
                f"{path}.max_turns_memory", 0),
            listening_timeout_sec=_as_number(
                _get(data, "listening_timeout_sec", path),
                f"{path}.listening_timeout_sec", positive=True),
            unknown_location_reply=_as_str(
                _get(data, "unknown_location_reply", path),
                f"{path}.unknown_location_reply"),
            unknown_information_reply=_as_str(
                _get(data, "unknown_information_reply", path),
                f"{path}.unknown_information_reply"),
            transcript_topic=_as_str(
                _get(data, "transcript_topic", path),
                f"{path}.transcript_topic"),
            response_topic=_as_str(
                _get(data, "response_topic", path),
                f"{path}.response_topic"),
            state_topic=_as_str(
                _get(data, "state_topic", path), f"{path}.state_topic"),
        )


# --------------------------------------------------------------------- mission
@dataclass(frozen=True)
class MissionConfig:
    greeting_enabled: bool
    emergency_location: str
    default_wait_sec: float
    emergency_keywords: tuple

    @classmethod
    def from_mapping(cls, data, path="mission"):
        data = _section(data, path)
        _check_keys(data, (
            "greeting_enabled", "emergency_location", "default_wait_sec",
            "emergency_keywords"), path)
        keywords = tuple(
            _as_str(mot, f"{path}.emergency_keywords[{index}]")
            for index, mot in enumerate(
                _as_list(_get(data, "emergency_keywords", path),
                         f"{path}.emergency_keywords")))
        if not keywords:
            raise ConfigurationError(
                f"{path}.emergency_keywords ne doit pas être vide.")
        return cls(
            greeting_enabled=_as_bool(
                _get(data, "greeting_enabled", path),
                f"{path}.greeting_enabled"),
            emergency_location=_as_str(
                _get(data, "emergency_location", path),
                f"{path}.emergency_location"),
            default_wait_sec=_as_number(
                _get(data, "default_wait_sec", path),
                f"{path}.default_wait_sec", positive=True),
            emergency_keywords=keywords,
        )


# ------------------------------------------------------------- carte sémantique
@dataclass(frozen=True)
class Pose2DConfig:
    x: float
    y: float
    yaw: float

    @classmethod
    def from_mapping(cls, data, path):
        data = _section(data, path)
        _check_keys(data, ("x", "y", "yaw"), path)
        return cls(
            x=_as_number(_get(data, "x", path), f"{path}.x"),
            y=_as_number(_get(data, "y", path), f"{path}.y"),
            yaw=_as_number(_get(data, "yaw", path), f"{path}.yaw"),
        )


@dataclass(frozen=True)
class Location:
    name: str
    aliases: tuple
    tag_ids: tuple
    description: str
    category: str
    approach_pose: object
    approach_offset_m: float

    @classmethod
    def from_mapping(cls, data, path):
        data = _section(data, path)
        _check_keys(data, (
            "name", "aliases", "tag_ids", "description", "category",
            "approach_pose", "approach_offset_m"), path)
        aliases = tuple(
            _as_str(alias, f"{path}.aliases[{index}]")
            for index, alias in enumerate(
                _as_list(_get(data, "aliases", path, []), f"{path}.aliases")))
        tag_ids = tuple(
            _as_int(tag, f"{path}.tag_ids[{index}]", 0)
            for index, tag in enumerate(
                _as_list(_get(data, "tag_ids", path, []), f"{path}.tag_ids")))
        pose_data = _get(data, "approach_pose", path, None)
        pose = (None if pose_data is None
                else Pose2DConfig.from_mapping(pose_data, f"{path}.approach_pose"))
        return cls(
            name=_as_str(_get(data, "name", path), f"{path}.name"),
            aliases=aliases,
            tag_ids=tag_ids,
            description=_as_str(
                _get(data, "description", path, ""), f"{path}.description",
                allow_empty=True),
            category=_as_str(
                _get(data, "category", path), f"{path}.category",
                allow_empty=True),
            approach_pose=pose,
            approach_offset_m=_as_number(
                _get(data, "approach_offset_m", path),
                f"{path}.approach_offset_m", minimum=0.0),
        )


@dataclass(frozen=True)
class SemanticMapConfig:
    world: str
    frame: str
    locations: tuple
    locations_topic: str
    resolve_service: str
    by_tag_service: str

    @classmethod
    def from_mapping(cls, data, path="locations"):
        data = _section(data, path)
        _check_keys(data, (
            "world", "frame", "locations_topic", "resolve_service",
            "by_tag_service", "locations"), path)
        locations = tuple(
            Location.from_mapping(lieu, f"{path}.locations[{index}]")
            for index, lieu in enumerate(
                _as_list(_get(data, "locations", path), f"{path}.locations")))
        noms = [lieu.name for lieu in locations]
        if len(set(noms)) != len(noms):
            raise ConfigurationError(f"{path}: noms de lieux en double.")
        alias_vus = set()
        tags_vus = set()
        for lieu in locations:
            for alias in lieu.aliases:
                cle = _normaliser(alias)
                if cle in alias_vus or cle in {_normaliser(n) for n in noms}:
                    raise ConfigurationError(
                        f"{path}: alias en double ou identique à un nom : {alias}")
                alias_vus.add(cle)
            for tag in lieu.tag_ids:
                if tag in tags_vus:
                    raise ConfigurationError(
                        f"{path}: tag {tag} associé à plusieurs lieux.")
                tags_vus.add(tag)
        return cls(
            world=_as_str(_get(data, "world", path), f"{path}.world"),
            frame=_as_str(_get(data, "frame", path), f"{path}.frame"),
            locations_topic=_as_str(
                _get(data, "locations_topic", path),
                f"{path}.locations_topic"),
            resolve_service=_as_str(
                _get(data, "resolve_service", path),
                f"{path}.resolve_service"),
            by_tag_service=_as_str(
                _get(data, "by_tag_service", path),
                f"{path}.by_tag_service"),
            locations=locations,
        )

    def resolve(self, requete):
        """Retrouve un lieu par nom ou alias, sans accents ni casse."""
        cle = _normaliser(requete)
        for lieu in self.locations:
            if cle == _normaliser(lieu.name):
                return lieu
        for lieu in self.locations:
            if cle in {_normaliser(alias) for alias in lieu.aliases}:
                return lieu
        return None

    def by_tag(self, tag_id):
        for lieu in self.locations:
            if tag_id in lieu.tag_ids:
                return lieu
        return None


# --------------------------------------------------------------- connaissances
@dataclass(frozen=True)
class ProgramEntry:
    time: str
    title: str
    location: str
    description: str

    @classmethod
    def from_mapping(cls, data, path):
        data = _section(data, path)
        _check_keys(data, ("time", "title", "location", "description"), path)
        return cls(
            time=_as_str(_get(data, "time", path), f"{path}.time"),
            title=_as_str(_get(data, "title", path), f"{path}.title"),
            location=_as_str(
                _get(data, "location", path), f"{path}.location",
                allow_empty=True),
            description=_as_str(
                _get(data, "description", path, ""), f"{path}.description",
                allow_empty=True),
        )


@dataclass(frozen=True)
class FaqEntry:
    question: str
    answer: str
    keywords: tuple

    @classmethod
    def from_mapping(cls, data, path):
        data = _section(data, path)
        _check_keys(data, ("question", "answer", "keywords"), path)
        keywords = tuple(
            _as_str(mot, f"{path}.keywords[{index}]")
            for index, mot in enumerate(
                _as_list(_get(data, "keywords", path, []),
                         f"{path}.keywords")))
        return cls(
            question=_as_str(
                _get(data, "question", path), f"{path}.question"),
            answer=_as_str(_get(data, "answer", path), f"{path}.answer"),
            keywords=keywords,
        )


@dataclass(frozen=True)
class KnowledgeConfig:
    world: str
    program: tuple
    faq: tuple

    @classmethod
    def from_mapping(cls, data, path="knowledge"):
        data = _section(data, path)
        _check_keys(data, ("world", "program", "faq"), path)
        return cls(
            world=_as_str(_get(data, "world", path), f"{path}.world"),
            program=tuple(
                ProgramEntry.from_mapping(entree, f"{path}.program[{index}]")
                for index, entree in enumerate(
                    _as_list(_get(data, "program", path, []),
                             f"{path}.program"))),
            faq=tuple(
                FaqEntry.from_mapping(entree, f"{path}.faq[{index}]")
                for index, entree in enumerate(
                    _as_list(_get(data, "faq", path, []), f"{path}.faq"))),
        )

    def find_answer(self, question):
        """Réponse FAQ la plus probable, ou None. Aucune invention."""
        mots = set(_normaliser(question).split())
        meilleur = None
        meilleur_score = 0
        for entree in self.faq:
            score = sum(
                1 for mot in entree.keywords
                if _normaliser(mot) in mots)
            if score > meilleur_score:
                meilleur, meilleur_score = entree, score
        return None if meilleur is None else meilleur.answer


# --------------------------------------------------------------------- prompts
@dataclass(frozen=True)
class PromptExample:
    question: str
    intent: str
    location: str

    @classmethod
    def from_mapping(cls, data, path):
        data = _section(data, path)
        _check_keys(data, ("question", "intent", "location"), path)
        return cls(
            question=_as_str(
                _get(data, "question", path), f"{path}.question"),
            intent=_as_str(_get(data, "intent", path), f"{path}.intent"),
            location=_as_str(
                _get(data, "location", path), f"{path}.location",
                allow_empty=True),
        )


@dataclass(frozen=True)
class PromptsConfig:
    system: str
    intentions: tuple
    rules: tuple
    examples: tuple
    fallback_keywords: dict
    responses: dict

    @classmethod
    def from_mapping(cls, data, path="prompts"):
        data = _section(data, path)
        _check_keys(data, (
            "system", "intentions", "rules", "examples",
            "fallback_keywords", "responses"), path)
        intentions = tuple(
            _as_str(intention, f"{path}.intentions[{index}]")
            for index, intention in enumerate(
                _as_list(_get(data, "intentions", path),
                         f"{path}.intentions")))
        rules = tuple(
            _as_str(regle, f"{path}.rules[{index}]")
            for index, regle in enumerate(
                _as_list(_get(data, "rules", path), f"{path}.rules")))
        if not intentions or not rules:
            raise ConfigurationError(
                f"{path}: intentions et rules ne doivent pas être vides.")
        fallback_data = _section(
            _get(data, "fallback_keywords", path),
            f"{path}.fallback_keywords")
        fallback_keys = (
            "yes", "no", "stop", "navigation", "localization",
            "greeting", "farewell")
        _check_keys(fallback_data, fallback_keys, f"{path}.fallback_keywords")
        fallback_keywords = {
            key: tuple(
                _as_str(word, f"{path}.fallback_keywords.{key}[{index}]")
                for index, word in enumerate(_as_list(
                    _get(fallback_data, key, f"{path}.fallback_keywords"),
                    f"{path}.fallback_keywords.{key}")))
            for key in fallback_keys
        }
        if any(not values for values in fallback_keywords.values()):
            raise ConfigurationError(
                f"{path}.fallback_keywords: chaque groupe doit être renseigné.")
        response_data = _section(
            _get(data, "responses", path), f"{path}.responses")
        responses = {
            key: _as_str(value, f"{path}.responses.{key}")
            for key, value in response_data.items()
        }
        if not responses:
            raise ConfigurationError(f"{path}.responses ne doit pas être vide.")
        return cls(
            system=_as_str(_get(data, "system", path), f"{path}.system"),
            intentions=intentions,
            rules=rules,
            examples=tuple(
                PromptExample.from_mapping(
                    exemple, f"{path}.examples[{index}]")
                for index, exemple in enumerate(
                    _as_list(_get(data, "examples", path, []),
                             f"{path}.examples"))),
            fallback_keywords=fallback_keywords,
            responses=responses,
        )


# -------------------------------------------------------------------- secrets
@dataclass(frozen=True)
class Secrets:
    openai_api_key: str | None
    elevenlabs_api_key: str | None
    fosa_llm_api_key: str | None
    fosa_stt_api_key: str | None
    fosa_tts_api_key: str | None


# ------------------------------------------------------------------ racine
@dataclass(frozen=True)
class FosaConfig:
    system: SystemConfig
    navigation: NavigationConfig
    perception: PerceptionConfig
    audio: AudioConfig
    dialogue: DialogueConfig
    mission: MissionConfig
    semantic_map: SemanticMapConfig
    knowledge: KnowledgeConfig
    prompts: PromptsConfig
    secrets: Secrets
    course: CourseConfig
    start_word: StartWordConfig
