"""Schémas de la course de la phase réelle (tags, statut, départ vocal).

Les noms de clés suivent le reste de `fosa.yaml` (anglais) ; les valeurs
viennent uniquement du YAML. Les topics des organisateurs encore inconnus sont
isolés ici pour être changés sans toucher au code.
"""

from dataclasses import dataclass

from caytu_config.errors import ConfigurationError
from caytu_config.validation import (
    _as_bool,
    _as_int,
    _as_list,
    _as_number,
    _as_str,
    _check_keys,
    _choice,
    _get,
    _section,
)

ORDER_TYPES = ("auto", "string", "int32_multi_array")
DEPTH_INPUTS = ("depth_image", "pointcloud")
AUDIO_MODES = ("utterance", "stream")
# Règlement : visite valide si le robot est aligné à ±12° près.
REGLEMENT_TOLERANCE_CAP_DEG = 12.0


def _as_int_list(value, path, minimum=0):
    entiers = tuple(
        _as_int(element, f"{path}[{index}]", minimum)
        for index, element in enumerate(_as_list(value, path)))
    if not entiers:
        raise ConfigurationError(f"{path} ne doit pas être vide.")
    if len(set(entiers)) != len(entiers):
        raise ConfigurationError(f"{path} contient des doublons.")
    return entiers


def _as_str_list(value, path):
    textes = tuple(
        _as_str(element, f"{path}[{index}]")
        for index, element in enumerate(_as_list(value, path)))
    if not textes:
        raise ConfigurationError(f"{path} ne doit pas être vide.")
    return textes


@dataclass(frozen=True)
class ObstacleSourceConfig:
    """Un scan (LaserScan) qui alimente les costmaps de Nav2."""

    name: str
    topic: str
    enabled: bool
    obstacle_min_range: float
    obstacle_max_range: float
    raytrace_max_range: float
    inf_is_valid: bool
    global_costmap: bool

    @classmethod
    def from_mapping(cls, data, path="navigation.obstacle_sources[]"):
        data = _section(data, path)
        _check_keys(data, (
            "name", "topic", "enabled", "obstacle_min_range",
            "obstacle_max_range", "raytrace_max_range", "inf_is_valid",
            "global_costmap"), path)
        source = cls(
            name=_as_str(_get(data, "name", path), f"{path}.name"),
            topic=_as_str(_get(data, "topic", path), f"{path}.topic"),
            enabled=_as_bool(_get(data, "enabled", path), f"{path}.enabled"),
            obstacle_min_range=_as_number(
                _get(data, "obstacle_min_range", path),
                f"{path}.obstacle_min_range", minimum=0.0),
            obstacle_max_range=_as_number(
                _get(data, "obstacle_max_range", path),
                f"{path}.obstacle_max_range", positive=True),
            raytrace_max_range=_as_number(
                _get(data, "raytrace_max_range", path),
                f"{path}.raytrace_max_range", positive=True),
            inf_is_valid=_as_bool(
                _get(data, "inf_is_valid", path), f"{path}.inf_is_valid"),
            global_costmap=_as_bool(
                _get(data, "global_costmap", path), f"{path}.global_costmap"),
        )
        if source.obstacle_max_range >= source.raytrace_max_range:
            raise ConfigurationError(
                f"{path}: obstacle_max_range doit être inférieur à "
                "raytrace_max_range (sinon les rayons vides marquent un obstacle).")
        if source.obstacle_min_range >= source.obstacle_max_range:
            raise ConfigurationError(
                f"{path}: obstacle_min_range doit être inférieur à obstacle_max_range.")
        return source


@dataclass(frozen=True)
class DepthScanConfig:
    """Profondeur d'une caméra -> LaserScan d'obstacles entre deux hauteurs."""

    name: str
    input_type: str
    input_topic: str
    info_topic: str
    output_topic: str
    min_height: float
    max_height: float
    range_min: float
    range_max: float
    angle_min: float
    angle_max: float
    angle_increment: float
    pixel_step: int
    min_points: int
    rate_hz: float

    @classmethod
    def from_mapping(cls, data, path="perception.depth_scans[]"):
        data = _section(data, path)
        nombres = ("min_height", "max_height", "range_min", "range_max",
                   "angle_min", "angle_max", "angle_increment", "rate_hz")
        _check_keys(data, ("name", "input_type", "input_topic", "info_topic",
                           "output_topic", "pixel_step", "min_points") + nombres, path)
        scan = cls(
            name=_as_str(_get(data, "name", path), f"{path}.name"),
            input_type=_choice(
                _get(data, "input_type", path), DEPTH_INPUTS, f"{path}.input_type"),
            input_topic=_as_str(
                _get(data, "input_topic", path), f"{path}.input_topic"),
            info_topic=_as_str(
                _get(data, "info_topic", path), f"{path}.info_topic",
                allow_empty=True),
            output_topic=_as_str(
                _get(data, "output_topic", path), f"{path}.output_topic"),
            pixel_step=_as_int(
                _get(data, "pixel_step", path), f"{path}.pixel_step", 1),
            min_points=_as_int(
                _get(data, "min_points", path), f"{path}.min_points", 1),
            **{cle: _as_number(_get(data, cle, path), f"{path}.{cle}")
               for cle in nombres},
        )
        if scan.input_type == "depth_image" and not scan.info_topic:
            raise ConfigurationError(f"{path}.info_topic est obligatoire pour une image.")
        for bas, haut in (("min_height", "max_height"), ("range_min", "range_max"),
                          ("angle_min", "angle_max")):
            if getattr(scan, bas) >= getattr(scan, haut):
                raise ConfigurationError(f"{path}: {bas} doit être inférieur à {haut}.")
        if scan.angle_increment <= 0.0 or scan.rate_hz <= 0.0:
            raise ConfigurationError(
                f"{path}: angle_increment et rate_hz doivent être positifs.")
        return scan


@dataclass(frozen=True)
class TagCameraConfig:
    """Caméra couleur utilisée pour détecter les tags."""

    name: str
    color_topic: str
    info_topic: str

    @classmethod
    def from_mapping(cls, data, path="perception.apriltag.cameras[]"):
        data = _section(data, path)
        _check_keys(data, ("name", "color_topic", "info_topic"), path)
        return cls(
            name=_as_str(_get(data, "name", path), f"{path}.name"),
            color_topic=_as_str(
                _get(data, "color_topic", path), f"{path}.color_topic"),
            info_topic=_as_str(
                _get(data, "info_topic", path), f"{path}.info_topic"),
        )


@dataclass(frozen=True)
class RefereeConfig:
    """Interface de l'arbitre officiel (`parc_msgs`, topics `/parc/*`)."""

    start_topic: str
    start_type: str
    start_method_command: int
    start_method_voice: int
    listen_start: bool
    visit_topic: str
    visit_type: str
    finish_topic: str
    finish_type: str
    finish_reason: int
    run_status_topic: str
    jpeg_quality: int

    @classmethod
    def from_mapping(cls, data, path="course.referee"):
        data = _section(data, path)
        textes = ("start_topic", "start_type", "visit_topic", "visit_type",
                  "finish_topic", "finish_type", "run_status_topic")
        entiers = ("start_method_command", "start_method_voice", "finish_reason")
        _check_keys(data, textes + entiers + ("listen_start", "jpeg_quality"), path)
        valeurs = {cle: _as_str(_get(data, cle, path), f"{path}.{cle}")
                   for cle in textes}
        valeurs.update({cle: _as_int(_get(data, cle, path), f"{path}.{cle}", 0, 255)
                        for cle in entiers})
        arbitre = cls(
            listen_start=_as_bool(
                _get(data, "listen_start", path), f"{path}.listen_start"),
            jpeg_quality=_as_int(
                _get(data, "jpeg_quality", path), f"{path}.jpeg_quality", 10, 100),
            **valeurs,
        )
        if arbitre.start_method_command == arbitre.start_method_voice:
            raise ConfigurationError(
                f"{path}: les méthodes de départ commande et voix doivent différer.")
        return arbitre


@dataclass(frozen=True)
class AlignmentConfig:
    """Asservissement visuel final devant un tag."""

    linear_gain: float
    angular_gain: float
    max_linear_mps: float
    max_angular_rps: float
    min_linear_mps: float
    min_angular_rps: float
    precision_point_m: float
    precision_heading_deg: float
    max_distance_error_m: float
    max_lateral_error_m: float
    timeout_sec: float
    max_renavigations: int

    @classmethod
    def from_mapping(cls, data, path="course.alignment"):
        data = _section(data, path)
        _check_keys(data, (
            "linear_gain", "angular_gain", "max_linear_mps", "max_angular_rps",
            "min_linear_mps", "min_angular_rps", "precision_point_m",
            "precision_heading_deg", "max_distance_error_m",
            "max_lateral_error_m", "timeout_sec", "max_renavigations"), path)
        alignement = cls(
            linear_gain=_as_number(
                _get(data, "linear_gain", path), f"{path}.linear_gain",
                positive=True),
            angular_gain=_as_number(
                _get(data, "angular_gain", path), f"{path}.angular_gain",
                positive=True),
            max_linear_mps=_as_number(
                _get(data, "max_linear_mps", path), f"{path}.max_linear_mps",
                positive=True, maximum=0.3),
            max_angular_rps=_as_number(
                _get(data, "max_angular_rps", path), f"{path}.max_angular_rps",
                positive=True, maximum=1.0),
            # Vitesses minimales : la base réelle ne bouge pas sous un seuil.
            min_linear_mps=_as_number(
                _get(data, "min_linear_mps", path), f"{path}.min_linear_mps",
                minimum=0.0),
            min_angular_rps=_as_number(
                _get(data, "min_angular_rps", path), f"{path}.min_angular_rps",
                minimum=0.0),
            precision_point_m=_as_number(
                _get(data, "precision_point_m", path),
                f"{path}.precision_point_m", positive=True),
            precision_heading_deg=_as_number(
                _get(data, "precision_heading_deg", path),
                f"{path}.precision_heading_deg", positive=True),
            max_distance_error_m=_as_number(
                _get(data, "max_distance_error_m", path),
                f"{path}.max_distance_error_m", positive=True),
            max_lateral_error_m=_as_number(
                _get(data, "max_lateral_error_m", path),
                f"{path}.max_lateral_error_m", positive=True),
            timeout_sec=_as_number(
                _get(data, "timeout_sec", path), f"{path}.timeout_sec",
                positive=True),
            max_renavigations=_as_int(
                _get(data, "max_renavigations", path),
                f"{path}.max_renavigations", 0),
        )
        if (alignement.min_linear_mps > alignement.max_linear_mps
                or alignement.min_angular_rps > alignement.max_angular_rps):
            raise ConfigurationError(
                f"{path}: les vitesses minimales dépassent les maximales.")
        return alignement


@dataclass(frozen=True)
class StrategyConfig:
    """Modèle de temps et de réussite de la stratégie de visite."""

    points_per_tag: float
    travel_speed_mps: float
    detour_factor: float
    visit_time_sec: float
    search_time_sec: float
    unknown_leg_m: float
    time_margin: float
    success_known: float
    success_unknown: float
    failure_decay: float
    min_skip_gain_points: float
    replan_period_sec: float

    _BORNES = {
        "points_per_tag": {"positive": True},
        "travel_speed_mps": {"positive": True, "maximum": 1.0},
        "detour_factor": {"minimum": 1.0},
        "visit_time_sec": {"positive": True},
        "search_time_sec": {"positive": True},
        "unknown_leg_m": {"positive": True},
        "time_margin": {"minimum": 0.0, "maximum": 2.0},
        "success_known": {"positive": True, "maximum": 1.0},
        "success_unknown": {"positive": True, "maximum": 1.0},
        "failure_decay": {"positive": True, "maximum": 1.0},
        "min_skip_gain_points": {"minimum": 0.0},
        "replan_period_sec": {"positive": True},
    }

    @classmethod
    def from_mapping(cls, data, path="course.strategy"):
        data = _section(data, path)
        _check_keys(data, tuple(cls._BORNES), path)
        return cls(**{
            cle: _as_number(_get(data, cle, path), f"{path}.{cle}", **bornes)
            for cle, bornes in cls._BORNES.items()})


@dataclass(frozen=True)
class PreparationConfig:
    """Période de préparation avant le match : carte et tags enregistrés."""

    directory: str
    carte_embarquee: str
    max_duration_sec: float
    close_view_m: float
    approach_attempts: int

    @classmethod
    def from_mapping(cls, data, path="course.preparation"):
        data = _section(data, path)
        _check_keys(data, ("directory", "carte_embarquee", "max_duration_sec",
                           "close_view_m", "approach_attempts"), path)
        return cls(
            directory=_as_str(_get(data, "directory", path), f"{path}.directory"),
            # Dossier ou préfixe de carte livré avec la solution. Vide : aucune
            # carte embarquée, le match utilise la préparation locale.
            carte_embarquee=_as_str(
                _get(data, "carte_embarquee", path, ""),
                f"{path}.carte_embarquee", allow_empty=True),
            max_duration_sec=_as_number(
                _get(data, "max_duration_sec", path), f"{path}.max_duration_sec",
                positive=True),
            close_view_m=_as_number(
                _get(data, "close_view_m", path), f"{path}.close_view_m",
                positive=True),
            approach_attempts=_as_int(
                _get(data, "approach_attempts", path), f"{path}.approach_attempts", 0),
        )


# (clé, bornes) des réglages numériques de la course.
_NOMBRES_COURSE = (
    ("time_limit_sec", {"positive": True}),
    ("stop_margin_sec", {"minimum": 0.0}),
    ("nav2_ready_timeout_sec", {"positive": True}),
    ("visit_distance_m", {"positive": True}),
    ("approach_distance_m", {"positive": True}),
    ("distance_tolerance_m", {"positive": True}),
    ("lateral_tolerance_m", {"positive": True}),
    ("heading_tolerance_deg", {
        "positive": True, "maximum": REGLEMENT_TOLERANCE_CAP_DEG - 1.0}),
    ("hold_sec", {"minimum": 3.0}),  # minimum du protocole parc_msgs/TagVisit
    ("detection_max_age_sec", {"positive": True}),
    ("image_max_age_sec", {"positive": True}),
    ("search_spin_time_sec", {"positive": True}),
    ("search_grid_step_m", {"positive": True}),
    ("search_clearance_m", {"positive": True}),
    ("search_frontier_margin_m", {"minimum": 0.0}),
    ("search_fallback_clearance_m", {"positive": True}),
    ("search_fallback_frontier_margin_m", {"minimum": 0.0}),
    ("search_visited_radius_m", {"positive": True}),
    ("front_guard_m", {"minimum": 0.0}),
    ("goal_xy_tolerance_m", {"positive": True}),
    ("goal_yaw_tolerance_deg", {"positive": True}),
    ("tag_lost_timeout_sec", {"positive": True}),
    ("preapproach_margin_m", {"minimum": 0.0}),
    ("search_spin_speed_rps", {"positive": True, "maximum": 1.0}),
    ("search_max_radius_m", {"positive": True}),
    ("unstuck_distance_m", {"positive": True, "maximum": 1.0}),
)
_TEXTES_COURSE = ("order_topic", "start_topic", "state_topic", "events_topic",
                  "map_topic")


@dataclass(frozen=True)
class CourseConfig:
    """Déroulement de la course officielle (ordre, visites, recherche)."""

    tag_ids: tuple
    order_topic: str
    order_type: str
    start_topic: str
    state_topic: str
    events_topic: str
    map_topic: str
    referee: RefereeConfig
    time_limit_sec: float
    stop_margin_sec: float
    nav2_ready_timeout_sec: float
    visit_distance_m: float
    approach_distance_m: float
    distance_tolerance_m: float
    lateral_tolerance_m: float
    heading_tolerance_deg: float
    stable_cycles: int
    hold_sec: float
    detection_max_age_sec: float
    image_max_age_sec: float
    search_spin_time_sec: float
    search_grid_step_m: float
    search_clearance_m: float
    search_frontier_margin_m: float
    search_fallback_clearance_m: float
    search_fallback_frontier_margin_m: float
    search_visited_radius_m: float
    front_guard_m: float
    goal_xy_tolerance_m: float
    goal_yaw_tolerance_deg: float
    tag_lost_timeout_sec: float
    preapproach_margin_m: float
    search_spin_speed_rps: float
    search_max_radius_m: float
    unstuck_distance_m: float
    unstuck_after_failures: int
    alignment: AlignmentConfig
    strategy: StrategyConfig
    preparation: PreparationConfig

    @classmethod
    def from_mapping(cls, data, path="course"):
        data = _section(data, path)
        _check_keys(data, (
            "tag_ids", "order_type", "referee", "stable_cycles", "alignment",
            "strategy", "preparation", "unstuck_after_failures")
            + _TEXTES_COURSE
            + tuple(cle for cle, _ in _NOMBRES_COURSE), path)
        valeurs = {
            cle: _as_number(_get(data, cle, path), f"{path}.{cle}", **bornes)
            for cle, bornes in _NOMBRES_COURSE}
        valeurs.update({
            cle: _as_str(_get(data, cle, path), f"{path}.{cle}")
            for cle in _TEXTES_COURSE})
        course = cls(
            tag_ids=_as_int_list(_get(data, "tag_ids", path), f"{path}.tag_ids"),
            order_type=_choice(
                _get(data, "order_type", path), ORDER_TYPES,
                f"{path}.order_type"),
            referee=RefereeConfig.from_mapping(
                _get(data, "referee", path), f"{path}.referee"),
            stable_cycles=_as_int(
                _get(data, "stable_cycles", path), f"{path}.stable_cycles", 1),
            unstuck_after_failures=_as_int(
                _get(data, "unstuck_after_failures", path),
                f"{path}.unstuck_after_failures", 1),
            alignment=AlignmentConfig.from_mapping(
                _get(data, "alignment", path), f"{path}.alignment"),
            strategy=StrategyConfig.from_mapping(
                _get(data, "strategy", path), f"{path}.strategy"),
            preparation=PreparationConfig.from_mapping(
                _get(data, "preparation", path), f"{path}.preparation"),
            **valeurs,
        )
        if course.stop_margin_sec >= course.time_limit_sec:
            raise ConfigurationError(
                f"{path}: stop_margin_sec doit être inférieur à time_limit_sec.")
        if course.approach_distance_m < course.visit_distance_m:
            raise ConfigurationError(
                f"{path}: approach_distance_m doit être >= visit_distance_m.")
        if course.search_fallback_clearance_m < course.search_clearance_m:
            raise ConfigurationError(
                f"{path}: search_fallback_clearance_m doit être >= "
                "search_clearance_m pour ne pas rapprocher les cibles des obstacles.")
        # Un tag jugé proche (sans Nav2) doit rester à portée de l'asservissement.
        if (course.approach_distance_m + course.preapproach_margin_m
                - course.visit_distance_m > course.alignment.max_distance_error_m):
            raise ConfigurationError(
                f"{path}: approach_distance_m + preapproach_margin_m - "
                "visit_distance_m dépasse alignment.max_distance_error_m.")
        return course


@dataclass(frozen=True)
class StartWordConfig:
    """Reconnaissance hors ligne du mot de départ (micros ReSpeaker)."""

    enabled: bool
    audio_topic: str
    mode: str
    sample_rate: int
    channels: int
    channel_index: int
    sample_width: int
    words: tuple
    restricted_grammar: bool
    model_path: str
    min_confidence: float

    @classmethod
    def from_mapping(cls, data, path="start_word"):
        data = _section(data, path)
        _check_keys(data, (
            "enabled", "audio_topic", "mode", "sample_rate", "channels",
            "channel_index", "sample_width", "words", "restricted_grammar",
            "model_path", "min_confidence"), path)
        voix = cls(
            enabled=_as_bool(_get(data, "enabled", path), f"{path}.enabled"),
            audio_topic=_as_str(
                _get(data, "audio_topic", path), f"{path}.audio_topic"),
            mode=_choice(_get(data, "mode", path), AUDIO_MODES, f"{path}.mode"),
            sample_rate=_as_int(
                _get(data, "sample_rate", path), f"{path}.sample_rate", 8000),
            channels=_as_int(
                _get(data, "channels", path), f"{path}.channels", 1),
            channel_index=_as_int(
                _get(data, "channel_index", path), f"{path}.channel_index", 0),
            # Vosk n'accepte que du PCM 16 bits.
            sample_width=_as_int(
                _get(data, "sample_width", path), f"{path}.sample_width", 2, 2),
            words=tuple(
                mot.casefold() for mot in _as_str_list(
                    _get(data, "words", path), f"{path}.words")),
            restricted_grammar=_as_bool(
                _get(data, "restricted_grammar", path),
                f"{path}.restricted_grammar"),
            model_path=_as_str(
                _get(data, "model_path", path), f"{path}.model_path"),
            min_confidence=_as_number(
                _get(data, "min_confidence", path), f"{path}.min_confidence",
                0.0, 1.0),
        )
        if voix.channel_index >= voix.channels:
            raise ConfigurationError(
                f"{path}: channel_index doit être inférieur à channels.")
        return voix
