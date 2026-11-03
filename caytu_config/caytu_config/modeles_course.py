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

ORDER_TYPES = ("string", "int32_multi_array")
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
    global_costmap: bool

    @classmethod
    def from_mapping(cls, data, path="navigation.obstacle_sources[]"):
        data = _section(data, path)
        _check_keys(data, (
            "name", "topic", "enabled", "obstacle_min_range",
            "obstacle_max_range", "raytrace_max_range", "global_costmap"), path)
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
class StatusTopicsConfig:
    """Topics de statut (départ, visite, image, fin) lus par les juges."""

    start_topic: str
    visit_topic: str
    image_topic: str
    finish_topic: str

    @classmethod
    def from_mapping(cls, data, path="course.status"):
        data = _section(data, path)
        cles = ("start_topic", "visit_topic", "image_topic", "finish_topic")
        _check_keys(data, cles, path)
        return cls(**{
            cle: _as_str(_get(data, cle, path), f"{path}.{cle}") for cle in cles})


@dataclass(frozen=True)
class AlignmentConfig:
    """Asservissement visuel final devant un tag."""

    linear_gain: float
    angular_gain: float
    max_linear_mps: float
    max_angular_rps: float
    timeout_sec: float
    max_renavigations: int

    @classmethod
    def from_mapping(cls, data, path="course.alignment"):
        data = _section(data, path)
        _check_keys(data, (
            "linear_gain", "angular_gain", "max_linear_mps", "max_angular_rps",
            "timeout_sec", "max_renavigations"), path)
        return cls(
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
            timeout_sec=_as_number(
                _get(data, "timeout_sec", path), f"{path}.timeout_sec",
                positive=True),
            max_renavigations=_as_int(
                _get(data, "max_renavigations", path),
                f"{path}.max_renavigations", 0),
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
    ("hold_sec", {"minimum": 0.0}),
    ("min_tag_budget_sec", {"positive": True}),
    ("detection_max_age_sec", {"positive": True}),
    ("search_spin_time_sec", {"positive": True}),
    ("search_grid_step_m", {"positive": True}),
    ("search_clearance_m", {"positive": True}),
    ("search_visited_radius_m", {"positive": True}),
    ("front_guard_m", {"minimum": 0.0}),
)
_TEXTES_COURSE = ("order_topic", "start_topic", "state_topic")
_CHEMINS_COURSE = ("slam_map_file", "memorized_tags_file", "tags_output_file")


@dataclass(frozen=True)
class CourseConfig:
    """Déroulement de la course officielle (ordre, visites, recherche)."""

    tag_ids: tuple
    order_topic: str
    order_type: str
    start_topic: str
    state_topic: str
    status: StatusTopicsConfig
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
    min_tag_budget_sec: float
    detection_max_age_sec: float
    search_spin_time_sec: float
    search_grid_step_m: float
    search_clearance_m: float
    search_visited_radius_m: float
    front_guard_m: float
    alignment: AlignmentConfig
    slam_map_file: str
    memorized_tags_file: str
    tags_output_file: str

    @classmethod
    def from_mapping(cls, data, path="course"):
        data = _section(data, path)
        _check_keys(data, (
            "tag_ids", "order_type", "status", "stable_cycles", "alignment")
            + _TEXTES_COURSE + _CHEMINS_COURSE
            + tuple(cle for cle, _ in _NOMBRES_COURSE), path)
        valeurs = {
            cle: _as_number(_get(data, cle, path), f"{path}.{cle}", **bornes)
            for cle, bornes in _NOMBRES_COURSE}
        valeurs.update({
            cle: _as_str(_get(data, cle, path), f"{path}.{cle}")
            for cle in _TEXTES_COURSE})
        valeurs.update({
            cle: _as_str(_get(data, cle, path), f"{path}.{cle}",
                         allow_empty=True)
            for cle in _CHEMINS_COURSE})
        course = cls(
            tag_ids=_as_int_list(_get(data, "tag_ids", path), f"{path}.tag_ids"),
            order_type=_choice(
                _get(data, "order_type", path), ORDER_TYPES,
                f"{path}.order_type"),
            status=StatusTopicsConfig.from_mapping(
                _get(data, "status", path), f"{path}.status"),
            stable_cycles=_as_int(
                _get(data, "stable_cycles", path), f"{path}.stable_cycles", 1),
            alignment=AlignmentConfig.from_mapping(
                _get(data, "alignment", path), f"{path}.alignment"),
            **valeurs,
        )
        if course.stop_margin_sec >= course.time_limit_sec:
            raise ConfigurationError(
                f"{path}: stop_margin_sec doit être inférieur à time_limit_sec.")
        if course.approach_distance_m < course.visit_distance_m:
            raise ConfigurationError(
                f"{path}: approach_distance_m doit être >= visit_distance_m.")
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
    model_path: str
    min_confidence: float

    @classmethod
    def from_mapping(cls, data, path="start_word"):
        data = _section(data, path)
        _check_keys(data, (
            "enabled", "audio_topic", "mode", "sample_rate", "channels",
            "channel_index", "sample_width", "words", "model_path",
            "min_confidence"), path)
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
