"""Configuration unique de la solution Fosa.

Utilisation :

    from caytu_config import load_config
    configuration = load_config()

Toutes les valeurs modifiables vivent dans `config/fosa.yaml` (et le profil
`config/profils/simulation.yaml`). Les nœuds ne doivent jamais coder une
coordonnée, un seuil, un topic ou un nom de modèle en dur.
"""

from caytu_config.errors import ConfigurationError
from caytu_config.loader import config_directory, load_config
from caytu_config.modeles_course import (
    AlignmentConfig,
    CourseConfig,
    ObstacleSourceConfig,
    RefereeConfig,
    StartWordConfig,
    TagCameraConfig,
)
from caytu_config.models import (
    AprilTagConfig,
    FosaConfig,
    NavigationConfig,
    PerceptionConfig,
    SystemConfig,
)

__all__ = [
    "AlignmentConfig",
    "AprilTagConfig",
    "ConfigurationError",
    "CourseConfig",
    "FosaConfig",
    "NavigationConfig",
    "ObstacleSourceConfig",
    "PerceptionConfig",
    "RefereeConfig",
    "StartWordConfig",
    "SystemConfig",
    "TagCameraConfig",
    "config_directory",
    "load_config",
]
