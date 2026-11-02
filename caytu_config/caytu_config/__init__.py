"""Configuration unique de la solution Fosa.

Utilisation :

    from caytu_config import load_config
    configuration = load_config()

Toutes les valeurs modifiables vivent dans `config/fosa.yaml` (et les données
dans `locations.yaml`, `knowledge.yaml`, `prompts.yaml`). Les nœuds ne doivent
jamais coder une coordonnée, un seuil, un topic ou un nom de modèle en dur.
"""

from caytu_config.errors import ConfigurationError
from caytu_config.loader import config_directory, load_config
from caytu_config.models import (
    AprilTagConfig,
    AudioConfig,
    DialogueConfig,
    FaqEntry,
    FosaConfig,
    KnowledgeConfig,
    Location,
    LlmConfig,
    MissionConfig,
    NavigationConfig,
    PerceptionConfig,
    PersonConfig,
    Pose2DConfig,
    ProgramEntry,
    PromptExample,
    PromptsConfig,
    ScanConversionConfig,
    Secrets,
    SemanticMapConfig,
    SttConfig,
    SystemConfig,
    TtsConfig,
    YoloConfig,
)

__all__ = [
    "AprilTagConfig",
    "AudioConfig",
    "ConfigurationError",
    "DialogueConfig",
    "FaqEntry",
    "FosaConfig",
    "KnowledgeConfig",
    "Location",
    "LlmConfig",
    "MissionConfig",
    "NavigationConfig",
    "PerceptionConfig",
    "PersonConfig",
    "Pose2DConfig",
    "ProgramEntry",
    "PromptExample",
    "PromptsConfig",
    "ScanConversionConfig",
    "Secrets",
    "SemanticMapConfig",
    "SttConfig",
    "SystemConfig",
    "TtsConfig",
    "YoloConfig",
    "config_directory",
    "load_config",
]
