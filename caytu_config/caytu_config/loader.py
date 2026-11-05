"""Chargement de la configuration unique Fosa.

Ordre de résolution du dossier :
  1. `FOSA_CONFIG_DIR` (tests, déploiement sur mesure) ;
  2. `share/caytu_config/config` du paquet installé ;
  3. `config/` du dépôt (exécution directe sans installation).

Un fichier d'override (`FOSA_CONFIG_OVERRIDE`) et la variable
`FOSA_DEPLOYMENT` permettent d'adapter un poste sans modifier le YAML versionné.
Les secrets ne sont jamais lus dans un YAML : uniquement dans l'environnement.
"""

import os
from pathlib import Path

import yaml

from caytu_config.errors import ConfigurationError
from caytu_config.models import (
    CourseConfig,
    AudioConfig,
    DialogueConfig,
    FosaConfig,
    KnowledgeConfig,
    MissionConfig,
    NavigationConfig,
    PerceptionConfig,
    PromptsConfig,
    Secrets,
    SemanticMapConfig,
    SystemConfig,
)

CONFIG_FILES = (
    "fosa.yaml", "locations.yaml", "knowledge.yaml", "prompts.yaml")


def _read_yaml(chemin):
    chemin = Path(chemin)
    try:
        texte = chemin.read_text(encoding="utf-8")
    except OSError as erreur:
        raise ConfigurationError(
            f"Fichier de configuration illisible : {chemin} ({erreur})"
        ) from erreur
    try:
        contenu = yaml.safe_load(texte)
    except yaml.YAMLError as erreur:
        raise ConfigurationError(
            f"YAML invalide dans {chemin} : {erreur}") from erreur
    if not isinstance(contenu, dict):
        raise ConfigurationError(
            f"{chemin} doit contenir un mapping YAML à la racine.")
    return contenu


def _fusion(base, surcharge):
    """Fusion récursive : les mappings se complètent, le reste est remplacé."""
    resultat = dict(base)
    for cle, valeur in surcharge.items():
        if isinstance(valeur, dict) and isinstance(resultat.get(cle), dict):
            resultat[cle] = _fusion(resultat[cle], valeur)
        else:
            resultat[cle] = valeur
    return resultat


def config_directory():
    """Dossier contenant les quatre fichiers YAML."""
    dossier = os.environ.get("FOSA_CONFIG_DIR")
    if dossier:
        return Path(dossier).expanduser().resolve()

    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("caytu_config")) / "config"
    except Exception:
        # Exécution depuis les sources, sans workspace installé.
        return Path(__file__).resolve().parents[1] / "config"


def _secrets():
    return Secrets(
        openai_api_key=os.environ.get("OPENAI_API_KEY") or None,
        elevenlabs_api_key=os.environ.get("ELEVENLABS_API_KEY") or None,
        fosa_llm_api_key=os.environ.get("FOSA_LLM_API_KEY") or None,
        fosa_stt_api_key=os.environ.get("FOSA_STT_API_KEY") or None,
        fosa_tts_api_key=os.environ.get("FOSA_TTS_API_KEY") or None,
    )


def _charger_fosa(dossier):
    fosa = _read_yaml(dossier / "fosa.yaml")

    override = os.environ.get("FOSA_CONFIG_OVERRIDE")
    if override:
        fosa = _fusion(fosa, _read_yaml(override))

    deployment = os.environ.get("FOSA_DEPLOYMENT")
    if deployment:
        systeme = fosa.setdefault("system", {})
        if not isinstance(systeme, dict):
            raise ConfigurationError("system doit être un mapping YAML.")
        systeme["deployment"] = deployment
    return fosa


def load_config(dossier=None):
    """Charge et valide l'ensemble de la configuration.

    Lève `ConfigurationError` avec le chemin exact de la clé fautive.
    """
    base = Path(dossier) if dossier is not None else config_directory()
    fosa = _charger_fosa(base)

    try:
        systeme = fosa["system"]
        navigation = fosa["navigation"]
        perception = fosa["perception"]
        audio = fosa["audio"]
        dialogue = fosa["dialogue"]
        mission = fosa["mission"]
        course = fosa["course"]
    except KeyError as erreur:
        raise ConfigurationError(
            f"Section obligatoire absente de fosa.yaml : {erreur.args[0]}"
        ) from erreur

    return FosaConfig(
        system=SystemConfig.from_mapping(systeme),
        navigation=NavigationConfig.from_mapping(navigation),
        perception=PerceptionConfig.from_mapping(perception),
        audio=AudioConfig.from_mapping(audio),
        dialogue=DialogueConfig.from_mapping(dialogue),
        mission=MissionConfig.from_mapping(mission),
        course=CourseConfig.from_mapping(course),
        semantic_map=SemanticMapConfig.from_mapping(
            _read_yaml(base / "locations.yaml")),
        knowledge=KnowledgeConfig.from_mapping(
            _read_yaml(base / "knowledge.yaml")),
        prompts=PromptsConfig.from_mapping(
            _read_yaml(base / "prompts.yaml")),
        secrets=_secrets(),
    )
