"""Chargement de la configuration unique Fosa.

Ordre de résolution du dossier :
  1. `FOSA_CONFIG_DIR` (tests, déploiement sur mesure) ;
  2. `share/caytu_config/config` du paquet installé ;
  3. `config/` du dépôt (exécution directe sans installation).

Un fichier d'override (`FOSA_CONFIG_OVERRIDE`) et la variable
`FOSA_DEPLOYMENT` permettent d'adapter un poste sans modifier le YAML versionné.
"""

import os
from pathlib import Path

import yaml

from caytu_config.errors import ConfigurationError
from caytu_config.modeles_course import CourseConfig, StartWordConfig
from caytu_config.models import (
    FosaConfig,
    NavigationConfig,
    PerceptionConfig,
    SystemConfig,
)

FICHIER_CONFIG = "fosa.yaml"


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
    """Dossier contenant `fosa.yaml` et ses profils."""
    dossier = os.environ.get("FOSA_CONFIG_DIR")
    if dossier:
        return Path(dossier).expanduser().resolve()

    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("caytu_config")) / "config"
    except Exception:
        # Exécution depuis les sources, sans workspace installé.
        return Path(__file__).resolve().parents[1] / "config"


def _charger_fosa(dossier):
    fosa = _read_yaml(dossier / FICHIER_CONFIG)

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
        course = fosa["course"]
        start_word = fosa["start_word"]
    except KeyError as erreur:
        raise ConfigurationError(
            f"Section obligatoire absente de fosa.yaml : {erreur.args[0]}"
        ) from erreur

    configuration = FosaConfig(
        system=SystemConfig.from_mapping(systeme),
        navigation=NavigationConfig.from_mapping(navigation),
        perception=PerceptionConfig.from_mapping(perception),
        course=CourseConfig.from_mapping(course),
        start_word=StartWordConfig.from_mapping(start_word),
    )
    manquants = set(configuration.course.tag_ids) - set(
        configuration.perception.apriltag.ids)
    if manquants:
        raise ConfigurationError(
            f"course.tag_ids : tags {sorted(manquants)} absents de "
            "perception.apriltag.ids (jamais détectés).")
    portees = {scan.output_topic: scan.range_max
               for scan in configuration.perception.depth_scans}
    for source in configuration.navigation.obstacle_sources:
        if source.topic in portees and source.obstacle_max_range >= portees[source.topic]:
            raise ConfigurationError(
                f"navigation.obstacle_sources[{source.name}] : obstacle_max_range doit "
                f"être inférieur à range_max du scan {source.topic} "
                "(sinon un secteur vide serait marqué).")
    return configuration
