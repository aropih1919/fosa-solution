"""Tests du chargeur et des schémas de configuration Fosa."""

from pathlib import Path
import shutil

import pytest
import yaml

from caytu_config import ConfigurationError, load_config
from caytu_config.loader import config_directory
from caytu_config.models import (
    KnowledgeConfig,
    Location,
    SemanticMapConfig,
)

CONFIG = Path(__file__).resolve().parents[1] / "config"


def _copier_configuration(destination):
    destination.mkdir(parents=True, exist_ok=True)
    for fichier in CONFIG.glob("*.yaml"):
        shutil.copy(fichier, destination / fichier.name)
    return destination


def _modifier_yaml(chemin, modification):
    contenu = yaml.safe_load(chemin.read_text(encoding="utf-8"))
    modification(contenu)
    chemin.write_text(
        yaml.safe_dump(contenu, sort_keys=False, allow_unicode=True),
        encoding="utf-8")


def test_la_configuration_par_defaut_est_valide():
    configuration = load_config(CONFIG)
    assert configuration.system.deployment == "simulation"
    assert configuration.system.use_sim_time is True
    assert configuration.system.odom_frame == "odom"
    assert configuration.navigation.controller == "rpp"
    assert configuration.navigation.profile == "balanced"
    assert configuration.navigation.localization == "odom_imu"
    assert configuration.navigation.time_limit_sec == 600.0
    assert configuration.perception.apriltag.family == "36h11"
    assert configuration.perception.apriltag.size_m == 0.18
    assert configuration.perception.yolo.nms_iou_threshold == 0.45
    assert configuration.perception.yolo.max_detections == 100
    assert configuration.perception.tracking.image_cloud_sync_slop_sec == 0.08
    assert configuration.navigation.obstacle_retreat_enabled is False
    assert configuration.mission.greeting_enabled is False
    assert configuration.dialogue.llm.provider == "local"
    assert configuration.audio.sample_rate == 16000
    assert "fr" in configuration.dialogue.languages


def test_les_lieux_se_resolvent_par_nom_alias_et_tag():
    carte = load_config(CONFIG).semantic_map
    assert carte.resolve("accueil").name == "accueil"
    assert carte.resolve("réception").name == "accueil"
    assert carte.resolve("WC").name == "toilettes"
    assert carte.resolve("inconnu") is None
    assert carte.by_tag(13).name == "infirmerie"
    assert carte.by_tag(999) is None


def test_la_faq_ne_repond_que_si_elle_trouve_une_entree():
    connaissances = load_config(CONFIG).knowledge
    assert "15 heures" in connaissances.find_answer(
        "À quelle heure commence la finale ?")
    assert connaissances.find_answer("Quelle est la capitale du Sénégal ?") is None


def test_une_cle_inconnue_est_refusee(tmp_path):
    dossier = _copier_configuration(tmp_path / "config")

    def ajouter(contenu):
        contenu["navigation"]["vitesse_magique"] = 42

    _modifier_yaml(dossier / "fosa.yaml", ajouter)
    with pytest.raises(ConfigurationError, match="vitesse_magique"):
        load_config(dossier)


def test_une_cle_obligatoire_absente_est_refusee(tmp_path):
    dossier = _copier_configuration(tmp_path / "config")

    def supprimer(contenu):
        del contenu["navigation"]["time_limit_sec"]

    _modifier_yaml(dossier / "fosa.yaml", supprimer)
    with pytest.raises(ConfigurationError, match="time_limit_sec"):
        load_config(dossier)


def test_une_valeur_invalide_est_refusee(tmp_path):
    dossier = _copier_configuration(tmp_path / "config")

    def invalider(contenu):
        contenu["navigation"]["success_radius"] = -1.0

    _modifier_yaml(dossier / "fosa.yaml", invalider)
    with pytest.raises(ConfigurationError, match="success_radius"):
        load_config(dossier)


def test_un_secteur_arriere_trop_large_est_refuse(tmp_path):
    dossier = _copier_configuration(tmp_path / "config")

    def invalider(contenu):
        contenu["navigation"]["obstacle_rear_sector_half_angle_rad"] = 2.0

    _modifier_yaml(dossier / "fosa.yaml", invalider)
    with pytest.raises(ConfigurationError, match="obstacle_rear_sector_half_angle_rad"):
        load_config(dossier)


def test_une_vitesse_de_recul_superieure_a_la_limite_est_refusee(tmp_path):
    dossier = _copier_configuration(tmp_path / "config")

    def invalider(contenu):
        contenu["navigation"]["obstacle_retreat_speed_mps"] = 0.11

    _modifier_yaml(dossier / "fosa.yaml", invalider)
    with pytest.raises(ConfigurationError, match="obstacle_retreat_speed_mps"):
        load_config(dossier)


def test_un_alias_en_double_est_refuse():
    with pytest.raises(ConfigurationError, match="alias"):
        SemanticMapConfig.from_mapping({
            "world": "test",
            "frame": "map",
            "locations": [
                {"name": "a", "aliases": ["meme"]},
                {"name": "b", "aliases": ["meme"]},
            ],
        })


def test_un_tag_ne_peut_pas_designer_deux_lieux():
    with pytest.raises(ConfigurationError, match="tag"):
        SemanticMapConfig.from_mapping({
            "world": "test",
            "frame": "map",
            "locations": [
                {"name": "a", "tag_ids": [7]},
                {"name": "b", "tag_ids": [7]},
            ],
        })


def test_un_lieu_sans_pose_reste_utilisable_par_tag():
    lieu = Location.from_mapping(
        {"name": "accueil", "tag_ids": [10]}, "locations[0]")
    assert lieu.approach_pose is None
    assert lieu.tag_ids == (10,)


def test_l_override_est_fusionne_sans_ecraser_le_reste(tmp_path, monkeypatch):
    dossier = _copier_configuration(tmp_path / "config")
    override = tmp_path / "override.yaml"
    override.write_text(
        "navigation:\n  time_limit_sec: 120.0\n", encoding="utf-8")
    monkeypatch.setenv("FOSA_CONFIG_OVERRIDE", str(override))

    configuration = load_config(dossier)
    assert configuration.navigation.time_limit_sec == 120.0
    assert configuration.navigation.controller == "rpp"
    assert configuration.system.deployment == "simulation"


def test_le_mode_de_deploiement_est_surchargeable(tmp_path, monkeypatch):
    monkeypatch.setenv("FOSA_DEPLOYMENT", "remote_pc")
    configuration = load_config(CONFIG)
    assert configuration.system.deployment == "remote_pc"


def test_les_secrets_viennent_uniquement_de_l_environnement(monkeypatch):
    monkeypatch.setenv("OPENAI_API_KEY", "cle-de-test")
    monkeypatch.setenv("FOSA_LLM_API_KEY", "cle-locale")
    configuration = load_config(CONFIG)
    assert configuration.secrets.openai_api_key == "cle-de-test"
    assert configuration.secrets.fosa_llm_api_key == "cle-locale"
    assert configuration.secrets.elevenlabs_api_key is None


def test_aucun_secret_dans_les_yaml_versionnes():
    for chemin in CONFIG.glob("*.yaml"):
        texte = chemin.read_text(encoding="utf-8").lower()
        assert "api_key:" not in texte
        assert "sk-" not in texte


def test_le_dossier_est_surchargeable(monkeypatch, tmp_path):
    monkeypatch.setenv("FOSA_CONFIG_DIR", str(tmp_path))
    assert config_directory() == tmp_path.resolve()


def test_les_prompts_et_regles_sont_presents():
    prompts = load_config(CONFIG).prompts
    assert "navigation" in prompts.intentions
    assert "confirmation" in prompts.intentions
    assert prompts.rules
    assert prompts.examples
    assert prompts.system


def test_les_connaissances_sont_optionnelles_mais_validees():
    configuration = KnowledgeConfig.from_mapping({
        "world": "test", "program": [], "faq": []})
    assert configuration.find_answer("bonjour") is None
