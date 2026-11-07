"""Paramètres injectés au lancement (SLAM) selon le mode et la préparation."""

from dataclasses import replace
from pathlib import Path

import pytest

from caytu_config import load_config
from caytu_course.lancement import (
    MODES,
    carte_preparee,
    executable_slam,
    fichier_tags_match,
    parametres_slam,
    pose_depart,
)

CONFIG_FOSA = Path(__file__).resolve().parents[2] / 'caytu_config' / 'config'


@pytest.fixture
def robot(monkeypatch, tmp_path):
    monkeypatch.delenv('FOSA_CONFIG_OVERRIDE', raising=False)
    configuration = load_config(CONFIG_FOSA)
    preparation = replace(configuration.course.preparation, directory=str(tmp_path),
                          carte_embarquee='', map_start_pose=(0.0, 0.0, 0.0))
    return replace(configuration, course=replace(configuration.course, preparation=preparation))


def _enregistrer_carte(dossier):
    for extension in ('.posegraph', '.data'):
        (Path(dossier) / f'carte{extension}').write_bytes(b'x')


def test_sans_preparation_le_match_cartographie_en_direct(robot):
    parametres = parametres_slam(robot)
    assert parametres['mode'] == 'mapping'
    assert 'map_file_name' not in parametres
    assert parametres['base_frame'] == 'base_link'
    assert parametres['odom_frame'] == 'odom'
    assert parametres['map_frame'] == 'map'
    assert parametres['scan_topic'] == '/scan_filtered'
    assert parametres['use_sim_time'] is False
    assert parametres['use_lifecycle_manager'] is False
    assert executable_slam(robot) == 'async_slam_toolbox_node'


def test_le_match_se_localise_sur_la_carte_de_la_preparation(robot):
    _enregistrer_carte(robot.course.preparation.directory)
    parametres = parametres_slam(robot, 'course')
    assert parametres['mode'] == 'localization'
    assert parametres['map_file_name'] == str(
        Path(robot.course.preparation.directory) / 'carte')
    assert parametres['map_start_pose'] == [0.0, 0.0, 0.0]
    assert executable_slam(robot, 'course') == 'localization_slam_toolbox_node'


def test_la_preparation_refait_toujours_une_carte_neuve(robot):
    """Règle des organisateurs : la carte peut changer, on recartographie."""
    _enregistrer_carte(robot.course.preparation.directory)
    assert parametres_slam(robot, 'preparation')['mode'] == 'mapping'
    assert executable_slam(robot, 'preparation') == 'async_slam_toolbox_node'
    assert parametres_slam(robot, 'cartographie')['mode'] == 'mapping'


def test_la_preparation_manuelle_enregistre_avec_slam_en_mode_mapping(robot):
    assert 'preparation_manuelle' in MODES
    parametres = parametres_slam(robot, 'preparation_manuelle')
    assert parametres['mode'] == 'mapping'
    assert executable_slam(robot, 'preparation_manuelle') == 'async_slam_toolbox_node'


def test_une_carte_incomplete_est_ignoree(robot):
    (Path(robot.course.preparation.directory) / 'carte.posegraph').write_bytes(b'x')
    assert carte_preparee(robot) is None


def test_la_carte_embarquee_sert_si_aucune_preparation(robot, tmp_path):
    _enregistrer_carte(tmp_path)
    preparation = replace(robot.course.preparation, carte_embarquee=str(tmp_path))
    configuration = replace(robot, course=replace(robot.course, preparation=preparation))
    assert carte_preparee(configuration) == tmp_path / 'carte'
    assert parametres_slam(configuration)['map_file_name'] == str(tmp_path / 'carte')


def test_la_preparation_locale_est_prioritaire_sur_la_carte_embarquee(robot, tmp_path):
    _enregistrer_carte(tmp_path)
    _enregistrer_carte(robot.course.preparation.directory)
    preparation = replace(robot.course.preparation, carte_embarquee=str(tmp_path))
    configuration = replace(robot, course=replace(robot.course, preparation=preparation))
    assert carte_preparee(configuration) == Path(
        robot.course.preparation.directory) / 'carte'


def test_fosa_carte_force_la_carte(robot, tmp_path, monkeypatch):
    _enregistrer_carte(tmp_path)
    monkeypatch.setenv('FOSA_CARTE', str(tmp_path))
    assert carte_preparee(robot) == tmp_path / 'carte'


def test_les_tags_suivent_la_carte_utilisee(robot):
    _enregistrer_carte(robot.course.preparation.directory)
    assert fichier_tags_match(robot) == Path(
        robot.course.preparation.directory) / 'tags.yaml'


def test_la_carte_livree_est_complete():
    base = CONFIG_FOSA / 'cartes' / 'arene_2026_11_06' / 'carte'
    for extension in ('.posegraph', '.data', '.yaml', '.pgm'):
        assert base.with_suffix(extension).is_file(), extension


def test_la_carte_embarquee_relative_est_resolue(robot, monkeypatch):
    """Depuis la racine du dépôt, `cartes/...` doit être résolu avant is_dir."""
    monkeypatch.setenv('FOSA_CONFIG_DIR', str(CONFIG_FOSA))
    preparation = replace(
        robot.course.preparation, carte_embarquee='cartes/arene_2026_11_06')
    configuration = replace(
        robot, course=replace(robot.course, preparation=preparation))
    assert carte_preparee(configuration) == (
        CONFIG_FOSA / 'cartes' / 'arene_2026_11_06' / 'carte')


def test_un_dossier_avec_un_seul_posegraph_est_accepte(robot, tmp_path):
    dossier = tmp_path / 'arene'
    dossier.mkdir()
    for extension in ('.posegraph', '.data'):
        (dossier / f'map{extension}').write_bytes(b'x')
    preparation = replace(robot.course.preparation, directory=str(dossier))
    configuration = replace(
        robot, course=replace(robot.course, preparation=preparation))
    assert carte_preparee(configuration) == dossier / 'map'


def test_des_fichiers_de_carte_vides_sont_refuses(robot, tmp_path):
    dossier = tmp_path / 'arene'
    dossier.mkdir()
    (dossier / 'carte.posegraph').write_bytes(b'')
    (dossier / 'carte.data').write_bytes(b'x')
    preparation = replace(robot.course.preparation, directory=str(dossier))
    configuration = replace(
        robot, course=replace(robot.course, preparation=preparation))
    assert carte_preparee(configuration) is None


def test_fosa_carte_relative_est_resolue_depuis_le_courant(
        robot, tmp_path, monkeypatch):
    dossier = tmp_path / 'arene'
    dossier.mkdir()
    for extension in ('.posegraph', '.data'):
        (dossier / f'carte{extension}').write_bytes(b'x')
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv('FOSA_CARTE', 'arene')
    assert carte_preparee(robot) == dossier / 'carte'


def test_la_pose_de_depart_vient_de_la_configuration(robot):
    preparation = replace(robot.course.preparation,
                          map_start_pose=(1.0, -2.0, 0.5))
    configuration = replace(
        robot, course=replace(robot.course, preparation=preparation))
    assert pose_depart(configuration) == [1.0, -2.0, 0.5]
    _enregistrer_carte(configuration.course.preparation.directory)
    assert parametres_slam(configuration)['map_start_pose'] == [1.0, -2.0, 0.5]


def test_fosa_pose_depart_surcharge_la_configuration(robot, monkeypatch):
    monkeypatch.setenv('FOSA_POSE_DEPART', '1,2,0.5')
    assert pose_depart(robot) == [1.0, 2.0, 0.5]
    monkeypatch.setenv('FOSA_POSE_DEPART', '1 2')
    with pytest.raises(ValueError):
        pose_depart(robot)
    monkeypatch.setenv('FOSA_POSE_DEPART', 'x y z')
    with pytest.raises(ValueError):
        pose_depart(robot)
