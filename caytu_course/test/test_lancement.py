"""Paramètres injectés au lancement (SLAM) selon le mode et la préparation."""

from dataclasses import replace
from pathlib import Path

import pytest

from caytu_config import load_config
from caytu_course.lancement import carte_preparee, executable_slam, parametres_slam

CONFIG_FOSA = Path(__file__).resolve().parents[2] / 'caytu_config' / 'config'


@pytest.fixture
def robot(monkeypatch, tmp_path):
    monkeypatch.delenv('FOSA_CONFIG_OVERRIDE', raising=False)
    configuration = load_config(CONFIG_FOSA)
    preparation = replace(configuration.course.preparation, directory=str(tmp_path))
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


def test_une_carte_incomplete_est_ignoree(robot):
    (Path(robot.course.preparation.directory) / 'carte.posegraph').write_bytes(b'x')
    assert carte_preparee(robot) is None
