"""Paramètres injectés au lancement (SLAM)."""

from dataclasses import replace
from pathlib import Path

import pytest

from caytu_config import load_config
from caytu_course.lancement import executable_slam, parametres_slam

CONFIG_FOSA = Path(__file__).resolve().parents[2] / 'caytu_config' / 'config'


@pytest.fixture
def robot(monkeypatch):
    monkeypatch.delenv('FOSA_CONFIG_OVERRIDE', raising=False)
    return load_config(CONFIG_FOSA)


def test_cartographie_en_direct_par_defaut(robot):
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


def test_localisation_sur_une_carte_des_essais(robot):
    course = replace(robot.course, slam_map_file='~/fosa/cartes/arene.posegraph')
    parametres = parametres_slam(replace(robot, course=course))
    assert parametres['mode'] == 'localization'
    assert parametres['map_file_name'] == str(Path('~/fosa/cartes/arene').expanduser())
    assert parametres['map_start_pose'] == [0.0, 0.0, 0.0]
    assert executable_slam(replace(robot, course=course)) == 'localization_slam_toolbox_node'
