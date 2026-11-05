"""YAML Nav2 de la course généré depuis caytu_config (robot réel et simulation)."""

import math
from pathlib import Path

import pytest
import yaml

from caytu_config import load_config
from caytu_nav_solution.parametres_nav2 import (
    construire_parametres_course,
    ecrire_parametres_course,
)

RACINE = Path(__file__).resolve().parents[2]
CONFIG_NAV2 = RACINE / 'caytu_nav_bringup' / 'config'
CONFIG_FOSA = RACINE / 'caytu_config' / 'config'


@pytest.fixture
def robot(monkeypatch):
    monkeypatch.delenv('FOSA_CONFIG_OVERRIDE', raising=False)
    return load_config(CONFIG_FOSA)


@pytest.fixture
def simulation(monkeypatch):
    monkeypatch.setenv(
        'FOSA_CONFIG_OVERRIDE', str(CONFIG_FOSA / 'profils' / 'simulation.yaml'))
    return load_config(CONFIG_FOSA)


def _parametres(configuration):
    return construire_parametres_course(configuration, CONFIG_NAV2)


def _costmap(parametres, nom):
    return parametres[nom][nom]['ros__parameters']


def test_aucun_repere_ni_topic_de_simulation_sur_le_robot(robot):
    texte = yaml.safe_dump(_parametres(robot))
    assert 'base_footprint' not in texte
    assert "'/odom'" not in texte and ' /odom\n' not in texte
    assert '/scan_clean' not in texte
    assert 'top_camera' not in texte


def test_les_repere_et_topics_viennent_de_la_configuration(robot):
    parametres = _parametres(robot)
    assert parametres['bt_navigator']['ros__parameters']['robot_base_frame'] == 'base_link'
    assert parametres['bt_navigator']['ros__parameters']['odom_topic'] == (
        '/botwheel_explorer/odom')
    assert parametres['controller_server']['ros__parameters']['odom_topic'] == (
        '/botwheel_explorer/odom')
    assert _costmap(parametres, 'global_costmap')['global_frame'] == 'map'
    assert _costmap(parametres, 'local_costmap')['global_frame'] == 'odom'
    assert parametres['behavior_server']['ros__parameters']['robot_base_frame'] == 'base_link'


def test_les_couches_suivent_les_sources_actives(robot):
    parametres = _parametres(robot)
    globale = _costmap(parametres, 'global_costmap')
    locale = _costmap(parametres, 'local_costmap')
    couches = ['lidar_layer', 'camera1_layer', 'camera2_layer', 'inflation_layer']
    assert globale['plugins'] == ['static_layer'] + couches
    assert locale['plugins'] == couches
    source = locale['lidar_layer']['lidar']
    assert source['topic'] == '/scan_filtered'
    assert source['obstacle_min_range'] == pytest.approx(0.12)
    assert source['raytrace_max_range'] > source['obstacle_max_range']
    camera = locale['camera1_layer']['camera1']
    assert camera['topic'] == '/fosa/scan_camera1'
    assert camera['inf_is_valid'] is True


def test_l_empreinte_et_les_vitesses_sont_bornees(robot):
    parametres = _parametres(robot)
    locale = _costmap(parametres, 'local_costmap')
    assert yaml.safe_load(locale['footprint']) == [
        [0.22, 0.25], [0.22, -0.25], [-0.31, -0.25], [-0.31, 0.25]]
    assert locale['footprint_padding'] == pytest.approx(0.03)
    suivi = parametres['controller_server']['ros__parameters']['FollowPath']
    assert suivi['desired_linear_vel'] <= robot.navigation.max_linear_speed
    assert suivi['rotate_to_heading_angular_vel'] <= robot.navigation.max_angular_speed
    assert parametres['behavior_server']['ros__parameters']['max_rotational_vel'] <= (
        robot.course.search_spin_speed_rps)


def test_le_planificateur_garde_le_cap_demande(robot):
    """Revue de code : avec use_final_approach_orientation, Smac remplaçait le
    cap du but par celui du dernier segment ; le robot arrivait de biais."""
    planificateur = _parametres(robot)['planner_server']['ros__parameters']['GridBased']
    assert planificateur['use_final_approach_orientation'] is False


def test_le_lidar_n_efface_pas_avec_les_rayons_sans_retour(robot):
    locale = _costmap(_parametres(robot), 'local_costmap')
    assert locale['lidar_layer']['lidar']['inf_is_valid'] is False


def test_le_cap_final_est_controle(robot):
    parametres = _parametres(robot)
    verificateur = parametres['controller_server']['ros__parameters']['goal_checker']
    assert verificateur['xy_goal_tolerance'] == pytest.approx(0.08)
    assert verificateur['yaw_goal_tolerance'] == pytest.approx(math.radians(7.0))


def test_le_temps_simule_suit_le_profil(robot, simulation):
    assert all(
        bloc['ros__parameters']['use_sim_time'] is False
        for nom, bloc in _parametres(robot).items() if 'ros__parameters' in bloc)
    parametres = _parametres(simulation)
    assert parametres['planner_server']['ros__parameters']['use_sim_time'] is True
    assert _costmap(parametres, 'local_costmap')['robot_base_frame'] == 'base_footprint'
    assert _costmap(parametres, 'local_costmap')['lidar_layer']['lidar'][
        'obstacle_min_range'] == pytest.approx(0.12)


def test_le_fichier_ecrit_se_relit(robot, tmp_path):
    chemin = ecrire_parametres_course(robot, tmp_path / 'nav2.yaml', CONFIG_NAV2)
    assert yaml.safe_load(chemin.read_text(encoding='utf-8')) == _parametres(robot)
