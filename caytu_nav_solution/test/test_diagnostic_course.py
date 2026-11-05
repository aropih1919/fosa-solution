"""Calculs du diagnostic de course (sans ROS)."""
import math

import numpy as np

from caytu_nav_solution import diagnostic_course as dc


def test_points_dans_zones_et_emprise():
    ranges = [float('inf')] * 360
    ranges[0] = 0.25                       # (0.35, 0) : zone de ralenti seulement
    ranges[60] = 0.20                      # (0.20, 0.17) : arrêt et emprise
    rotation, translation = np.eye(3), [0.1, 0.0, 0.2]
    xs, ys, r, a = dc.points_scan(ranges, 0.0, math.radians(1.0), 0.05, 12.0, rotation, translation)
    res = dc.analyser_points(xs, ys, r, a, dc.ZONES_DEFAUT, dc.EMPRISE_DEFAUT)
    assert res['zones'] == {'ZoneArret': 1, 'ZoneRalenti': 1}
    assert res['emprise'] == 1 and len(res['proches']) == 1


def test_costmap_letal_sous_le_robot_et_orientation():
    grille = np.zeros((80, 80), dtype=int)
    grille[40, 43] = 100
    res = dc.analyser_costmap(grille, 0.05, (-2.0, -2.0, 0.0), (0.0, 0.0, 0.0), dc.EMPRISE_DEFAUT)
    assert res['letal_emprise'] == 1 and res['centre'] == 0 and res['letal_proche'][1] > 0
    res = dc.analyser_costmap(grille, 0.05, (-2.0, -2.0, 0.0), (0.0, 0.0, math.pi / 2),
                              dc.EMPRISE_DEFAUT)
    assert res['letal_proche'][2] < 0      # la cellule est à droite du robot tourné


def test_messages_nav2_reconnus():
    assert dc.indice_message('Failed to make progress') == 'pas_de_progres'
    assert dc.indice_message('Start occupied') == 'depart_occupe'
    assert dc.indice_message('failed to create plan with tolerance 0.50') == 'pas_de_chemin'
    assert dc.indice_message('tout va bien') is None


def test_polygones_parametres():
    attendu = [(0.3, 0.2), (0.3, -0.2), (-0.3, -0.2)]
    assert dc.parse_polygone('[[0.3, 0.2], [0.3, -0.2], [-0.3, -0.2]]') == attendu
    assert dc.parse_polygone([0.3, 0.2, 0.3, -0.2, -0.3, -0.2]) == attendu


def _periode(**changements):
    p = {
        'nav': {'actif': True}, 'chaine': {'brut': '/b', 'sortie': '/s', 'base': None},
        'cmd': {'/b': {'part': 1.0}, '/s': {'part': 0.0}},
        'odom': {'v': 0.0, 'w': 0.0, 'deplacement': 0.0, 'rotation': 0.0},
        'garde_fou': {'durees': {}, 'actuel': 'ARRÊT ZoneArret'}, 'sources_garde_fou': ['/scan_clean'],
        'scans': {'/scan_clean': {'analyses': 3, 'zones': {'ZoneArret': 5}, 'emprise_max': 0}},
        'costmaps': {}, 'loc': {'sauts': 0, 'tf_map_manquante': 0, 'echantillons': 10},
        'progres': 0.05, 'progres_rayon': 0.25, 'progres_delai': 15.0,
        'bt': {'recovery': False}, 'manquants': [], 'muets': [], 'controleur_odom': None,
        'conflits': [], 'inactifs': [], 'temps': [],
    }
    p.update(changements)
    return p


def test_verdict_garde_fou_et_progres():
    cles = [v[0] for v in dc.diagnostiquer(_periode())]
    assert 'garde_fou_obstacle' in cles and 'pas_de_progres' in cles


def test_verdict_organisateurs_puis_base():
    p = _periode(chaine={'brut': '/b', 'sortie': '/s', 'base': '/base'},
                 cmd={'/b': {'part': 1.0}, '/s': {'part': 1.0}, '/base': {'part': 0.0}},
                 garde_fou={'durees': {'libre': 5.0}, 'actuel': 'libre'})
    assert 'organisateurs' in [v[0] for v in dc.diagnostiquer(p)]
    p['cmd']['/base'] = {'part': 1.0}
    assert 'base_immobile' in [v[0] for v in dc.diagnostiquer(p)]


def test_aucun_attribut_ne_masque_ceux_de_rclpy_node():
    # Le nœud de diagnostic hérite de rclpy.Node : un attribut du même nom qu'un
    # attribut interne (self._clients, une liste dans rclpy) casse create_client.
    import pathlib
    import re
    internes = {'_publishers', '_subscriptions', '_clients', '_services', '_timers', '_guards',
                '_parameters', '_descriptors', '_clock', '_context', '_logger', '_time_source',
                '_rate_group', '_default_callback_group', '_parameter_overrides',
                '_parameter_service', '_parameter_event_publisher', '_type_description_service',
                '_logger_service', '_allow_undeclared_parameters'}
    source = pathlib.Path(dc.__file__).read_text(encoding='utf-8')
    utilises = set(re.findall(r'self\.(_[a-z_]+)\s*=', source))
    assert not utilises & internes, utilises & internes
