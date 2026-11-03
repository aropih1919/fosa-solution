"""Entrées du nœud de course, avec un faux nœud ROS (sans rclpy.init)."""

from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('caytu_interfaces.msg')

from caytu_course.entrees_course import EntreesCourse  # noqa: E402
from caytu_course.memoire_tags import MemoireTags  # noqa: E402


class _Journal:
    def __init__(self):
        self.messages = []

    def info(self, texte, **_):
        self.messages.append(('info', texte))

    warning = error = info


class _FauxNoeud:
    def __init__(self):
        self.journal = _Journal()
        self.abonnements = {}

    def get_logger(self):
        return self.journal

    def create_subscription(self, type_message, topic, rappel, qos):
        self.abonnements.setdefault(topic, []).append(rappel)


def _configuration():
    statut = SimpleNamespace(start_topic='/s', visit_topic='/v', image_topic='/i',
                             finish_topic='/f')
    return SimpleNamespace(
        course=SimpleNamespace(
            order_topic='/ordre', order_type='string', start_topic='/depart',
            map_topic='/map', tag_ids=(1, 4, 7, 9), status=statut),
        system=SimpleNamespace(base_frame='base_link'),
        perception=SimpleNamespace(
            tags_topic='/tags',
            apriltag=SimpleNamespace(detection_image_topic='/tags/image')),
        navigation=SimpleNamespace(scan_topic='/scan'))


def _entrees(attente):
    noeud = _FauxNoeud()
    entrees = EntreesCourse(
        noeud, _configuration(), MemoireTags(4.0), None, lambda: attente['valeur'])
    return noeud, entrees


def _publier(noeud, topic, texte):
    for rappel in noeud.abonnements[topic]:
        rappel(SimpleNamespace(data=texte))


def test_l_ordre_arrive_avant_le_depart():
    attente = {'valeur': True}
    noeud, entrees = _entrees(attente)
    _publier(noeud, '/ordre', '7,1,9,4')
    _publier(noeud, '/depart', 'commande')
    assert entrees.ordre == (7, 1, 9, 4)
    assert entrees.source_depart == 'commande'


def test_l_ordre_publie_apres_un_depart_par_commande_est_accepte():
    """Revue de code : l'ordre arrivant après le départ était refusé (0 point)."""
    attente = {'valeur': True}
    noeud, entrees = _entrees(attente)
    _publier(noeud, '/depart', 'commande')
    attente['valeur'] = False
    _publier(noeud, '/ordre', '7,1,9,4')
    assert entrees.ordre == (7, 1, 9, 4)


def test_un_nouvel_ordre_pendant_la_course_est_ignore():
    attente = {'valeur': True}
    noeud, entrees = _entrees(attente)
    _publier(noeud, '/ordre', '7,1,9,4')
    attente['valeur'] = False
    _publier(noeud, '/ordre', '1,4,7,9')
    assert entrees.ordre == (7, 1, 9, 4)


def test_un_depart_n_est_compte_qu_une_fois():
    attente = {'valeur': True}
    noeud, entrees = _entrees(attente)
    _publier(noeud, '/depart', 'voix')
    _publier(noeud, '/depart', 'commande')
    assert entrees.source_depart == 'voix'
