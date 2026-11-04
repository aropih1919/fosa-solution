"""Entrées du nœud de course, avec un faux nœud ROS (sans rclpy.init)."""

from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('caytu_interfaces.msg')

from caytu_course import entrees_course  # noqa: E402
from caytu_course.entrees_course import EntreesCourse  # noqa: E402
from caytu_course.memoire_tags import MemoireTags  # noqa: E402


class _Journal:
    def __init__(self):
        self.messages = []

    def info(self, texte, **_):
        self.messages.append(('info', texte))

    warning = error = info


class _Minuterie:
    def __init__(self, rappel):
        self.rappel = rappel
        self.annulee = False

    def cancel(self):
        self.annulee = True


class _FauxNoeud:
    def __init__(self):
        self.journal = _Journal()
        self.abonnements = {}
        self.types_abonnes = {}
        self.minuteries = []
        self.graphe = []

    def get_logger(self):
        return self.journal

    def create_subscription(self, type_message, topic, rappel, qos):
        self.abonnements.setdefault(topic, []).append(rappel)
        self.types_abonnes.setdefault(topic, []).append(type_message)

    def create_timer(self, periode, rappel):
        minuterie = _Minuterie(rappel)
        self.minuteries.append(minuterie)
        return minuterie

    def get_topic_names_and_types(self):
        return self.graphe

    def avancer(self):
        for minuterie in self.minuteries:
            if not minuterie.annulee:
                minuterie.rappel()


def _configuration(order_type='string', listen_start=True):
    arbitre = SimpleNamespace(
        start_topic='/parc/start', start_type='parc_msgs/msg/StartSignal',
        start_method_command=0, start_method_voice=1, listen_start=listen_start,
        run_status_topic='/parc/run_status')
    return SimpleNamespace(
        course=SimpleNamespace(
            order_topic='/ordre', order_type=order_type, start_topic='/depart',
            map_topic='/map', tag_ids=(1, 4, 7, 9), referee=arbitre),
        system=SimpleNamespace(base_frame='base_link'),
        perception=SimpleNamespace(
            tags_topic='/tags',
            apriltag=SimpleNamespace(detection_image_topic='/tags/image')),
        navigation=SimpleNamespace(scan_topic='/scan'))


class _DepartArbitre:
    def __init__(self, method=0, heard=''):
        self.method = method
        self.heard = heard


def _entrees(attente, monkeypatch=None, **options):
    if monkeypatch is not None:
        types = {'parc_msgs/msg/StartSignal': _DepartArbitre}
        monkeypatch.setattr(entrees_course, 'charger_type', types.get)
    noeud = _FauxNoeud()
    entrees = EntreesCourse(
        noeud, _configuration(**options), MemoireTags(4.0), None,
        lambda: attente['valeur'])
    return noeud, entrees


def _publier(noeud, topic, texte):
    from std_msgs.msg import String
    for rappel in noeud.abonnements[topic]:
        rappel(String(data=texte))


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


def test_le_depart_vocal_garde_le_mot_entendu():
    attente = {'valeur': True}
    noeud, entrees = _entrees(attente)
    _publier(noeud, '/depart', '{"source": "voix", "entendu": "allez"}')
    assert (entrees.source_depart, entrees.mot_entendu) == ('voix', 'allez')
    assert entrees.depart_de_l_arbitre is False


def test_un_depart_publie_sur_le_topic_de_l_arbitre_lance_la_course(monkeypatch):
    attente = {'valeur': True}
    noeud, entrees = _entrees(attente, monkeypatch)
    for rappel in noeud.abonnements['/parc/start']:
        rappel(_DepartArbitre(method=1, heard='allez'))
    assert entrees.source_depart == 'voix'
    assert entrees.depart_de_l_arbitre is True
    _publier(noeud, '/depart', 'commande')
    assert entrees.source_depart == 'voix'


def test_le_topic_de_depart_de_l_arbitre_peut_etre_ignore(monkeypatch):
    noeud, _ = _entrees({'valeur': True}, monkeypatch, listen_start=False)
    assert '/parc/start' not in noeud.abonnements


def test_le_type_de_l_ordre_est_decouvert_sur_le_graphe():
    from std_msgs.msg import Int32MultiArray
    attente = {'valeur': True}
    noeud, entrees = _entrees(attente, order_type='auto')
    noeud.avancer()
    assert '/ordre' not in noeud.abonnements
    noeud.graphe = [('/ordre', ['std_msgs/msg/Int32MultiArray'])]
    noeud.avancer()
    assert noeud.types_abonnes['/ordre'] == [Int32MultiArray, Int32MultiArray]
    noeud.avancer()
    assert len(noeud.abonnements['/ordre']) == 2
    noeud.abonnements['/ordre'][0](Int32MultiArray(data=[9, 4, 7, 1]))
    assert entrees.ordre == (9, 4, 7, 1)
