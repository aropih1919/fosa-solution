"""Entrées du nœud de course, avec un faux nœud ROS (sans rclpy.init)."""

import math
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


class _Horloge:
    def __init__(self):
        self.secondes = 100.0

    def now(self):
        return SimpleNamespace(nanoseconds=int(self.secondes * 1e9))


class _FauxNoeud:
    def __init__(self):
        self.horloge = _Horloge()
        self.journal = _Journal()
        self.abonnements = {}
        self.types_abonnes = {}
        self.minuteries = []
        self.graphe = []

    def get_logger(self):
        return self.journal

    def get_clock(self):
        return self.horloge

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
            map_topic='/map', tag_ids=(1, 4, 7, 9), referee=arbitre, image_max_age_sec=1.0),
        system=SimpleNamespace(base_frame='base_link'),
        perception=SimpleNamespace(
            tags_topic='/tags',
            apriltag=SimpleNamespace(detection_image_topic='/tags/image')),
        navigation=SimpleNamespace(
            scan_topic='/scan', mode='carte', odom_frame='odom'))


class _DepartArbitre:
    def __init__(self, method=0, heard=''):
        self.method = method
        self.heard = heard


class _StatutArbitre:
    IDLE, ARMED, RUNNING, FINISHED, TIMED_OUT, TERMINATED = range(6)

    def __init__(self, state, tags_reported=(), tag_order=()):
        self.state = state
        self.tags_reported = list(tags_reported)
        self.tag_order = list(tag_order)


def _entrees(attente, monkeypatch=None, memoire_tf=None, **options):
    if monkeypatch is not None:
        types = {'parc_msgs/msg/StartSignal': _DepartArbitre}
        monkeypatch.setattr(entrees_course, 'charger_type', types.get)
    noeud = _FauxNoeud()
    memoire_odom = MemoireTags(4.0, repere='odom')
    entrees = EntreesCourse(
        noeud, _configuration(**options), MemoireTags(4.0), memoire_tf,
        lambda: attente['valeur'], memoire_odom=memoire_odom)
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


def test_l_age_d_une_detection_suit_l_horloge_ros():
    """Gazebo lent : une image par seconde réelle ne doit pas paraître périmée."""
    import math
    from caytu_interfaces.msg import TagDetection, TagDetectionArray
    noeud, entrees = _entrees({'valeur': True})
    detection = TagDetection(tag_id=7, has_pose_robot=True, distance_m=0.8)
    detection.header.stamp.sec = 100
    detection.pose_robot.pose.position.x = 0.8
    demi = math.sqrt(0.5)
    detection.pose_robot.pose.orientation.y = demi
    detection.pose_robot.pose.orientation.w = demi
    for rappel in noeud.abonnements['/tags']:
        rappel(TagDetectionArray(detections=[detection]))
    noeud.horloge.secondes += 0.5
    assert entrees.detection_recente(7, 0.6) is not None
    noeud.horloge.secondes += 0.2
    assert entrees.detection_recente(7, 0.6) is None


def test_un_etat_terminal_de_l_arbitre_arrete_la_course():
    _noeud, entrees = _entrees({'valeur': False})
    entrees._recevoir_statut_arbitre(_StatutArbitre(_StatutArbitre.RUNNING))
    assert entrees.arbitre_termine is False
    entrees._recevoir_statut_arbitre(_StatutArbitre(_StatutArbitre.TERMINATED))
    assert entrees.arbitre_termine is True
    assert entrees.motif_arbitre == 'arbitre_termine'
    assert entrees.arbitre_interrompt_visite


def test_une_fin_normale_de_l_arbitre_est_distinguée_d_un_arret():
    _, entrees = _entrees({'valeur': False})
    entrees._recevoir_statut_arbitre(_StatutArbitre(_StatutArbitre.FINISHED))
    assert entrees.arbitre_termine
    assert entrees.motif_arbitre == 'arbitre_fini'
    assert not entrees.arbitre_interrompt_visite


def test_une_ancienne_fin_ne_termine_pas_le_prochain_match():
    _, entrees = _entrees({'valeur': True})
    entrees._recevoir_statut_arbitre(_StatutArbitre(_StatutArbitre.FINISHED))
    assert not entrees.arbitre_termine


def test_les_visites_accusees_par_l_arbitre_sont_suivies():
    _, entrees = _entrees({'valeur': False})
    entrees._recevoir_statut_arbitre(_StatutArbitre(_StatutArbitre.RUNNING))
    assert entrees.tags_reportes == ()
    assert not entrees.visite_confirmee(7)
    entrees._recevoir_statut_arbitre(_StatutArbitre(
        _StatutArbitre.RUNNING, tags_reported=(7,)))
    assert entrees.tags_reportes == (7,)
    assert entrees.visite_confirmee(7)
    assert not entrees.visite_confirmee(1)
    entrees._recevoir_statut_arbitre(_StatutArbitre(
        _StatutArbitre.RUNNING, tags_reported=(7, 1)))
    assert entrees.visite_confirmee(1)


def test_l_ordre_de_l_arbitre_est_memorise_sans_ecraser_l_ordre_vide():
    _, entrees = _entrees({'valeur': False})
    entrees._recevoir_statut_arbitre(_StatutArbitre(
        _StatutArbitre.RUNNING, tag_order=(4, 1, 9, 7)))
    assert entrees.ordre_arbitre == (4, 1, 9, 7)
    entrees._recevoir_statut_arbitre(_StatutArbitre(_StatutArbitre.RUNNING))
    assert entrees.ordre_arbitre == (4, 1, 9, 7)


def test_un_accuse_de_la_course_precedente_est_efface_au_depart():
    attente = {'valeur': True}
    noeud, entrees = _entrees(attente)
    entrees._recevoir_statut_arbitre(_StatutArbitre(
        _StatutArbitre.FINISHED, tags_reported=(7, 1)))
    assert entrees.visite_confirmee(7)
    _publier(noeud, '/depart', 'commande')
    assert not entrees.visite_confirmee(7)


def test_une_image_d_un_autre_tag_ou_perimee_n_est_pas_une_preuve():
    from caytu_interfaces.msg import TagDetection, TagDetectionArray
    from sensor_msgs.msg import Image
    _, entrees = _entrees({'valeur': False})
    image = Image()
    image.header.stamp.sec = 100
    image.header.frame_id = 'camera2'
    tag = TagDetection(tag_id=4, header=image.header)
    entrees._recevoir_detections(TagDetectionArray(detections=[tag]))
    entrees._recevoir_image(image)
    assert entrees.image_preuve(4) is image
    assert entrees.image_preuve(9) is None
    entrees._node.horloge.secondes = 102
    assert entrees.image_preuve(4) is None


def test_une_detection_en_retard_ne_redevient_pas_fraiche():
    from caytu_interfaces.msg import TagDetection, TagDetectionArray
    _, entrees = _entrees({'valeur': False})
    tag = TagDetection(tag_id=4, has_pose_robot=True, distance_m=0.5)
    tag.header.stamp.sec = 97
    tag.pose_robot.pose.orientation.y = 2 ** -0.5
    tag.pose_robot.pose.orientation.w = 2 ** -0.5
    entrees._recevoir_detections(TagDetectionArray(detections=[tag]))
    assert entrees.detection_recente(4, 0.6) is None


def test_costmap_roulante_ne_double_pas_l_inflation():
    from nav_msgs.msg import OccupancyGrid
    _, entrees = _entrees({'valeur': False})
    entrees._sans_carte = True
    grille = OccupancyGrid(data=[-1, 0, 30, 99, 100])
    grille.info.width, grille.info.height, grille.info.resolution = 5, 1, 0.05
    entrees._recevoir_carte(grille)
    assert entrees.grille().donnees == [-1, 0, 0, 0, 100]


class _FauxTF:
    """TF odom → base_link fixe, pour tester la mémoire parallèle."""

    def __init__(self, x=0.0, y=0.0, lacet=0.0):
        self._x, self._y, self._lacet = x, y, lacet

    def lookup_transform(self, cible, source, instant):
        return SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=self._x, y=self._y, z=0.0),
            rotation=SimpleNamespace(
                x=0.0, y=0.0,
                z=math.sin(self._lacet / 2.0),
                w=math.cos(self._lacet / 2.0))))


def test_une_detection_est_aussi_memorisee_en_odom():
    """La TF odom → base_link replace la vue robot dans le repère odom."""
    import math
    from caytu_interfaces.msg import TagDetection, TagDetectionArray
    memoire_tf = _FauxTF(x=1.0, y=2.0, lacet=math.pi / 2.0)
    noeud, entrees = _entrees({'valeur': True}, memoire_tf=memoire_tf)
    detection = TagDetection(tag_id=7, has_pose_robot=True, distance_m=0.8)
    detection.header.stamp.sec = 100
    detection.pose_robot.pose.position.x = 0.8
    demi = math.sqrt(0.5)
    detection.pose_robot.pose.orientation.y = demi
    detection.pose_robot.pose.orientation.w = demi
    for rappel in noeud.abonnements['/tags']:
        rappel(TagDetectionArray(detections=[detection]))

    pose = entrees._memoire_odom.pose(7)
    assert (pose.x, pose.y) == pytest.approx((1.0, 2.8))
    assert pose.cap == pytest.approx(math.pi / 2.0)
    assert entrees._memoire_odom.repere() == 'odom'


def test_sans_tf_odom_la_detection_robot_reste_utilisable():
    import math
    from caytu_interfaces.msg import TagDetection, TagDetectionArray
    noeud, entrees = _entrees({'valeur': True})
    detection = TagDetection(tag_id=4, has_pose_robot=True, distance_m=0.5)
    detection.header.stamp.sec = 100
    detection.pose_robot.pose.position.x = 0.5
    demi = math.sqrt(0.5)
    detection.pose_robot.pose.orientation.y = demi
    detection.pose_robot.pose.orientation.w = demi
    for rappel in noeud.abonnements['/tags']:
        rappel(TagDetectionArray(detections=[detection]))
    assert entrees.detection_recente(4, 0.6) is not None
    assert entrees._memoire_odom.ids() == []
