"""Échanges DDS réels : ordre mémorisé, JPEG, arrêt arbitre et garde de vitesse.

Exécutés uniquement dans le conteneur isolé, avec les vrais parc_msgs.
"""

from dataclasses import replace
import json
import os
from pathlib import Path
import time

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get('FOSA_TEST_ROS_ISOLE') != '1',
    reason='Publications moteur réservées au conteneur sans réseau extérieur')


@pytest.fixture
def banc(monkeypatch):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from caytu_config import load_config

    monkeypatch.setenv('ROS_DOMAIN_ID', '42')
    rclpy.init()
    configuration = load_config(Path(__file__).resolve().parents[2] / 'caytu_config/config')
    pilote = rclpy.create_node('banc_fosa')
    executor = SingleThreadedExecutor()
    executor.add_node(pilote)
    noeuds = [pilote]

    def ajouter(noeud):
        noeuds.append(noeud)
        executor.add_node(noeud)
        return noeud

    def attendre(condition, delai=5.0, publier=None):
        # Les méthodes synchrones de course utilisent rclpy.spin_once, qui
        # retire leur nœud de l'exécuteur courant à sa sortie.
        for noeud in noeuds:
            executor.add_node(noeud)
        fin = time.monotonic() + delai
        while time.monotonic() < fin:
            if publier is not None:
                publier()
            executor.spin_once(timeout_sec=0.02)
            if condition():
                return
        assert condition(), 'Échange ROS non reçu dans le délai du banc'

    yield configuration, pilote, ajouter, attendre
    executor.shutdown()
    for noeud in reversed(noeuds):
        noeud.destroy_node()
    rclpy.shutdown()


def test_ordre_jpeg_et_arret_officiels(banc):
    import cv2
    import numpy as np
    from cv_bridge import CvBridge
    from parc_msgs.msg import RunStatus, StartSignal, TagOrder, TagVisit
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from caytu_course.noeud_course import NoeudCourse
    from caytu_course.sequence import SequenceTags
    from caytu_course.statut import qos_statut
    from caytu_interfaces.msg import TagDetection, TagDetectionArray

    configuration, pilote, ajouter, attendre = banc
    ordre = pilote.create_publisher(TagOrder, configuration.course.order_topic, qos_statut(1))
    ordre.publish(TagOrder(tag_ids=[4, 9, 1, 7]))
    course = ajouter(NoeudCourse(configuration))
    attendre(lambda: course._entrees.ordre == (4, 9, 1, 7))
    assert course._entrees.source_depart is None

    departs, visites = [], []
    pilote.create_subscription(StartSignal, configuration.course.referee.start_topic,
                               departs.append, 10)
    pilote.create_subscription(TagVisit, configuration.course.referee.visit_topic,
                               visites.append, 10)
    depart = pilote.create_publisher(String, configuration.course.start_topic, 10)
    attendre(lambda: depart.get_subscription_count() > 0)
    depart.publish(String(data='commande'))
    attendre(lambda: course._entrees.source_depart == 'commande')
    course._en_attente = False
    course._chrono.demarrer()
    course._sequence = SequenceTags((4, 9, 1, 7))
    course._statut.signaler_depart(False, '')
    attendre(lambda: len(departs) == 1)
    assert departs[0].method == StartSignal.METHOD_COMMAND

    detections = pilote.create_publisher(TagDetectionArray, configuration.perception.tags_topic, 20)
    images = pilote.create_publisher(Image, configuration.perception.apriltag.detection_image_topic, 5)
    attendre(lambda: detections.get_subscription_count() > 0 and images.get_subscription_count() > 0)
    image = CvBridge().cv2_to_imgmsg(np.full((240, 320, 3), 255, np.uint8), 'bgr8')
    image.header.stamp = pilote.get_clock().now().to_msg()
    image.header.frame_id = 'camera2_color_optical_frame'
    tag = TagDetection(tag_id=4, header=image.header, has_pose_robot=True, distance_m=0.5)
    tag.pose_robot.pose.position.x = 0.5
    tag.pose_robot.pose.orientation.y = -(0.5 ** 0.5)
    tag.pose_robot.pose.orientation.w = 0.5 ** 0.5
    detections.publish(TagDetectionArray(detections=[tag]))
    attendre(lambda: course._entrees.detection_recente(4, 0.6) is not None)
    assert course._valider_visite(4) == 'echec'  # aucune photo du tag
    images.publish(image)
    attendre(lambda: course._entrees.image_preuve(4) is not None)
    assert course._valider_visite(9) == 'perdu'  # l'image du 4 ne prouve pas le 9
    debut = time.monotonic()
    assert course._valider_visite(4) == 'visite'
    assert time.monotonic() - debut >= configuration.course.hold_sec
    attendre(lambda: len(visites) == 1)
    assert visites[0].tag_id == 4 and visites[0].image.format == 'jpeg'
    assert visites[0].header.frame_id == image.header.frame_id
    assert cv2.imdecode(np.frombuffer(visites[0].image.data, np.uint8), cv2.IMREAD_COLOR) is not None
    assert course._valider_visite(4) == 'perdu'  # même observation devenue périmée

    arbitre = pilote.create_publisher(RunStatus, configuration.course.referee.run_status_topic, qos_statut(1))
    attendre(lambda: arbitre.get_subscription_count() > 0)
    arbitre.publish(RunStatus(state=RunStatus.TERMINATED))
    attendre(course._temps_ecoule)


def test_garde_blocage_capteurs_et_fin_arbitre(banc):
    from geometry_msgs.msg import Twist
    from parc_msgs.msg import RunStatus
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from caytu_course.noeud_garde_vitesse import GardeVitesse
    from caytu_course.statut import qos_statut

    configuration, pilote, ajouter, attendre = banc
    navigation = configuration.navigation
    securite = replace(navigation.securite, age_scan_max_sec=0.3, age_commande_max_sec=0.2)
    configuration = replace(configuration, navigation=replace(navigation, securite=securite))
    garde = ajouter(GardeVitesse(configuration))
    sorties = []
    pilote.create_subscription(Twist, navigation.cmd_vel_topic, sorties.append, 10)
    commandes = pilote.create_publisher(Twist, securite.topic_entree, 10)
    etats = pilote.create_publisher(String, configuration.course.state_topic, qos_statut(1))
    arbitre = pilote.create_publisher(RunStatus, configuration.course.referee.run_status_topic, qos_statut(1))
    scans = [pilote.create_publisher(LaserScan, source.topic, 10)
             for source in navigation.obstacle_sources if source.enabled]
    attendre(lambda: commandes.get_subscription_count() > 0 and etats.get_subscription_count() > 0)
    commande = Twist()
    commande.linear.x = 0.2
    distance = 3.0
    sources_actives = scans

    def publier():
        scan = LaserScan(range_min=0.1, range_max=4.0, ranges=[distance], angle_increment=0.1)
        scan.header.frame_id = configuration.system.base_frame
        scan.header.stamp = pilote.get_clock().now().to_msg()
        for source in sources_actives:
            source.publish(scan)
        commandes.publish(commande)

    attendre(lambda: len(sorties) >= 3, publier=publier)
    assert all(message.linear.x == 0.0 for message in sorties)  # pas de départ
    etats.publish(String(data=json.dumps({'etat': 'DEMARRAGE'})))
    attendre(lambda: garde._debut_course is not None)
    etats.publish(String(data=json.dumps({'etat': 'RECHERCHE'})))
    attendre(lambda: sorties[-1].linear.x > 0.0, publier=publier)
    distance = 0.4
    attendre(lambda: sorties[-1].linear.x == 0.0, publier=publier)  # obstacle sur la trajectoire
    distance = 3.0
    attendre(lambda: sorties[-1].linear.x > 0.0, publier=publier)
    sources_actives = scans[:-1]
    attendre(lambda: sorties[-1].linear.x == 0.0, publier=publier)  # une caméra se tait
    sources_actives = scans
    attendre(lambda: sorties[-1].linear.x > 0.0, publier=publier)
    attendre(lambda: sorties[-1].linear.x == 0.0)  # commande abandonnée
    attendre(lambda: sorties[-1].linear.x > 0.0, publier=publier)
    arbitre.publish(RunStatus(state=RunStatus.TERMINATED))
    attendre(lambda: garde._fin_arbitre and sorties[-1].linear.x == 0.0, publier=publier)
