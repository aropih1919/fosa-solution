"""Entrées du nœud de course : ordre, départ, détections, images, carte, scan.

Chaque callback ne fait que mémoriser : les décisions restent dans la boucle
de course (`noeud_course.py`).
"""

from collections import deque
from dataclasses import dataclass
import json
import math

from nav_msgs.msg import OccupancyGrid
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Int32MultiArray, String
from tf2_ros import TransformException

from caytu_course.arbitre import contenu_ordre
from caytu_course.geometrie_tags import cap_normale
from caytu_course.ordre_tags import OrdreInvalide, lire_ordre
from caytu_course.recherche import GrilleOccupation
from caytu_course.statut import charger_type
from caytu_interfaces.msg import TagDetectionArray

IMAGES_GARDEES = 30
TYPES_ORDRE = {'string': String, 'int32_multi_array': Int32MultiArray}
PERIODE_DECOUVERTE = 0.5

# États terminaux de parc_msgs/RunStatus : (nom du champ, valeur de secours).
# La valeur lue dans la classe du message est prioritaire ; les valeurs de
# secours sont celles du protocole officiel (parc_msgs/RunStatus.msg).
ETATS_TERMINAUX_ARBITRE = (
    ('FINISHED', 3),
    ('TIMED_OUT', 4),
    ('TERMINATED', 5),
)


@dataclass(frozen=True)
class DetectionRobot:
    """Dernière vue d'un tag dans le repère du robot."""

    x: float
    y: float
    cap: float          # normale du tag dans le repère du robot
    distance: float
    instant: float      # horloge ROS à la réception (simulée dans Gazebo)


def _qos(fiabilite, durabilite, profondeur):
    return QoSProfile(depth=profondeur, reliability=fiabilite, durability=durabilite)


class EntreesCourse:

    def __init__(self, node, configuration, memoire, memoire_tf, attente_depart):
        """attente_depart() : vrai tant que la course n'a pas commencé."""
        self._node = node
        self._attente_depart = attente_depart
        self._course = configuration.course
        self._memoire = memoire
        self._memoire_tf = memoire_tf
        self._repere_robot = configuration.system.base_frame
        self.ordre = None
        self.source_depart = None
        self.mot_entendu = ''
        self.depart_de_l_arbitre = False
        self.statut_arbitre = None
        self.arbitre_termine = False
        self.arbitre_interrompt_visite = False
        self.motif_arbitre = ''
        self._detections_robot = {}
        self._stamps_tag = {}
        self._images = deque(maxlen=IMAGES_GARDEES)
        self._grille = None
        self._scan = None

        if self._course.order_type == 'auto':
            self._abonner_quand_publie(self._course.order_topic, self._abonner_ordre)
        else:
            self._abonner_ordre(TYPES_ORDRE[self._course.order_type])
        node.create_subscription(
            String, self._course.start_topic, self._recevoir_depart, 10)
        self._abonner_arbitre(self._course.referee)
        node.create_subscription(
            TagDetectionArray, configuration.perception.tags_topic,
            self._recevoir_detections, 20)
        node.create_subscription(
            Image, configuration.perception.apriltag.detection_image_topic,
            self._recevoir_image, 5)
        node.create_subscription(
            OccupancyGrid, self._course.map_topic, self._recevoir_carte,
            # SLAM publie la carte en TRANSIENT_LOCAL : l'exploration doit aussi
            # recevoir la dernière grille si elle démarre après sa publication.
            _qos(ReliabilityPolicy.RELIABLE, DurabilityPolicy.TRANSIENT_LOCAL, 1))
        node.create_subscription(
            LaserScan, configuration.navigation.scan_topic, self._recevoir_scan,
            qos_profile_sensor_data)

    def _abonner_quand_publie(self, topic, abonner):
        """Attend qu'un éditeur apparaisse sur `topic`, puis s'abonne avec son type."""
        def verifier():
            types = dict(self._node.get_topic_names_and_types()).get(topic)
            if not types:
                return
            classe = charger_type(types[0])
            if classe is None:
                self._node.get_logger().error(
                    f'{topic} : type {types[0]} introuvable (espace de travail '
                    'du robot non sourcé ?).', throttle_duration_sec=10.0)
                return
            minuterie.cancel()
            self._node.get_logger().info(f'{topic} : type {types[0]}.')
            abonner(classe)
        minuterie = self._node.create_timer(PERIODE_DECOUVERTE, verifier)

    def _abonner_ordre(self, type_ordre):
        # Deux abonnements : le premier récupère un ordre publié avant notre
        # démarrage (transient local), le second accepte tout éditeur.
        for qos in (
                _qos(ReliabilityPolicy.RELIABLE, DurabilityPolicy.TRANSIENT_LOCAL, 1),
                _qos(ReliabilityPolicy.BEST_EFFORT, DurabilityPolicy.VOLATILE, 10)):
            self._node.create_subscription(
                type_ordre, self._course.order_topic, self._recevoir_ordre, qos)

    def _abonner_arbitre(self, arbitre):
        if arbitre.listen_start:
            classe = charger_type(arbitre.start_type)
            if classe is None:
                self._node.get_logger().warning(
                    f'{arbitre.start_type} introuvable : départ par {arbitre.start_topic} '
                    'non écouté.')
            else:
                # Volatile : un ancien départ resté en mémoire ne relance rien.
                self._node.create_subscription(
                    classe, arbitre.start_topic, self._recevoir_depart_arbitre, 10)
        # Best effort : compatible avec tout éditeur, simple suivi dans le journal.
        self._abonner_quand_publie(
            arbitre.run_status_topic,
            lambda classe: self._node.create_subscription(
                classe, arbitre.run_status_topic, self._recevoir_statut_arbitre,
                _qos(ReliabilityPolicy.BEST_EFFORT, DurabilityPolicy.VOLATILE, 10)))

    # ------------------------------------------------------------ ordre/départ
    def _recevoir_ordre(self, message):
        valeur = contenu_ordre(message)
        try:
            if valeur is None:
                raise OrdreInvalide(f'aucun ordre lisible dans {message!r}')
            ordre = lire_ordre(valeur, self._course.tag_ids)
        except OrdreInvalide as erreur:
            self._node.get_logger().error(
                f'Ordre ignoré : {erreur}', throttle_duration_sec=2.0)
            return
        if ordre == self.ordre:
            return
        # Départ par commande avant l'ordre : le premier ordre reste accepté.
        if self.ordre is not None and not self._attente_depart():
            self._node.get_logger().warning(
                f'Nouvel ordre {list(ordre)} ignoré : la course a commencé.')
            return
        self.ordre = ordre
        self._node.get_logger().info(f'Ordre des tags reçu : {list(ordre)}.')

    def _recevoir_depart(self, message):
        """Départ interne : « commande » ou JSON {"source": "voix", "entendu": ...}."""
        texte = message.data.strip()
        try:
            contenu = json.loads(texte)
        except ValueError:
            contenu = None
        if isinstance(contenu, dict):
            self._noter_depart(
                str(contenu.get('source', 'commande')), str(contenu.get('entendu', '')),
                de_l_arbitre=False)
        else:
            self._noter_depart(texte or 'commande', '', de_l_arbitre=False)

    def _recevoir_depart_arbitre(self, message):
        """Départ publié directement sur /parc/start (déjà connu de l'arbitre)."""
        vocal = int(message.method) == self._course.referee.start_method_voice
        self._noter_depart(
            'voix' if vocal else 'commande', str(getattr(message, 'heard', '')),
            de_l_arbitre=True)

    def _noter_depart(self, source, entendu, de_l_arbitre):
        if self.source_depart is not None or not self._attente_depart():
            return
        self.source_depart = source
        self.mot_entendu = entendu
        self.depart_de_l_arbitre = de_l_arbitre
        origine = ' via l\'arbitre' if de_l_arbitre else ''
        self._node.get_logger().info(f'DÉPART reçu ({source}{origine}).')

    def _recevoir_statut_arbitre(self, message):
        texte = str(message)
        if texte != self.statut_arbitre:
            self.statut_arbitre = texte
            self._node.get_logger().info(f'Statut de l\'arbitre : {texte[:300]}')
        etat = getattr(message, 'state', None)
        for nom, secours in ETATS_TERMINAUX_ARBITRE:
            if etat == getattr(type(message), nom, secours):
                self.arbitre_interrompt_visite = nom in ('TIMED_OUT', 'TERMINATED')
                if not self.arbitre_termine:
                    self.arbitre_termine = True
                    self.motif_arbitre = 'arbitre_termine'
                    self._node.get_logger().warning(
                        f'ARBITRE : état {nom} — arrêt de la course.')
                return

    # -------------------------------------------------------------- détections
    def _recevoir_detections(self, message):
        maintenant = self._maintenant()
        for detection in message.detections:
            if detection.has_pose_map:
                pose = detection.pose_map.pose
                cap = cap_normale(
                    pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w)
                if cap is not None:
                    # TF approchée (confiance < 1) : la vue compte comme plus
                    # lointaine, donc moins que les vues exactes.
                    self._memoire.ajouter(
                        detection.tag_id, pose.position.x, pose.position.y, cap,
                        detection.distance_m / max(detection.confidence, 0.1))
            if detection.has_pose_robot:
                pose = detection.pose_robot.pose
                cap = cap_normale(
                    pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w)
                if cap is not None:
                    self._detections_robot[detection.tag_id] = DetectionRobot(
                        pose.position.x, pose.position.y, cap,
                        detection.distance_m, maintenant)
            self._stamps_tag[detection.tag_id] = _cle_image(detection.header)

    def _maintenant(self):
        """Horloge ROS : Gazebo peut tourner bien plus lentement que le temps réel."""
        return self._node.get_clock().now().nanoseconds * 1e-9

    def detection_recente(self, tag_id, age_max):
        detection = self._detections_robot.get(tag_id)
        if detection is None or self._maintenant() - detection.instant > age_max:
            return None
        return detection

    def _recevoir_image(self, message):
        self._images.append((_cle_image(message.header), message))

    def image_preuve(self, tag_id):
        """Image où le tag a été détecté ; à défaut la plus récente de la même
        caméra (l'autre caméra ne montre pas forcément le tag)."""
        cle = self._stamps_tag.get(tag_id)
        if cle is None:
            return self._images[-1][1] if self._images else None
        for cle_image, image in reversed(self._images):
            if cle_image == cle:
                return image
        for cle_image, image in reversed(self._images):
            if cle_image[2] == cle[2]:
                return image
        return None

    # ------------------------------------------------------------- carte/scan
    def _recevoir_carte(self, message):
        info = message.info
        self._grille = GrilleOccupation(
            info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y, list(message.data))

    def grille(self):
        return self._grille

    def _recevoir_scan(self, message):
        self._scan = message

    def points_scan_robot(self):
        """Points du dernier scan dans le repère du robot (liste vide sinon)."""
        scan = self._scan
        if scan is None:
            return []
        try:
            transformation = self._memoire_tf.lookup_transform(
                self._repere_robot, scan.header.frame_id, Time())
        except TransformException:
            return []
        t = transformation.transform.translation
        q = transformation.transform.rotation
        lacet = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        points = []
        for index, distance in enumerate(scan.ranges):
            if not (scan.range_min <= distance <= scan.range_max):
                continue
            angle = lacet + scan.angle_min + index * scan.angle_increment
            points.append((t.x + distance * math.cos(angle),
                           t.y + distance * math.sin(angle)))
        return points


def _cle_image(entete):
    return (entete.stamp.sec, entete.stamp.nanosec, entete.frame_id)
