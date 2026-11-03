"""Entrées du nœud de course : ordre, départ, détections, images, carte, scan.

Chaque callback ne fait que mémoriser : les décisions restent dans la boucle
de course (`noeud_course.py`).
"""

from collections import deque
from dataclasses import dataclass
import math
import time

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

from caytu_course.geometrie_tags import cap_normale
from caytu_course.ordre_tags import OrdreInvalide, lire_ordre
from caytu_course.recherche import GrilleOccupation
from caytu_interfaces.msg import TagDetectionArray

IMAGES_GARDEES = 30


@dataclass(frozen=True)
class DetectionRobot:
    """Dernière vue d'un tag dans le repère du robot."""

    x: float
    y: float
    cap: float          # normale du tag dans le repère du robot
    distance: float
    instant: float      # time.monotonic() de réception


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
        self._detections_robot = {}
        self._stamps_tag = {}
        self._images = deque(maxlen=IMAGES_GARDEES)
        self._grille = None
        self._scan = None

        type_ordre = (String if self._course.order_type == 'string'
                      else Int32MultiArray)
        # Deux abonnements : le premier récupère un ordre publié avant notre
        # démarrage (transient local), le second accepte tout éditeur.
        for qos in (
                _qos(ReliabilityPolicy.RELIABLE, DurabilityPolicy.TRANSIENT_LOCAL, 1),
                _qos(ReliabilityPolicy.BEST_EFFORT, DurabilityPolicy.VOLATILE, 10)):
            node.create_subscription(
                type_ordre, self._course.order_topic, self._recevoir_ordre, qos)
        node.create_subscription(
            String, self._course.start_topic, self._recevoir_depart, 10)
        node.create_subscription(
            TagDetectionArray, configuration.perception.tags_topic,
            self._recevoir_detections, 20)
        node.create_subscription(
            Image, configuration.perception.apriltag.detection_image_topic,
            self._recevoir_image, 5)
        node.create_subscription(
            OccupancyGrid, self._course.map_topic, self._recevoir_carte,
            _qos(ReliabilityPolicy.RELIABLE, DurabilityPolicy.VOLATILE, 1))
        node.create_subscription(
            LaserScan, configuration.navigation.scan_topic, self._recevoir_scan,
            qos_profile_sensor_data)

    # ------------------------------------------------------------ ordre/départ
    def _recevoir_ordre(self, message):
        valeur = message.data
        if not isinstance(valeur, str):
            valeur = list(valeur)     # Int32MultiArray : tableau d'entiers
        try:
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
        if self.source_depart is not None or not self._attente_depart():
            return
        self.source_depart = message.data.strip() or 'commande'
        self._node.get_logger().info(f'DÉPART reçu ({self.source_depart}).')

    # -------------------------------------------------------------- détections
    def _recevoir_detections(self, message):
        maintenant = time.monotonic()
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

    def detection_recente(self, tag_id, age_max):
        detection = self._detections_robot.get(tag_id)
        if detection is None or time.monotonic() - detection.instant > age_max:
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
