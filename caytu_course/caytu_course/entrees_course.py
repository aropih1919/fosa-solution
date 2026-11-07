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


def _distance_ponderee(detection):
    """Distance de pondération : une pose moins fiable compte plus loin.

    Le détecteur met 1,0 pour une pose exacte et 0,5 pour une TF approchée.
    Une confiance absente (0, valeur par défaut d'un message) ne doit pas
    faire rejeter la détection : elle est traitée comme exacte.
    """
    confiance = detection.confidence
    if not math.isfinite(confiance) or confiance <= 0.0:
        confiance = 1.0
    return detection.distance_m / confiance


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

    def __init__(self, node, configuration, memoire, memoire_tf, attente_depart,
                 ecouter_arbitre=True, memoire_odom=None):
        """attente_depart() : vrai tant que la course n'a pas commencé.

        `memoire_odom` reçoit les mêmes détections dans le repère `odom` :
        contrairement à `map`, ce repère n'est pas déplacé par les corrections
        SLAM, donc l'export final reste cohérent avec la carte enregistrée.
        """
        self._node = node
        self._attente_depart = attente_depart
        self._course = configuration.course
        self._memoire = memoire
        self._memoire_odom = memoire_odom
        self._memoire_tf = memoire_tf
        self._repere_robot = configuration.system.base_frame
        self._repere_odom = configuration.navigation.odom_frame
        self._sans_carte = configuration.navigation.mode == 'sans_carte'
        self.ordre = None
        self.source_depart = None
        self.mot_entendu = ''
        self.depart_de_l_arbitre = False
        self.statut_arbitre = None
        self.tags_reportes = ()       # accusés `tags_reported` de l'arbitre
        self.ordre_arbitre = ()
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
        if ecouter_arbitre:
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
        # QoS du protocole officiel : reliable + transient local, profondeur 1.
        # L'arbitre republie son état à 2 Hz ; un abonné tardif reçoit le dernier.
        self._abonner_quand_publie(
            arbitre.run_status_topic,
            lambda classe: self._node.create_subscription(
                classe, arbitre.run_status_topic, self._recevoir_statut_arbitre,
                _qos(ReliabilityPolicy.RELIABLE, DurabilityPolicy.TRANSIENT_LOCAL, 1)))

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
        # Un accusé de la course précédente (RunStatus transient local) ne doit
        # pas confirmer une visite de la nouvelle course.
        self.tags_reportes = ()
        origine = ' via l\'arbitre' if de_l_arbitre else ''
        self._node.get_logger().info(f'DÉPART reçu ({source}{origine}).')

    def _recevoir_statut_arbitre(self, message):
        texte = str(message)
        if texte != self.statut_arbitre:
            self.statut_arbitre = texte
            self._node.get_logger().info(f'Statut de l\'arbitre : {texte[:300]}')
        reportes = tuple(getattr(message, 'tags_reported', ()))
        if reportes != self.tags_reportes:
            self.tags_reportes = reportes
            self._node.get_logger().info(
                f'Visites reçues par l’arbitre : {list(reportes)}.')
        ordre = tuple(getattr(message, 'tag_order', ()))
        if ordre and ordre != self.ordre_arbitre:
            self.ordre_arbitre = ordre
        etat = getattr(message, 'state', None)
        # La dernière course peut être encore FINISHED quand nous démarrons.
        # Elle ne doit pas terminer la nouvelle course avant son départ.
        if self._attente_depart() and self.source_depart is None:
            return
        for nom, secours in ETATS_TERMINAUX_ARBITRE:
            if etat == getattr(type(message), nom, secours):
                self.arbitre_interrompt_visite = nom in ('TIMED_OUT', 'TERMINATED')
                if not self.arbitre_termine:
                    self.arbitre_termine = True
                    # FINISHED est la fin normale (4 tags ou /parc/finish) ;
                    # TIMED_OUT et TERMINATED sont des arrêts imposés.
                    self.motif_arbitre = (
                        'arbitre_fini' if nom == 'FINISHED' else 'arbitre_termine')
                    self._node.get_logger().warning(
                        f'ARBITRE : état {nom} — arrêt de la course.')
                return

    def visite_confirmee(self, tag_id):
        """Vrai si l'arbitre a accusé réception de ce tag (`tags_reported`)."""
        return int(tag_id) in self.tags_reportes

    # -------------------------------------------------------------- détections
    def _recevoir_detections(self, message):
        for detection in message.detections:
            maintenant = Time.from_msg(detection.header.stamp).nanoseconds * 1e-9
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
                        _distance_ponderee(detection))
            if detection.has_pose_robot:
                pose = detection.pose_robot.pose
                cap = cap_normale(
                    pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w)
                if cap is not None:
                    self._detections_robot[detection.tag_id] = DetectionRobot(
                        pose.position.x, pose.position.y, cap,
                        detection.distance_m, maintenant)
                    self._ajouter_odom(
                        detection, pose.position.x, pose.position.y, cap)
            self._stamps_tag.setdefault(detection.tag_id, deque(maxlen=IMAGES_GARDEES)).append(
                _cle_image(detection.header))

    def _ajouter_odom(self, detection, x, y, cap):
        """Mémorise la détection dans `odom`, insensible aux corrections SLAM."""
        if self._memoire_odom is None or self._memoire_tf is None:
            return
        try:
            transformation = self._memoire_tf.lookup_transform(
                self._repere_odom, self._repere_robot,
                Time.from_msg(detection.header.stamp))
        except TransformException:
            try:
                transformation = self._memoire_tf.lookup_transform(
                    self._repere_odom, self._repere_robot, Time())
            except TransformException:
                self._node.get_logger().warning(
                    f'TF {self._repere_odom} → {self._repere_robot} '
                    'indisponible : détection non mémorisée en odom.',
                    throttle_duration_sec=5.0)
                return
        t = transformation.transform.translation
        q = transformation.transform.rotation
        lacet = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cosinus, sinus = math.cos(lacet), math.sin(lacet)
        cap_odom = math.atan2(math.sin(cap + lacet), math.cos(cap + lacet))
        self._memoire_odom.ajouter(
            detection.tag_id,
            t.x + cosinus * x - sinus * y,
            t.y + sinus * x + cosinus * y,
            cap_odom,
            _distance_ponderee(detection))

    def _maintenant(self):
        """Horloge ROS : Gazebo peut tourner bien plus lentement que le temps réel."""
        return self._node.get_clock().now().nanoseconds * 1e-9

    def detection_recente(self, tag_id, age_max):
        detection = self._detections_robot.get(tag_id)
        if detection is None or not 0.0 <= self._maintenant() - detection.instant <= age_max:
            return None
        return detection

    def _recevoir_image(self, message):
        self._images.append((_cle_image(message.header), message))

    def image_preuve(self, tag_id):
        """Image fraîche où CET identifiant a effectivement été détecté."""
        cles = self._stamps_tag.get(tag_id, ())
        for cle_image, image in reversed(self._images):
            age = self._maintenant() - (cle_image[0] + cle_image[1] * 1e-9)
            if cle_image in cles and 0.0 <= age <= self._course.image_max_age_sec:
                return image
        return None

    # ------------------------------------------------------------- carte/scan
    def _recevoir_carte(self, message):
        info = message.info
        donnees = list(message.data)
        if self._sans_carte:
            # La costmap inclut déjà l'inflation Nav2. La recherche applique
            # sa propre marge autour des cellules létales, une seule fois.
            donnees = [-1 if valeur < 0 else (100 if valeur == 100 else 0)
                       for valeur in donnees]
        self._grille = GrilleOccupation(
            info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y, donnees)

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
