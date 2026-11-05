"""Registre des AprilTags : position de chaque tag dans `map`, enregistrée.

Sources (perception.apriltag.detector dans caytu_config) :
  opencv       /perception/tags du nœud tag_detector (poses déjà dans map) ;
  apriltag_ros /detections du nœud apriltag_ros + sa TF tag36h11:<id>.
               apriltag_node publie /detections AVANT la TF du tag : chaque
               message attend ici (au plus ATTENTE_TF_TAG s) que sa TF arrive.

Publie :
  /fosa/tags_vus      chaque observation, au fil des images (TagDetectionArray) ;
  /fosa/tags_connus   l'estimation filtrée de chaque tag (latched, 1 Hz) ;
  /fosa/tags_marqueurs  flèches et numéros pour RViz.
Dans les deux messages, pose_map.position = centre du tag, et
pose_map.orientation = cap de la normale horizontale, côté libre.

Avec fichier_tags:=<dossier de carte>/tags.yaml, les tags déjà enregistrés
sont rechargés au démarrage et le fichier est réécrit à chaque nouveauté.
"""

import math
from pathlib import Path
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from caytu_config import load_config
from caytu_interfaces.msg import TagDetection, TagDetectionArray
from caytu_perception.detecteur_tags import pose_camera_dans_carte
from caytu_perception.tags_carte import (
    ObservationTag, RegistreTags, lacet_normale)

# apriltag_ros : délai maximal entre /detections et la TF du tag (même image).
ATTENTE_TF_TAG = 0.5
MESSAGES_EN_ATTENTE_MAX = 20


def _angle_vue(position_tag, lacet, camera):
    """Angle entre la normale du tag et la direction tag -> caméra."""
    if camera is None:
        return None
    vers_camera = math.atan2(camera[1] - position_tag[1], camera[0] - position_tag[0])
    return abs(math.atan2(math.sin(vers_camera - lacet), math.cos(vers_camera - lacet)))


def _quaternion_lacet(lacet):
    return (0.0, 0.0, math.sin(lacet / 2.0), math.cos(lacet / 2.0))


class NoeudRegistreTags(Node):

    def __init__(self, configuration=None):
        super().__init__('registre_tags')
        self._configuration = configuration or load_config()
        perception = self._configuration.perception
        course = self._configuration.course

        self.declare_parameter('detector', perception.apriltag.detector)
        self.declare_parameter('detections_topic', perception.tags_topic)
        self.declare_parameter('apriltag_ros_topic', '/detections')
        self.declare_parameter('fichier_tags', '')
        # False en cartographie : nouvelle carte, nouveau repère, anciens tags faux.
        self.declare_parameter('recharger', True)
        self.declare_parameter('repere', self._configuration.system.global_frame)
        self.declare_parameter('min_observations', course.min_observations)
        # rad/s : au-delà, les vues lointaines ne sont pas mémorisées.
        self.declare_parameter('rotation_max', 0.15)
        self._rotation_max = float(self.get_parameter('rotation_max').value)

        self._detecteur = self.get_parameter('detector').value
        self._repere = self.get_parameter('repere').value
        self._fichier = str(self.get_parameter('fichier_tags').value or '')
        self._min_observations = int(self.get_parameter('min_observations').value)
        self._registre = RegistreTags(
            identifiants_attendus=tuple(course.expected_ids),
            distance_max=perception.apriltag.max_distance_m)
        if self._fichier and self.get_parameter('recharger').value:
            nombre = self._registre.charger(self._fichier)
            self.get_logger().info(
                f'{nombre} tag(s) rechargé(s) depuis {self._fichier}.')
        self._a_enregistrer = False
        self._derniere_sauvegarde = 0.0

        self._tf = Buffer()
        self._ecouteur_tf = TransformListener(self._tf, self, spin_thread=True)

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._publicateur_vus = self.create_publisher(
            TagDetectionArray, '/fosa/tags_vus', 10)
        self._publicateur_connus = self.create_publisher(
            TagDetectionArray, course.tags_topic, latched)
        self._publicateur_marqueurs = self.create_publisher(
            MarkerArray, '/fosa/tags_marqueurs', latched)

        self._apriltag_en_attente = []      # (échéance, message) sans TF encore
        self._apriltag_perdus = 0
        if self._detecteur == 'apriltag_ros':
            from apriltag_msgs.msg import AprilTagDetectionArray
            self.create_subscription(
                AprilTagDetectionArray,
                self.get_parameter('apriltag_ros_topic').value,
                self.recevoir_apriltag_ros, 10)
            self.create_timer(0.05, self._traiter_apriltag_en_attente)
        else:
            self.create_subscription(
                TagDetectionArray, self.get_parameter('detections_topic').value,
                self.recevoir_opencv, 10)
        self.create_timer(1.0, self.publier_connus)
        self.get_logger().info(
            f'Registre des tags : détecteur {self._detecteur}, repère {self._repere}, '
            f'fichier {self._fichier or "(aucun)"}.')

    # ------------------------------------------------------------ entrées
    def _position_camera(self, repere_camera, horodatage):
        try:
            tf = pose_camera_dans_carte(self._tf, self._repere, repere_camera, horodatage,
                                        attendre=False)
        except TransformException:
            return None
        return (tf.transform.translation.x, tf.transform.translation.y)

    def recevoir_opencv(self, message):
        vues = []
        for detection in message.detections:
            pose = detection.pose_map.pose
            q = pose.orientation
            position = (pose.position.x, pose.position.y, pose.position.z)
            camera = self._position_camera(
                detection.header.frame_id or message.header.frame_id,
                Time.from_msg(detection.header.stamp))
            normale = lacet_normale((q.x, q.y, q.z, q.w), position, camera)
            vues.append(ObservationTag(
                identifiant=int(detection.tag_id),
                x=position[0], y=position[1], z=position[2],
                lacet_normale=normale,
                distance=float(detection.distance_m),
                horodatage=time.monotonic(),
                angle_vue=_angle_vue(position, normale, camera)))
        self._traiter(vues, message.header)

    def recevoir_apriltag_ros(self, message):
        if not message.detections:
            return
        self._apriltag_en_attente.append((time.monotonic() + ATTENTE_TF_TAG, message))
        del self._apriltag_en_attente[:-MESSAGES_EN_ATTENTE_MAX]
        self._traiter_apriltag_en_attente()

    def _tf_tags_arrivees(self, message):
        """Vrai quand la TF caméra -> tag de chaque détection de l'image est là."""
        famille = self._configuration.perception.apriltag.family
        horodatage = Time.from_msg(message.header.stamp)
        return all(self._tf.can_transform(message.header.frame_id,
                                          f'tag{famille}:{detection.id}', horodatage)
                   for detection in message.detections)

    def _traiter_apriltag_en_attente(self):
        maintenant = time.monotonic()
        restants = []
        for echeance, message in self._apriltag_en_attente:
            if self._tf_tags_arrivees(message):
                self._convertir_apriltag_ros(message)
            elif maintenant < echeance:
                restants.append((echeance, message))
            else:
                self._apriltag_perdus += 1
                if self._apriltag_perdus in (1, 10, 100) or self._apriltag_perdus % 1000 == 0:
                    self.get_logger().warn(
                        f'apriltag_ros : {self._apriltag_perdus} image(s) sans TF du tag '
                        f'après {ATTENTE_TF_TAG} s (TF publiée par apriltag_node ?).')
        self._apriltag_en_attente = restants

    def _convertir_apriltag_ros(self, message):
        famille = self._configuration.perception.apriltag.family
        horodatage = Time.from_msg(message.header.stamp)
        vues = []
        for detection in message.detections:
            nom = f'tag{famille}:{detection.id}'
            try:
                tf = pose_camera_dans_carte(self._tf, self._repere, nom, horodatage,
                                            attendre=False)
            except TransformException:
                continue
            t, q = tf.transform.translation, tf.transform.rotation
            camera = self._position_camera(message.header.frame_id, horodatage)
            distance = (math.hypot(t.x - camera[0], t.y - camera[1])
                        if camera is not None else 0.0)
            normale = lacet_normale((q.x, q.y, q.z, q.w), (t.x, t.y, t.z), camera)
            vues.append(ObservationTag(
                identifiant=int(detection.id), x=t.x, y=t.y, z=t.z,
                lacet_normale=normale, distance=distance, horodatage=time.monotonic(),
                angle_vue=_angle_vue((t.x, t.y), normale, camera)))
        self._traiter(vues, message.header)

    def _rotation_rapide(self, horodatage):
        """Vrai si le robot tournait vite à l'heure de l'image.

        Pendant un virage, la caméra est mal située (le cap de l'odométrie
        dérive jusqu'à la correction suivante de slam_toolbox) : une vue
        lointaine prise alors peut placer le tag à plus d'un mètre.
        """
        repere_robot = self._configuration.system.base_frame
        if horodatage.nanoseconds < 250_000_000:
            return False                    # temps simulé qui démarre : pas d'avant
        try:
            maintenant = self._tf.lookup_transform('odom', repere_robot, horodatage)
            avant = self._tf.lookup_transform(
                'odom', repere_robot,
                Time(nanoseconds=horodatage.nanoseconds - 250_000_000))
        except TransformException:
            return False
        def lacet(tf):
            q = tf.transform.rotation
            return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        ecart = math.atan2(math.sin(lacet(maintenant) - lacet(avant)),
                           math.cos(lacet(maintenant) - lacet(avant)))
        return abs(ecart) / 0.25 > self._rotation_max

    def _traiter(self, vues, entete):
        # /fosa/tags_vus : toute vue valide, même si elle contredit l'estimation
        # (l'approche finale se fie à la vue fraîche, pas à la mémoire).
        valides = [vue for vue in vues if self._registre.observation_valide(vue)]
        if valides:
            sortie = TagDetectionArray()
            sortie.header.stamp = entete.stamp
            sortie.header.frame_id = self._repere
            sortie.detections = [self._message(vue.identifiant, vue.x, vue.y, vue.z,
                                               vue.lacet_normale, vue.distance, 1.0,
                                               entete.stamp)
                                 for vue in valides]
            self._publicateur_vus.publish(sortie)
        # Mémoire : pas de vue lointaine prise en plein virage.
        if valides and self._rotation_rapide(Time.from_msg(entete.stamp)):
            valides = [vue for vue in valides if vue.distance <= self._registre.distance_proche]
        acceptees = [vue for vue in valides if self._registre.ajouter(vue)]
        if not acceptees:
            return
        for vue in acceptees:
            estimation = self._registre.estimation(vue.identifiant)
            if estimation.observations == self._min_observations:
                x, y, cap = estimation.pose_visite(self._configuration.course.visit_distance_m)
                self.get_logger().info(
                    f'Tag {vue.identifiant} repéré en ({estimation.x:.2f}, {estimation.y:.2f}), '
                    f'pose de visite ({x:.2f}, {y:.2f}, {math.degrees(cap):.0f}°).')
        self._a_enregistrer = True

    # ------------------------------------------------------------ sorties
    def _message(self, identifiant, x, y, z, lacet, distance, confiance, stamp):
        detection = TagDetection()
        detection.header.stamp = stamp
        detection.header.frame_id = self._repere
        detection.tag_id = int(identifiant)
        detection.pose_map.header = detection.header
        detection.pose_map.pose.position.x = float(x)
        detection.pose_map.pose.position.y = float(y)
        detection.pose_map.pose.position.z = float(z)
        (detection.pose_map.pose.orientation.x, detection.pose_map.pose.orientation.y,
         detection.pose_map.pose.orientation.z,
         detection.pose_map.pose.orientation.w) = _quaternion_lacet(lacet)
        detection.distance_m = float(distance)
        detection.confidence = float(confiance)
        return detection

    def publier_connus(self):
        maintenant = self.get_clock().now().to_msg()
        estimations = self._registre.estimations()
        sortie = TagDetectionArray()
        sortie.header.stamp = maintenant
        sortie.header.frame_id = self._repere
        marqueurs = MarkerArray()
        distance = self._configuration.course.visit_distance_m
        for estimation in estimations:
            confiance = min(1.0, estimation.observations / float(self._min_observations))
            sortie.detections.append(self._message(
                estimation.identifiant, estimation.x, estimation.y, estimation.z,
                estimation.lacet_normale, 0.0, confiance, maintenant))
            marqueurs.markers.extend(self._marqueurs(estimation, distance, maintenant))
        self._publicateur_connus.publish(sortie)
        self._publicateur_marqueurs.publish(marqueurs)
        if (self._fichier and self._a_enregistrer
                and time.monotonic() - self._derniere_sauvegarde > 2.0):
            try:
                Path(self._fichier).parent.mkdir(parents=True, exist_ok=True)
                self._registre.enregistrer(self._fichier, self._repere)
                self._a_enregistrer = False
                self._derniere_sauvegarde = time.monotonic()
            except OSError as erreur:
                self.get_logger().error(f'Écriture de {self._fichier} impossible : {erreur}')

    def _marqueurs(self, estimation, distance, stamp):
        base = 100 * estimation.identifiant
        fleche = Marker()
        fleche.header.frame_id = self._repere
        fleche.header.stamp = stamp
        fleche.ns, fleche.id, fleche.type = 'tags', base, Marker.ARROW
        x_visite, y_visite, _ = estimation.pose_visite(distance)
        fleche.points = [Point(x=estimation.x, y=estimation.y, z=0.05),
                         Point(x=x_visite, y=y_visite, z=0.05)]
        fleche.scale.x, fleche.scale.y, fleche.scale.z = 0.04, 0.08, 0.1
        fleche.color.r, fleche.color.g, fleche.color.a = 0.1, 0.8, 1.0
        texte = Marker()
        texte.header = fleche.header
        texte.ns, texte.id, texte.type = 'tags', base + 1, Marker.TEXT_VIEW_FACING
        texte.pose.position.x, texte.pose.position.y = estimation.x, estimation.y
        texte.pose.position.z = estimation.z + 0.25
        texte.pose.orientation.w = 1.0
        texte.scale.z = 0.25
        texte.color.r = texte.color.g = texte.color.b = texte.color.a = 1.0
        texte.text = f'tag {estimation.identifiant} ({estimation.observations})'
        return [fleche, texte]


def main(args=None):
    rclpy.init(args=args)
    noeud = NoeudRegistreTags()
    try:
        rclpy.spin(noeud)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                       # Ctrl-C ou arrêt du lancement : sortie propre
    finally:
        if noeud._fichier and noeud._a_enregistrer:
            try:
                noeud._registre.enregistrer(noeud._fichier, noeud._repere)
            except OSError:
                pass
        noeud.destroy_node()
        rclpy.try_shutdown()
