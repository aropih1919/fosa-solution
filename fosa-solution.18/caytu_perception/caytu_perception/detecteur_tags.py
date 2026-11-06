"""Nœud de détection AprilTag : une instance par caméra couleur.

Publie, pour chaque tag autorisé (1, 4, 7, 9 par défaut), sa pose dans le
repère optique, dans le repère du robot et dans `map` quand la TF existe.
Republie aussi l'image (cadence limitée) quand un tag est vu : c'est l'image
de preuve envoyée au statut des organisateurs lors d'une visite.
"""

import time

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from caytu_config import load_config
from caytu_interfaces.msg import TagDetection, TagDetectionArray
from caytu_perception.apriltag_opencv import DetecteurAprilTag, estimer_pose
from caytu_perception.transformations import composer, quaternion_depuis_matrice


def _remplir_pose(pose_stamped, entete, repere, position, orientation):
    pose_stamped.header.stamp = entete.stamp
    pose_stamped.header.frame_id = repere
    (pose_stamped.pose.position.x, pose_stamped.pose.position.y,
     pose_stamped.pose.position.z) = (float(v) for v in position)
    (pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y,
     pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w) = (
        float(v) for v in orientation)


class DetecteurTags(Node):

    def __init__(self, configuration=None):
        super().__init__('tag_detector')
        self._configuration = configuration or load_config()
        tag = self._configuration.perception.apriltag
        camera = tag.cameras[0]
        self.declare_parameter('camera_name', camera.name)
        self.declare_parameter('image_topic', camera.color_topic)
        self.declare_parameter('camera_info_topic', camera.info_topic)
        self.declare_parameter(
            'output_topic', self._configuration.perception.tags_topic)
        self.declare_parameter('image_output_topic', tag.detection_image_topic)

        self._nom_camera = self.get_parameter('camera_name').value
        self._reglages = tag
        self._repere_robot = self._configuration.system.base_frame
        self._repere_global = self._configuration.system.global_frame
        self._delai_tf = Duration(seconds=tag.tf_timeout_sec)
        self._retard_max = Duration(seconds=tag.tf_max_lag_sec)
        self._periode = 1.0 / tag.max_rate_hz
        self._periode_image = 1.0 / tag.detection_image_rate_hz
        self._derniere_image = 0.0
        self._derniere_preuve = 0.0
        self._matrice = None
        self._distorsion = None

        self._detecteur = DetecteurAprilTag(
            tag.family, tag.min_marker_distance_rate, tag.ids)
        self._pont = CvBridge()
        self._memoire_tf = Buffer()
        # Fil dédié : la recherche de TF avec délai ne bloque pas les callbacks.
        self._ecouteur_tf = TransformListener(
            self._memoire_tf, self, spin_thread=True)

        self._publicateur = self.create_publisher(
            TagDetectionArray, self.get_parameter('output_topic').value, 10)
        self._publicateur_image = self.create_publisher(
            Image, self.get_parameter('image_output_topic').value, 2)
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self._recevoir_infos_camera, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.get_parameter('image_topic').value,
            self._recevoir_image, qos_profile_sensor_data)
        self.get_logger().info(
            f'Détection AprilTag {tag.family} {list(tag.ids)} sur '
            f'{self.get_parameter("image_topic").value} ({self._nom_camera}).')

    def _recevoir_infos_camera(self, message):
        self._matrice = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        distorsion = np.asarray(message.d, dtype=np.float64)
        self._distorsion = distorsion if distorsion.size else np.zeros(5)

    # ----------------------------------------------------------------- image
    def _recevoir_image(self, message):
        maintenant = time.monotonic()
        if self._matrice is None or maintenant - self._derniere_image < self._periode:
            return
        self._derniere_image = maintenant
        try:
            image = self._pont.imgmsg_to_cv2(message, desired_encoding='mono8')
            detections = self._detecteur.detecter(image)
        except Exception as erreur:  # image corrompue ou encodage inattendu
            self.get_logger().warning(
                f'Image ignorée ({self._nom_camera}) : {erreur}',
                throttle_duration_sec=5.0)
            return

        sortie = TagDetectionArray()
        sortie.header = message.header
        if detections:
            vers_robot = self._chercher_tf(self._repere_robot, message.header)
            vers_carte = self._chercher_tf(self._repere_global, message.header)
            for detection in detections:
                tag = self._convertir(
                    message.header, detection, vers_robot, vers_carte)
                if tag is not None:
                    sortie.detections.append(tag)
        self._publicateur.publish(sortie)

        if sortie.detections and (
                maintenant - self._derniere_preuve >= self._periode_image):
            self._derniere_preuve = maintenant
            self._publicateur_image.publish(message)

    def _chercher_tf(self, repere_cible, entete):
        """(translation, rotation, exacte) cible <- caméra, ou None.

        À l'instant de l'image si possible ; sinon la dernière transformation
        connue, si elle n'a pas plus de `tf_max_lag_sec` d'écart avec l'image
        (odométrie publiée après l'image).
        """
        instant = Time.from_msg(entete.stamp)
        exacte = True
        try:
            transformation = self._memoire_tf.lookup_transform(
                repere_cible, entete.frame_id, instant, timeout=self._delai_tf)
        except TransformException:
            exacte = False
            try:
                transformation = self._memoire_tf.lookup_transform(
                    repere_cible, entete.frame_id, Time())
            except TransformException:
                return None
            ecart = instant - Time.from_msg(transformation.header.stamp)
            if abs(ecart.nanoseconds) > self._retard_max.nanoseconds:
                return None
        translation = transformation.transform.translation
        rotation = transformation.transform.rotation
        return ((translation.x, translation.y, translation.z),
                (rotation.x, rotation.y, rotation.z, rotation.w), exacte)

    def _convertir(self, entete, detection, vers_robot, vers_carte):
        pose = estimer_pose(
            detection.coins, self._reglages.size_m, self._matrice,
            self._distorsion)
        if pose is None:
            return None
        rotation, translation = pose
        distance = float(np.linalg.norm(translation))
        if distance > self._reglages.max_distance_m:
            return None
        orientation = quaternion_depuis_matrice(rotation)

        tag = TagDetection()
        tag.header = entete
        tag.tag_id = detection.tag_id
        tag.distance_m = distance
        tag.confidence = 1.0
        _remplir_pose(tag.pose_camera, entete, entete.frame_id,
                      translation, orientation)
        if vers_robot is not None:
            _remplir_pose(tag.pose_robot, entete, self._repere_robot,
                          *composer(*vers_robot[:2], translation, orientation))
            tag.has_pose_robot = True
        if vers_carte is not None:
            _remplir_pose(tag.pose_map, entete, self._repere_global,
                          *composer(*vers_carte[:2], translation, orientation))
            tag.has_pose_map = True
            if not vers_carte[2]:
                tag.confidence = 0.5    # TF approchée : pose map moins fiable
        return tag


def main(args=None):
    rclpy.init(args=args)
    noeud = DetecteurTags()
    try:
        rclpy.spin(noeud)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        noeud.destroy_node()
        rclpy.try_shutdown()
