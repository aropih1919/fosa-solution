"""Détection AprilTag, annotations et publication facultative de poses dans map."""

import math
from pathlib import Path
import tempfile
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from caytu_config import load_config
from caytu_interfaces.msg import TagDetection, TagDetectionArray
from caytu_perception.tags_image import (
    DetecteurImage, annoter_image, enregistrer_image)


def _quaternion_depuis_matrice(matrice):
    """Calcule le quaternion (x, y, z, w) correspondant à une matrice de rotation 3x3."""
    trace = matrice[0, 0] + matrice[1, 1] + matrice[2, 2]
    if trace > 0.0:
        echelle = math.sqrt(trace + 1.0) * 2.0
        return (float((matrice[2, 1] - matrice[1, 2]) / echelle),
                float((matrice[0, 2] - matrice[2, 0]) / echelle),
                float((matrice[1, 0] - matrice[0, 1]) / echelle),
                float(0.25 * echelle))
    index = int(np.argmax(np.diag(matrice)))
    prochain_index = (index + 1) % 3
    dernier_index = (index + 2) % 3
    echelle = math.sqrt(
        1.0 + matrice[index, index] - matrice[prochain_index, prochain_index]
        - matrice[dernier_index, dernier_index]) * 2.0
    resultat = [0.0, 0.0, 0.0, 0.0]
    resultat[index] = 0.25 * echelle
    resultat[3] = (matrice[dernier_index, prochain_index]
                   - matrice[prochain_index, dernier_index]) / echelle
    resultat[prochain_index] = (matrice[prochain_index, index]
                                + matrice[index, prochain_index]) / echelle
    resultat[dernier_index] = (matrice[dernier_index, index]
                               + matrice[index, dernier_index]) / echelle
    return tuple(float(valeur) for valeur in resultat)


def _multiplier_quaternions(gauche, droite):
    """Produit de deux quaternions de rotation."""
    gx, gy, gz, gw = gauche
    dx, dy, dz, dw = droite
    return (
        gw * dx + gx * dw + gy * dz - gz * dy,
        gw * dy - gx * dz + gy * dw + gz * dx,
        gw * dz + gx * dy - gy * dx + gz * dw,
        gw * dw - gx * dx - gy * dy - gz * dz,
    )


def _tourner_vecteur(quaternion, vecteur):
    """Applique une rotation par quaternion à un vecteur 3D."""
    x, y, z, w = quaternion
    vx, vy, vz = vecteur
    qv = (x, y, z)
    cross1 = (qv[1] * vz - qv[2] * vy,
              qv[2] * vx - qv[0] * vz,
              qv[0] * vy - qv[1] * vx)
    cross2 = (qv[1] * cross1[2] - qv[2] * cross1[1],
              qv[2] * cross1[0] - qv[0] * cross1[2],
              qv[0] * cross1[1] - qv[1] * cross1[0])
    return tuple(vecteur[i] + 2.0 * (w * cross1[i] + cross2[i])
                 for i in range(3))


class DetecteurTags(Node):
    """Détecte les AprilTags sans carte ; peut aussi publier leurs poses dans `map`.

    Accessible débutant-intermédiaire :
      - Reçoit l'image de la caméra haute et ses paramètres de distorsion ;
      - Détecte les balises de la famille configurée (`36h11` par défaut) ;
      - Calcule la distance et l'orientation via OpenCV `solvePnP` ;
      - Transforme la pose de la caméra vers le repère global `map` grâce à TF2 ;
      - Publie un tableau de détections sur le topic `/perception/tags`.
    """

    def __init__(self, configuration=None):
        super().__init__('tag_detector')
        self._configuration = configuration or load_config()
        self._bridge = CvBridge()
        self._camera_matrix = None
        self._distortion = None
        reglages = self._configuration.perception.apriltag
        for nom in ('publish_map_pose', 'annotated_directory',
                    'save_interval_sec', 'max_saved_images'):
            self.declare_parameter(nom, getattr(reglages, nom))
        self._publier_pose = self.get_parameter('publish_map_pose').value
        self._intervalle = self.get_parameter('save_interval_sec').value
        self._limite_images = self.get_parameter('max_saved_images').value
        if self._intervalle <= 0 or self._limite_images < 1:
            raise ValueError('Intervalle et limite des images doivent être positifs.')
        dossier = self.get_parameter('annotated_directory').value
        self._dossier_images = None
        if dossier:
            racine = Path(dossier).expanduser()
            racine.mkdir(parents=True, exist_ok=True)
            self._dossier_images = Path(tempfile.mkdtemp(prefix='session_', dir=racine))
            self.get_logger().info(f'Images annotées : {self._dossier_images}')
        self._nombre_images = 0
        self._dernier_enregistrement = None
        self._detecteur_image = DetecteurImage(reglages.family)
        self._tf_buffer = Buffer() if self._publier_pose else None
        self._tf_listener = (TransformListener(self._tf_buffer, self, spin_thread=False)
                             if self._publier_pose else None)
        self.declare_parameter(
            'image_topic', self._configuration.perception.top_color_topic)
        self.declare_parameter(
            'camera_info_topic', self._configuration.perception.top_info_topic)
        self.declare_parameter(
            'output_topic', self._configuration.perception.tags_topic)
        self._publisher = (self.create_publisher(
            TagDetectionArray, self.get_parameter('output_topic').value, 10)
            if self._publier_pose else None)
        self._camera_sub = self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self._recevoir_camera_info, qos_profile_sensor_data)
        self._image_sub = self.create_subscription(
            Image, self.get_parameter('image_topic').value,
            self._recevoir_image, qos_profile_sensor_data)

    def _recevoir_camera_info(self, message):
        """Mémorise la matrice intrinsèque K et les coefficients de distorsion D."""
        self._camera_matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        self._distortion = np.asarray(message.d, dtype=np.float64)

    def _detecter_marqueurs(self, image):
        """Exécute l'algorithme ArUco/AprilTag d'OpenCV sur l'image en niveaux de gris."""
        return self._detecteur_image.detecter(image)

    def _enregistrer_annotation(self, image, coins, ids, message):
        """Échantillonne les images avec une horloge monotone et une limite disque."""
        if self._dossier_images is None or self._nombre_images >= self._limite_images:
            return
        maintenant = time.monotonic()
        if (self._dernier_enregistrement is not None
                and maintenant - self._dernier_enregistrement < self._intervalle):
            return
        stamp = message.header.stamp
        nom = f'{self._nombre_images:04d}_{stamp.sec}_{stamp.nanosec:09d}.png'
        try:
            enregistrer_image(self._dossier_images / nom, annoter_image(image, coins, ids))
        except (OSError, cv2.error) as erreur:
            self.get_logger().error(f'Enregistrement désactivé : {erreur}')
            self._dossier_images = None
            return
        self._nombre_images += 1
        self._dernier_enregistrement = maintenant
        identifiants = [] if ids is None else ids.flatten().tolist()
        self.get_logger().info(f'{nom} : IDs détectés {identifiants}')

    def _recevoir_image(self, message):
        """Traite une nouvelle image caméra, extrait les tags et publie leurs poses."""
        try:
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding='mono8')
            coins, identifiants = self._detecter_marqueurs(image)
        except (cv2.error, ValueError) as erreur:
            self.get_logger().warning(f'Image AprilTag ignorée : {erreur}')
            return
        self._enregistrer_annotation(image, coins, identifiants, message)
        # Les IDs et annotations restent disponibles sans CameraInfo ni map.
        # Aucune pose fictive n'est envoyée aux consommateurs de /perception/tags.
        if not self._publier_pose or self._camera_matrix is None:
            return
        resultat = TagDetectionArray()
        resultat.header = message.header
        if identifiants is not None:
            taille_tag = self._configuration.perception.apriltag.size_m
            demi = taille_tag / 2.0
            points_objet = np.asarray([
                [-demi, demi, 0.0], [demi, demi, 0.0],
                [demi, -demi, 0.0], [-demi, -demi, 0.0],
            ], dtype=np.float64)
            for coin, identifiant in zip(coins, identifiants.flatten()):
                points_image = coin.reshape(4, 2).astype(np.float64)
                trouve, vecteur_rotation, translation = cv2.solvePnP(
                    points_objet, points_image,
                    self._camera_matrix, self._distortion)
                if not trouve:
                    continue
                distance = float(np.linalg.norm(translation))
                if distance > self._configuration.perception.apriltag.max_distance_m:
                    continue
                try:
                    detection = self._convertir_en_detection_carte(
                        message, int(identifiant), vecteur_rotation,
                        translation.reshape(3), distance)
                except TransformException:
                    continue
                resultat.detections.append(detection)
        self._publisher.publish(resultat)

    def _convertir_en_detection_carte(self, image, tag_id, rotation_vector,
                                     translation, distance):
        """Transforme la pose relative caméra vers le repère global `map` via TF2."""
        transform = self._tf_buffer.lookup_transform(
            self._configuration.semantic_map.frame,
            image.header.frame_id,
            Time.from_msg(image.header.stamp))
        rotation, _ = cv2.Rodrigues(rotation_vector)
        q_camera_tag = _quaternion_depuis_matrice(rotation)
        q_map_camera_msg = transform.transform.rotation
        q_map_camera = (q_map_camera_msg.x, q_map_camera_msg.y,
                        q_map_camera_msg.z, q_map_camera_msg.w)
        q_map_tag = _multiplier_quaternions(q_map_camera, q_camera_tag)
        rotated = _tourner_vecteur(q_map_camera, tuple(translation))
        detection = TagDetection()
        detection.header = image.header
        detection.tag_id = tag_id
        detection.pose_map.header.stamp = image.header.stamp
        detection.pose_map.header.frame_id = self._configuration.semantic_map.frame
        detection.pose_map.pose.position.x = transform.transform.translation.x + rotated[0]
        detection.pose_map.pose.position.y = transform.transform.translation.y + rotated[1]
        detection.pose_map.pose.position.z = transform.transform.translation.z + rotated[2]
        (detection.pose_map.pose.orientation.x,
         detection.pose_map.pose.orientation.y,
         detection.pose_map.pose.orientation.z,
         detection.pose_map.pose.orientation.w) = q_map_tag
        detection.distance_m = distance
        detection.confidence = 1.0
        return detection

def main(args=None):
    rclpy.init(args=args)
    node = DetecteurTags()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
