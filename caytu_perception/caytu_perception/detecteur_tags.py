"""Détection AprilTag et publication de poses transformées dans le repère map."""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from caytu_config import load_config
from caytu_interfaces.msg import TagDetection, TagDetectionArray


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
    """Détecte les balises AprilTag par caméra et publie leurs poses dans le repère `map`.

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
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False)
        self.declare_parameter(
            'image_topic', self._configuration.perception.top_color_topic)
        self.declare_parameter(
            'camera_info_topic', self._configuration.perception.top_info_topic)
        self.declare_parameter(
            'output_topic', self._configuration.perception.tags_topic)
        self._publisher = self.create_publisher(
            TagDetectionArray, self.get_parameter('output_topic').value, 10)
        self._camera_sub = self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self._recevoir_camera_info, 10)
        self._image_sub = self.create_subscription(
            Image, self.get_parameter('image_topic').value,
            self._recevoir_image, 10)
        try:
            family = self._configuration.perception.apriltag.family.upper()
            dictionary_name = f'DICT_APRILTAG_{family}'
            dictionary_id = getattr(cv2.aruco, dictionary_name)
            self._dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        except (AttributeError, cv2.error) as error:
            raise RuntimeError(
                'OpenCV contrib avec cv2.aruco est requis pour les AprilTags.') from error

    def _recevoir_camera_info(self, message):
        """Mémorise la matrice intrinsèque K et les coefficients de distorsion D."""
        self._camera_matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        self._distortion = np.asarray(message.d, dtype=np.float64)

    def _detecter_marqueurs(self, image):
        """Exécute l'algorithme ArUco/AprilTag d'OpenCV sur l'image en niveaux de gris."""
        if hasattr(cv2.aruco, 'DetectorParameters'):
            parameters = cv2.aruco.DetectorParameters()
        else:
            parameters = cv2.aruco.DetectorParameters_create()
        if hasattr(cv2.aruco, 'ArucoDetector'):
            corners, ids, _ = cv2.aruco.ArucoDetector(
                self._dictionary, parameters).detectMarkers(image)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                image, self._dictionary, parameters=parameters)
        return corners, ids

    def _recevoir_image(self, message):
        """Traite une nouvelle image caméra, extrait les tags et publie leurs poses."""
        if self._camera_matrix is None:
            return
        try:
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding='mono8')
            corners, ids = self._detecter_marqueurs(image)
        except (cv2.error, ValueError) as error:
            self.get_logger().warning(f'Image AprilTag ignorée : {error}')
            return
        output = TagDetectionArray()
        output.header = message.header
        if ids is not None:
            marker_size = self._configuration.perception.apriltag.size_m
            half = marker_size / 2.0
            object_points = np.asarray([
                [-half, half, 0.0], [half, half, 0.0],
                [half, -half, 0.0], [-half, -half, 0.0],
            ], dtype=np.float64)
            for corner, tag_id in zip(corners, ids.flatten()):
                image_points = corner.reshape(4, 2).astype(np.float64)
                found, rotation_vector, translation = cv2.solvePnP(
                    object_points, image_points,
                    self._camera_matrix, self._distortion)
                if not found:
                    continue
                distance = float(np.linalg.norm(translation))
                if distance > self._configuration.perception.apriltag.max_distance_m:
                    continue
                try:
                    detection = self._convertir_en_detection_carte(
                        message, int(tag_id), rotation_vector,
                        translation.reshape(3), distance)
                except TransformException:
                    continue
                output.detections.append(detection)
        self._publisher.publish(output)

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
