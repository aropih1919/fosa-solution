"""Nœud profondeur -> LaserScan d'obstacles (une instance par caméra).

Entrée : image de profondeur + calibration (robot réel) ou nuage de points
(Gazebo). Sortie : LaserScan dans le repère du robot, consommé par les
costmaps de Nav2 comme une source d'obstacles (`navigation.obstacle_sources`).
"""

import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener

from caytu_config import load_config
from caytu_perception.fraicheur_capteurs import horodatage_recent, nanosecondes_stamp
from caytu_perception.scan_profondeur import (
    points_depuis_profondeur,
    scan_depuis_points,
    transformer_points,
)

# Encodage ROS -> (type numpy, facteur vers les mètres).
ENCODAGES = {'16UC1': (np.uint16, 0.001), 'mono16': (np.uint16, 0.001),
             '32FC1': (np.float32, 1.0)}


class NoeudScanProfondeur(Node):

    def __init__(self, configuration):
        super().__init__('scan_profondeur')
        scans = {scan.name: scan for scan in configuration.perception.depth_scans}
        self.declare_parameter('source_name', '')
        nom = self.get_parameter('source_name').value
        if nom not in scans:
            raise ValueError(
                f'source_name « {nom} » absent de perception.depth_scans : {sorted(scans)}')
        self._reglages = scans[nom]
        self._age_max = configuration.navigation.securite.age_scan_max_sec
        self._delai_tf = configuration.perception.apriltag.tf_timeout_sec
        self._repere_robot = configuration.system.base_frame
        self._periode = 1.0 / self._reglages.rate_hz
        self._dernier = 0.0
        self._matrice = None
        self._memoire_tf = Buffer()
        self._ecouteur_tf = TransformListener(self._memoire_tf, self, spin_thread=True)
        self._publicateur = self.create_publisher(
            LaserScan, self._reglages.output_topic, 10)
        if self._reglages.input_type == 'depth_image':
            self.create_subscription(
                CameraInfo, self._reglages.info_topic, self._recevoir_infos,
                qos_profile_sensor_data)
            self.create_subscription(
                Image, self._reglages.input_topic, self._recevoir_image,
                qos_profile_sensor_data)
        else:
            self.create_subscription(
                PointCloud2, self._reglages.input_topic, self._recevoir_nuage,
                qos_profile_sensor_data)
        self.get_logger().info(
            f'Obstacles {self._reglages.min_height:.2f}-{self._reglages.max_height:.2f} m '
            f'depuis {self._reglages.input_topic} -> {self._reglages.output_topic}.')

    def _recevoir_infos(self, message):
        premiere = self._matrice is None
        self._matrice = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        ecart = self._reglages.pixel_step / max(self._matrice[0, 0], 1e-6)
        if premiere and ecart > self._reglages.angle_increment:
            self.get_logger().warning(
                f'Deux colonnes échantillonnées sont à {ecart:.4f} rad, plus qu\'un '
                f'secteur ({self._reglages.angle_increment:.4f}) : des secteurs vides '
                'seront lus comme libres. Augmenter angle_increment ou baisser pixel_step.')

    def _a_traiter(self):
        maintenant = time.monotonic()
        if maintenant - self._dernier < self._periode:
            return False
        self._dernier = maintenant
        return True

    def _recevoir_image(self, message):
        if self._matrice is None or not self._a_traiter():
            return
        encodage = ENCODAGES.get(message.encoding)
        if encodage is None:
            self.get_logger().warning(
                f'Encodage de profondeur non géré : {message.encoding}',
                throttle_duration_sec=10.0)
            return
        type_pixel, echelle = encodage
        largeur_ligne = message.step // np.dtype(type_pixel).itemsize
        profondeur = np.asarray(message.data, dtype=np.uint8).view(type_pixel).reshape(
            message.height, largeur_ligne)[:, :message.width]
        self._publier(message.header, points_depuis_profondeur(
            profondeur, self._matrice, self._reglages.pixel_step, echelle))

    def _recevoir_nuage(self, message):
        if not self._a_traiter():
            return
        from sensor_msgs_py import point_cloud2
        brut = point_cloud2.read_points(
            message, field_names=('x', 'y', 'z'), skip_nans=True)
        points = np.stack([brut['x'], brut['y'], brut['z']], axis=1).astype(np.float64)
        pas = self._reglages.pixel_step
        self._publier(message.header, points[::pas * pas])

    def _publier(self, entete, points):
        # Adaptation du contrôle de b4fae57 : une caméra muette ou une image
        # périmée ne doit pas effacer les obstacles de la costmap.
        if (not len(points) or not entete.frame_id or not horodatage_recent(
                nanosecondes_stamp(entete.stamp), self.get_clock().now().nanoseconds,
                self._age_max)):
            return
        try:
            transformation = self._memoire_tf.lookup_transform(
                self._repere_robot, entete.frame_id, Time.from_msg(entete.stamp),
                timeout=Duration(seconds=self._delai_tf))
        except TransformException as erreur:
            self.get_logger().warning(
                f'TF {self._repere_robot} <- {entete.frame_id} absente : {erreur}',
                throttle_duration_sec=10.0)
            return
        if not horodatage_recent(nanosecondes_stamp(entete.stamp),
                                 self.get_clock().now().nanoseconds, self._age_max):
            return
        t, q = transformation.transform.translation, transformation.transform.rotation
        reglages = self._reglages
        points = transformer_points(points, (t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        scan = LaserScan()
        scan.header.stamp = entete.stamp
        scan.header.frame_id = self._repere_robot
        scan.angle_min = reglages.angle_min
        scan.angle_increment = reglages.angle_increment
        scan.range_min = reglages.range_min
        scan.range_max = reglages.range_max
        scan.ranges = scan_depuis_points(
            points, reglages.min_height, reglages.max_height, reglages.angle_min,
            reglages.angle_max, reglages.angle_increment, reglages.range_min,
            reglages.range_max, reglages.min_points)
        scan.angle_max = reglages.angle_min + reglages.angle_increment * (len(scan.ranges) - 1)
        self._publicateur.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    noeud = NoeudScanProfondeur(load_config())
    try:
        rclpy.spin(noeud)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        noeud.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
