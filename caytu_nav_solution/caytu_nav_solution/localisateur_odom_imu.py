#!/usr/bin/env python3
"""Localisation sans carte : publie map -> odom à partir des roues et de l'IMU.

Remplace AMCL. Le repère `map` est rendu identique au repère du monde Gazebo en
partant du départ de task_params.yaml ; le but de task_params.yaml s'envoie donc
à Nav2 tel quel. Le cap vient de l'IMU, car l'odométrie de Gazebo surestime les
rotations (entraxe déclaré plus petit que l'entraxe réel).

Entrées : /odom (nav_msgs/Odometry), /imu (sensor_msgs/Imu)
Sorties : /tf (map -> odom), /localization_pose (geometry_msgs/PoseStamped)
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from caytu_nav_solution.calculs_nav import (
    FusionOdomImu,
    Pose2D,
    lacet_depuis_quaternion,
    normaliser_angle,
    quaternion_depuis_lacet,
)
from caytu_nav_solution.parametres_tache import charger_parametres_tache


class LocalisateurOdomImu(Node):

    def __init__(self):
        super().__init__('localisateur_odom_imu')

        self._declarer_parametres()
        self._lire_parametres()
        self._initialiser_etat()
        self._connecter_ros()

        self.get_logger().info(
            f'Localisation roues + IMU : départ lu dans {self._parametres.chemin} '
            f'(x={self._parametres.depart.x:.3f}, y={self._parametres.depart.y:.3f}, '
            f'yaw={self._parametres.depart.yaw:.3f})')

    # ------------------------------------------------------------ réglages
    def _declarer_parametres(self):
        self.declare_parameter('task_params_file', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/imu')
        # Mettre à False pour revenir au cap de l'odométrie (diagnostic).
        self.declare_parameter('use_imu_yaw', True)
        # Au-delà de ce délai sans IMU, on retombe sur le cap de l'odométrie.
        self.declare_parameter('imu_timeout', 0.5)
        # La TF est post-datée comme le fait AMCL, pour rester utilisable entre
        # deux messages d'odométrie.
        self.declare_parameter('transform_tolerance', 0.2)

    def _lire_parametres(self):
        self._parametres = charger_parametres_tache(
            self.get_parameter('task_params_file').value or None)
        self._repere_map = self.get_parameter('map_frame').value
        self._repere_odom = self.get_parameter('odom_frame').value
        self._utiliser_imu = bool(self.get_parameter('use_imu_yaw').value)
        self._delai_imu = Duration(
            seconds=float(self.get_parameter('imu_timeout').value))
        self._tolerance = Duration(
            seconds=float(self.get_parameter('transform_tolerance').value))

    def _initialiser_etat(self):
        self._fusion = FusionOdomImu(self._parametres.depart)
        self._lacet_imu = None            # lacet absolu courant donné par l'IMU
        self._horodatage_imu = None
        self._lacet_gyro = 0.0            # secours : intégration du gyroscope
        self._dernier_horodatage_gyro = None
        self._odom_recue = False
        self._mode_imu_logge = None

    def _connecter_ros(self):
        self._publicateur_tf = TransformBroadcaster(self)
        self._publicateur_pose = self.create_publisher(
            PoseStamped, '/localization_pose', 10)

        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.recevoir_odom,
            qos_profile_sensor_data)
        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self.recevoir_imu,
            qos_profile_sensor_data)

        # Tant que l'odométrie n'est pas arrivée, Nav2 a quand même besoin de
        # map -> odom pour démarrer ses costmaps.
        self._minuteur_demarrage = self.create_timer(
            0.1, self.publier_transformation_demarrage)

    # ------------------------------------------------------------------ IMU
    def recevoir_imu(self, message):
        horodatage = Time.from_msg(message.header.stamp)
        quaternion = message.orientation
        norme = math.sqrt(quaternion.x * quaternion.x
                          + quaternion.y * quaternion.y
                          + quaternion.z * quaternion.z
                          + quaternion.w * quaternion.w)
        orientation_valide = (abs(norme - 1.0) < 0.05
                              and message.orientation_covariance[0] >= 0.0)

        # Secours : intégration de la vitesse de lacet si l'orientation manque.
        if self._dernier_horodatage_gyro is not None:
            dt = (horodatage - self._dernier_horodatage_gyro).nanoseconds * 1e-9
            if 0.0 < dt < 0.5:
                self._lacet_gyro = normaliser_angle(
                    self._lacet_gyro + message.angular_velocity.z * dt)
        self._dernier_horodatage_gyro = horodatage

        if orientation_valide:
            self._lacet_imu = lacet_depuis_quaternion(
                quaternion.x, quaternion.y, quaternion.z, quaternion.w)
            mode = 'orientation'
        else:
            self._lacet_imu = self._lacet_gyro
            mode = 'gyroscope intégré'

        self._horodatage_imu = horodatage
        if mode != self._mode_imu_logge:
            self._mode_imu_logge = mode
            self.get_logger().info(f'Cap fourni par l\'IMU ({mode}).')

    # ----------------------------------------------------------------- odom
    def recevoir_odom(self, message):
        horodatage = Time.from_msg(message.header.stamp)
        pose = message.pose.pose
        odom = Pose2D(
            pose.position.x,
            pose.position.y,
            lacet_depuis_quaternion(
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w),
        )

        lacet_imu = None
        if (self._utiliser_imu
                and self._lacet_imu is not None
                and self._horodatage_imu is not None):
            age = horodatage - self._horodatage_imu
            if abs(age.nanoseconds) <= self._delai_imu.nanoseconds:
                lacet_imu = self._lacet_imu

        utilisait_imu = self._fusion.utilise_imu()
        map_vers_odom = self._fusion.mettre_a_jour(odom, lacet_imu)

        if self._odom_recue and utilisait_imu and not self._fusion.utilise_imu():
            self.get_logger().warn(
                'IMU absente : cap repris sur l\'odométrie (moins précis en virage).')

        if not self._odom_recue:
            self._odom_recue = True
            self._minuteur_demarrage.cancel()
            self.get_logger().info(
                'Première odométrie reçue : map -> odom publié en continu.')

        self.publier_transformation(map_vers_odom, horodatage + self._tolerance)

        pose_estimee = self._fusion.get_pose()
        message_pose = PoseStamped()
        message_pose.header.stamp = message.header.stamp
        message_pose.header.frame_id = self._repere_map
        message_pose.pose.position.x = pose_estimee.x
        message_pose.pose.position.y = pose_estimee.y
        qx, qy, qz, qw = quaternion_depuis_lacet(pose_estimee.yaw)
        message_pose.pose.orientation.x = qx
        message_pose.pose.orientation.y = qy
        message_pose.pose.orientation.z = qz
        message_pose.pose.orientation.w = qw
        self._publicateur_pose.publish(message_pose)

    # ------------------------------------------------------------------- TF
    def publier_transformation_demarrage(self):
        maintenant = self.get_clock().now()
        if maintenant.nanoseconds == 0:
            return                      # /clock pas encore reçu (temps simulé)
        self.publier_transformation(
            self._fusion.transformation_initiale(), maintenant + self._tolerance)

    def publier_transformation(self, transformation, horodatage):
        message = TransformStamped()
        message.header.stamp = horodatage.to_msg()
        message.header.frame_id = self._repere_map
        message.child_frame_id = self._repere_odom
        message.transform.translation.x = transformation.x
        message.transform.translation.y = transformation.y
        qx, qy, qz, qw = quaternion_depuis_lacet(transformation.yaw)
        message.transform.rotation.x = qx
        message.transform.rotation.y = qy
        message.transform.rotation.z = qz
        message.transform.rotation.w = qw
        self._publicateur_tf.sendTransform(message)


def main(args=None):
    rclpy.init(args=args)
    noeud = LocalisateurOdomImu()
    try:
        rclpy.spin(noeud)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Arrêt silencieux : le launch envoie SIGINT à la fermeture.
        try:
            noeud.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
