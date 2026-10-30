#!/usr/bin/env python3
"""Corrige les effets de l'inclinaison du robot sur les capteurs.

Le Sito-É repose sur deux roues et une roulette arrière qui descend 1 cm plus
bas que les roues : il penche donc d'environ 2,5° vers l'avant. La TF officielle
(odom -> base_footprint) est plane et l'ignore. Deux conséquences, mesurées dans
nos journaux de diagnostic :
  1. des rayons du lidar touchent le sol et créent de faux obstacles ;
  2. pour les caméras, le sol « monte » de 4 cm par mètre dans base_footprint.

Ce nœud mesure l'inclinaison réelle avec l'accéléromètre de l'IMU (robot à
l'arrêt, au démarrage), puis :
  - publie /scan_clean  : scan sans les retours du sol (pour MARQUER) ;
  - publie /scan_clear  : scan où le sol devient « libre jusque-là » (pour
    EFFACER), afin que la costmap continue à se nettoyer devant le robot ;
  - publie la TF statique base_footprint -> base_footprint_level, un repère
    réellement horizontal dans lequel les nuages des caméras sont filtrés.

Rien n'est modifié dans le URDF officiel.
"""

import math
import statistics

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped, Vector3Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import StaticTransformBroadcaster

from caytu_nav_solution.nav_math import (
    GRAVITE,
    distances_sol,
    hauteur_lidar,
    inclinaison_depuis_acceleration,
    quaternion_depuis_roulis_tangage_lacet,
    separer_retours_sol,
)


class FiltreSolLidar(Node):

    def __init__(self):
        super().__init__('filtre_sol_lidar')

        self._declarer_parametres()
        self._lire_parametres()
        self._initialiser_etat()
        self._connecter_ros()

        # On publie tout de suite un repère horizontal avec l'inclinaison par
        # défaut, pour que les caméras puissent démarrer avant la mesure IMU.
        self.publier_repere_horizontal()
        self.get_logger().info(
            f'Inclinaison par défaut {math.degrees(self._tangage):.2f}° '
            '(géométrie du URDF) en attendant la mesure IMU.')

    # ------------------------------------------------------------ réglages
    def _declarer_parametres(self):
        self.declare_parameter('scan_in', '/scan_filtered')
        self.declare_parameter('scan_mark', '/scan_clean')
        self.declare_parameter('scan_clear', '/scan_clear')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('level_frame', 'base_footprint_level')

        # Géométrie lue dans le URDF officiel (sitoe_robot.urdf) :
        #   capteur lidar à z = 0.0381 + 0.01 m, incliné de -2,5° (nez relevé) ;
        #   lidar 0.116 m derrière l'axe des roues ;
        #   roulette 9,8 mm plus basse que les roues, 0.228 m derrière l'axe.
        self.declare_parameter('lidar_mount_pitch', -0.04363323129985824)
        self.declare_parameter('lidar_mount_z', 0.0481)
        self.declare_parameter('lidar_lever_x', 0.116)
        self.declare_parameter('default_robot_pitch', 0.0429)

        # Nombre de mesures IMU immobiles avant de figer l'inclinaison.
        self.declare_parameter('tilt_samples', 150)

        # Un retour à plus de floor_ratio fois la distance du sol est du sol.
        self.declare_parameter('floor_ratio', 0.80)
        self.declare_parameter('clear_ratio', 0.70)

        # Forçage manuel : distance (m) de la ligne de sol droit devant le
        # lidar, mesurée dans /scan. 0.0 = calcul automatique par l'IMU.
        self.declare_parameter('floor_line_distance', 0.0)

    def _lire_parametres(self):
        self._repere_robot = self.get_parameter('base_frame').value
        self._repere_horizontal = self.get_parameter('level_frame').value
        self._tangage_fixation = float(self.get_parameter('lidar_mount_pitch').value)
        self._hauteur_fixation = float(self.get_parameter('lidar_mount_z').value)
        self._bras_x = float(self.get_parameter('lidar_lever_x').value)
        self._ratio_sol = float(self.get_parameter('floor_ratio').value)
        self._ratio_effacement = float(self.get_parameter('clear_ratio').value)
        self._ligne_forcee = float(self.get_parameter('floor_line_distance').value)
        self._nombre_mesures = int(self.get_parameter('tilt_samples').value)

    def _initialiser_etat(self):
        self._roulis = 0.0
        self._tangage = float(self.get_parameter('default_robot_pitch').value)
        self._inclinaison_figee = False
        self._mesures = []
        self._cache_sol = None          # (clé du scan, distances du sol)
        self._dernier_nombre_sol_logge = -1

    def _connecter_ros(self):
        self._publicateur_tf_statique = StaticTransformBroadcaster(self)
        self._publicateur_inclinaison = self.create_publisher(
            Vector3Stamped, '/robot_tilt', 10)

        # Publication fiable : compatible avec les abonnés « best effort » de
        # Nav2 comme avec RViz.
        self._publicateur_marquage = self.create_publisher(
            LaserScan, self.get_parameter('scan_mark').value, 10)
        self._publicateur_effacement = self.create_publisher(
            LaserScan, self.get_parameter('scan_clear').value, 10)

        self.create_subscription(
            LaserScan, self.get_parameter('scan_in').value, self.recevoir_scan,
            qos_profile_sensor_data)
        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self.recevoir_imu,
            qos_profile_sensor_data)

    # ------------------------------------------------------------------ IMU
    def recevoir_imu(self, message):
        if self._inclinaison_figee:
            return

        acceleration = message.linear_acceleration
        vitesse_angulaire = message.angular_velocity
        norme = math.sqrt(acceleration.x * acceleration.x
                          + acceleration.y * acceleration.y
                          + acceleration.z * acceleration.z)
        immobile = math.sqrt(vitesse_angulaire.x * vitesse_angulaire.x
                             + vitesse_angulaire.y * vitesse_angulaire.y
                             + vitesse_angulaire.z * vitesse_angulaire.z) < 0.02

        if not immobile or abs(norme - GRAVITE) > 0.5:
            self._mesures.clear()       # le robot bouge : on recommence
            return

        self._mesures.append((acceleration.x, acceleration.y, acceleration.z))
        if len(self._mesures) < self._nombre_mesures:
            return

        # La médiane ignore les quelques mesures aberrantes.
        ax = statistics.median(mesure[0] for mesure in self._mesures)
        ay = statistics.median(mesure[1] for mesure in self._mesures)
        az = statistics.median(mesure[2] for mesure in self._mesures)
        roulis, tangage = inclinaison_depuis_acceleration(ax, ay, az)

        self._inclinaison_figee = True
        self._mesures.clear()

        if abs(tangage) > 0.15 or abs(roulis) > 0.10:
            self.get_logger().warn(
                f'Inclinaison IMU invraisemblable (tangage {math.degrees(tangage):.1f}°, '
                f'roulis {math.degrees(roulis):.1f}°) : valeur par défaut conservée.')
            return

        self._roulis = roulis
        self._tangage = tangage
        self._cache_sol = None
        self.publier_repere_horizontal()

        tangage_plan = self.tangage_plan_lidar()
        ligne = (self.hauteur_lidar_actuelle() / math.sin(tangage_plan)
                 if tangage_plan > 1e-4 else math.inf)
        self.get_logger().info(
            f'Inclinaison mesurée : tangage {math.degrees(tangage):.2f}°, '
            f'roulis {math.degrees(roulis):.2f}°. Plan du lidar : '
            f'{math.degrees(tangage_plan):+.2f}° -> '
            + (f'ligne de sol à {ligne:.2f} m devant le lidar.' if math.isfinite(ligne)
               else 'aucun rayon ne touche le sol.'))

    # ----------------------------------------------------------- géométrie
    def hauteur_lidar_actuelle(self):
        return hauteur_lidar(self._tangage, self._hauteur_fixation, self._bras_x)

    def tangage_plan_lidar(self):
        if self._ligne_forcee > 0.0:
            # Dans /scan, la distance lue droit devant vaut hauteur / sin(tangage).
            return math.asin(min(1.0, self.hauteur_lidar_actuelle() / self._ligne_forcee))
        return self._tangage + self._tangage_fixation

    def publier_repere_horizontal(self):
        """Publie base_footprint -> base_footprint_level (repère horizontal)."""
        # Inverse de l'inclinaison du robot : conjugué du quaternion (r, p, 0).
        qx, qy, qz, qw = quaternion_depuis_roulis_tangage_lacet(
            self._roulis, self._tangage, 0.0)

        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._repere_robot
        message.child_frame_id = self._repere_horizontal
        message.transform.rotation.x = -qx
        message.transform.rotation.y = -qy
        message.transform.rotation.z = -qz
        message.transform.rotation.w = qw
        self._publicateur_tf_statique.sendTransform(message)

    def publier_inclinaison(self, entete):
        """Diagnostic : x = roulis, y = tangage du robot, z = tangage du plan lidar."""
        inclinaison = Vector3Stamped()
        inclinaison.header.stamp = entete.stamp
        inclinaison.header.frame_id = self._repere_robot
        inclinaison.vector.x = self._roulis
        inclinaison.vector.y = self._tangage
        inclinaison.vector.z = self.tangage_plan_lidar()
        self._publicateur_inclinaison.publish(inclinaison)

    # ----------------------------------------------------------------- scan
    def recevoir_scan(self, message):
        # Les distances du sol ne dépendent que de la géométrie du scan : on les
        # recalcule seulement quand le nombre de rayons ou leur pas change.
        cle = (len(message.ranges), message.angle_min, message.angle_increment)
        if self._cache_sol is None or self._cache_sol[0] != cle:
            angles = [message.angle_min + i * message.angle_increment
                      for i in range(len(message.ranges))]
            self._cache_sol = (
                cle,
                distances_sol(
                    angles,
                    self.tangage_plan_lidar(),
                    self._roulis,
                    self.hauteur_lidar_actuelle(),
                ),
            )

        marquage, effacement, nombre_sol = separer_retours_sol(
            message.ranges, self._cache_sol[1], message.range_max,
            self._ratio_sol, self._ratio_effacement)

        # Premier scan : on retire les retours du sol pour ne pas marquer de
        # faux obstacles.
        scan_marquage = LaserScan()
        scan_marquage.header = message.header
        scan_marquage.angle_min = message.angle_min
        scan_marquage.angle_max = message.angle_max
        scan_marquage.angle_increment = message.angle_increment
        scan_marquage.time_increment = message.time_increment
        scan_marquage.scan_time = message.scan_time
        scan_marquage.range_min = message.range_min
        scan_marquage.range_max = message.range_max
        scan_marquage.ranges = marquage
        scan_marquage.intensities = message.intensities
        self._publicateur_marquage.publish(scan_marquage)

        # Second scan : le sol devient « libre jusque-là » pour effacer les
        # anciens obstacles de la costmap.
        scan_effacement = LaserScan()
        scan_effacement.header = message.header
        scan_effacement.angle_min = message.angle_min
        scan_effacement.angle_max = message.angle_max
        scan_effacement.angle_increment = message.angle_increment
        scan_effacement.time_increment = message.time_increment
        scan_effacement.scan_time = message.scan_time
        scan_effacement.range_min = message.range_min
        scan_effacement.range_max = message.range_max
        scan_effacement.ranges = effacement
        scan_effacement.intensities = message.intensities
        self._publicateur_effacement.publish(scan_effacement)

        self.publier_inclinaison(message.header)

        # Un seul message quand le nombre de rayons au sol change nettement.
        if (self._inclinaison_figee
                and abs(nombre_sol - self._dernier_nombre_sol_logge) > 10):
            self._dernier_nombre_sol_logge = nombre_sol
            self.get_logger().info(
                f'{nombre_sol} rayons du lidar attribués au sol et retirés.')


def main(args=None):
    rclpy.init(args=args)
    noeud = FiltreSolLidar()
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
