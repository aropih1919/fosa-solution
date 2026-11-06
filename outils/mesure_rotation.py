#!/usr/bin/env python3
"""Odométrie en rotation du robot réel (créneau d'essai, ~1 min, robot dégagé).

Le robot fait deux tours lents sur place. Trois mesures du même angle :
- odométrie (navigation.odom_topic) : le robot s'arrête à 720° d'odométrie ;
- gyromètre d'une caméra (Imu ramené dans base_frame, biais retiré) ;
- lidar : si le robot a vraiment fait 720°, le scan final recouvre le scan
  initial ; le décalage qui les aligne est l'erreur de l'odométrie.
Mesuré en simulation : une odométrie qui surestime les rotations de 15 %
fait dériver le cap de slam_toolbox de dizaines de degrés pendant les tours
de recherche, et les poses de tags mémorisées deviennent fausses.

Usage : ./run.sh mesure-rotation [--tours 2] [--vitesse 0.3] [--gyro TOPIC]
La course ne doit pas tourner (une seule source de commandes).
"""

import argparse
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from caytu_config import load_config

SEUIL_ALERTE = 0.03          # écart relatif au-delà duquel le cap SLAM est fragile


def lacet(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def vitesse_lacet(omega, rotation):
    """Composante z, dans le repère du robot, d'une vitesse angulaire du capteur.

    rotation : quaternion (x, y, z, w) de la TF robot <- capteur.
    """
    x, y, z, w = rotation
    ligne_z = (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y))
    return sum(a * b for a, b in zip(ligne_z, omega))


def integrer(echantillons, biais=0.0):
    """Intégrale (trapèzes) d'échantillons (t, valeur), biais retiré."""
    total = 0.0
    for (t0, v0), (t1, v1) in zip(echantillons, echantillons[1:]):
        total += 0.5 * (v0 + v1 - 2 * biais) * (t1 - t0)
    return total


def decalage_scans(initial, final, increment, decalage_max):
    """Angle δ tel que final[i] ≈ initial[i + δ / increment] (robot tourné de δ en plus)."""
    a, b = np.asarray(initial, dtype=float), np.asarray(final, dtype=float)
    a[a <= 0.0], b[b <= 0.0] = np.nan, np.nan
    meilleur, meilleur_ecart = None, math.inf
    pas_max = int(decalage_max / increment)
    for pas in range(-pas_max, pas_max + 1):
        if pas >= 0:
            x, y = b[:len(b) - pas], a[pas:]
        else:
            x, y = b[-pas:], a[:len(a) + pas]
        valides = np.isfinite(x) & np.isfinite(y)
        if valides.sum() < 0.3 * len(a):
            continue
        ecart = float(np.median(np.abs(x[valides] - y[valides])))
        if ecart < meilleur_ecart:
            meilleur, meilleur_ecart = pas * increment, ecart
    return meilleur, meilleur_ecart


def angle_lidar(odom, decalage):
    """Angle réel : le lidar le donne modulo un tour, l'odométrie choisit le tour."""
    return round((odom - decalage) / math.tau) * math.tau + decalage


class MesureRotation(Node):

    def __init__(self, configuration, topic_gyro):
        super().__init__('mesure_rotation_fosa', parameter_overrides=[
            Parameter('use_sim_time', value=configuration.system.use_sim_time)])
        self.base = configuration.system.base_frame
        self.cap_odom = None
        self.scan = None
        self.gyro = []
        self.repere_gyro = None
        self.tf = Buffer()
        TransformListener(self.tf, self, spin_thread=False)
        navigation = configuration.navigation
        self.commande = self.create_publisher(Twist, navigation.cmd_vel_topic, 10)
        self.create_subscription(Odometry, navigation.odom_topic, self._odom, 20)
        self.create_subscription(LaserScan, navigation.scan_topic, self._scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, topic_gyro, self._gyro, qos_profile_sensor_data)

    def _odom(self, message):
        self.cap_odom = lacet(message.pose.pose.orientation)

    def _scan(self, message):
        self.scan = message

    def _gyro(self, message):
        self.repere_gyro = message.header.frame_id
        v, stamp = message.angular_velocity, message.header.stamp
        instant = stamp.sec + stamp.nanosec * 1e-9 or self.maintenant()
        self.gyro.append((instant, (v.x, v.y, v.z)))

    def maintenant(self):
        """Horloge ROS (temps simulé en simulation, comme la course)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def attendre(self, duree):
        limite = time.monotonic() + 10.0
        while self.maintenant() == 0.0 and time.monotonic() < limite:
            rclpy.spin_once(self, timeout_sec=0.05)      # horloge simulée pas encore reçue
        fin = self.maintenant() + duree
        while self.maintenant() < fin:
            rclpy.spin_once(self, timeout_sec=0.02)

    def arreter(self):
        for _ in range(5):
            self.commande.publish(Twist())
            self.attendre(0.05)


def gyro_robot(noeud, echantillons):
    try:
        q = noeud.tf.lookup_transform(noeud.base, noeud.repere_gyro, Time()).transform.rotation
    except TransformException as erreur:
        print(f'  [ÉCHEC] TF {noeud.base} <- {noeud.repere_gyro} : {erreur}')
        return None
    return [(t, vitesse_lacet(w, (q.x, q.y, q.z, q.w))) for t, w in echantillons]


def tourner(noeud, angle, vitesse):
    """Tourne jusqu'à `angle` d'odométrie, puis attend l'arrêt complet.

    Renvoie l'angle d'odométrie, glissement après la commande d'arrêt compris.
    """
    commande = Twist()
    commande.angular.z = vitesse
    cumul, precedent = 0.0, noeud.cap_odom
    limite = noeud.maintenant() + 2.0 * angle / vitesse + 10.0
    while cumul < angle and noeud.maintenant() < limite:
        noeud.commande.publish(commande)
        noeud.attendre(0.05)
        cumul += math.remainder(noeud.cap_odom - precedent, math.tau)
        precedent = noeud.cap_odom
    noeud.arreter()
    noeud.attendre(2.0)
    return cumul + math.remainder(noeud.cap_odom - precedent, math.tau)


def rapport(odom, gyro, decalage):
    print(f'\n  odométrie : {math.degrees(odom):.1f}°')
    reference = None
    if gyro is not None:
        print(f'  gyromètre : {math.degrees(gyro):.1f}°')
        reference = gyro
    if decalage is not None:
        lidar = angle_lidar(odom, decalage)
        print(f'  lidar     : {math.degrees(lidar):.1f}° (décalage final/initial '
              f'{math.degrees(decalage):+.1f}°, valable si |erreur| < 30°)')
        reference = lidar if reference is None else reference
    if reference:
        erreur = odom / reference - 1.0
        verdict = 'OK' if abs(erreur) <= SEUIL_ALERTE else 'À SIGNALER'
        print(f'  odométrie / référence : {erreur * 100:+.1f} % -> {verdict}')
        if verdict != 'OK':
            print('  Cap SLAM fragile pendant les tours : réduire les tours de recherche '
                  'et vérifier la carte (./run.sh sauver-carte) après un essai.')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--tours', type=float, default=2.0)
    parser.add_argument('--vitesse', type=float, default=0.3, help='rad/s')
    parser.add_argument('--gyro', default='/camera1/camera1/gyro/sample')
    arguments = parser.parse_args()
    configuration = load_config()
    vitesse = min(abs(arguments.vitesse), configuration.navigation.max_angular_speed)
    rclpy.init()
    noeud = MesureRotation(configuration, arguments.gyro)
    try:
        noeud.attendre(3.0)
        if noeud.cap_odom is None or noeud.scan is None:
            print('[ÉCHEC] odométrie ou scan absent : robot démarré ? domaine ROS ?')
            return
        if not noeud.gyro:
            print(f'  [INFO] aucun échantillon sur {arguments.gyro} : mesure sans gyromètre')
        immobile = list(noeud.gyro)
        scan_initial = noeud.scan
        debut = len(noeud.gyro)
        print(f'Rotation de {arguments.tours:.1f} tour(s) à {vitesse:.2f} rad/s...')
        odom = tourner(noeud, arguments.tours * math.tau, vitesse)
        gyro = None
        if len(immobile) > 10 and len(noeud.gyro) > debut + 10:
            fixes, rotation = gyro_robot(noeud, immobile), gyro_robot(noeud, noeud.gyro[debut:])
            if fixes and rotation:
                biais = integrer(fixes) / max(fixes[-1][0] - fixes[0][0], 1e-6)
                gyro = integrer(rotation, biais)
        decalage, _ = decalage_scans(
            scan_initial.ranges, noeud.scan.ranges, scan_initial.angle_increment,
            math.radians(30.0))
        rapport(odom, gyro, decalage)
    except KeyboardInterrupt:
        pass
    finally:
        noeud.arreter()
        noeud.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
