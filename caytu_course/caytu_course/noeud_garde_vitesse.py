"""Dernière garde Fosa avant /cmd_vel, commune à Nav2 et à l'alignement."""

import math
import json
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from caytu_config import load_config
from caytu_course.securite import donnees_fraiches, trajectoire_libre
from caytu_course.statut import charger_type, qos_statut
from caytu_perception.fraicheur_capteurs import horodatage_recent, nanosecondes_stamp
from caytu_perception.transformations import tourner_vecteur


class GardeVitesse(Node):
    def __init__(self, configuration):
        super().__init__('garde_vitesse')
        self._navigation = configuration.navigation
        self._reglages = self._navigation.securite
        self._repere = configuration.system.base_frame
        self._mode = self.declare_parameter('mode', 'course').value
        self._etat_course = None
        self._debut_course = None
        self._fin_arbitre = False
        self._budget = configuration.course.time_limit_sec - configuration.course.stop_margin_sec
        if self._mode == 'course':
            self.create_subscription(String, configuration.course.state_topic,
                                     self.recevoir_etat, qos_statut(1))
            classe = charger_type('parc_msgs/msg/RunStatus')
            if classe is not None:
                self.create_subscription(classe, configuration.course.referee.run_status_topic,
                                         self.recevoir_arbitre, qos_statut(1))
        self._tf = Buffer()
        self._ecouteur = TransformListener(self._tf, self)
        self._commande = Twist()
        self._instant_commande = None
        self._sources = {source.topic: source for source in self._navigation.obstacle_sources
                         if source.enabled}
        self._scans = {topic: None for topic in self._sources}
        self._sortie = self.create_publisher(Twist, self._navigation.cmd_vel_topic, 10)
        self.create_subscription(Twist, self._reglages.topic_entree,
                                 self.recevoir_commande, 1)
        for topic in self._scans:
            self.create_subscription(
                LaserScan, topic, lambda message, nom=topic: self.recevoir_scan(nom, message),
                qos_profile_sensor_data)
        # Même si /clock s'arrête, une commande abandonnée finit par être annulée.
        self.create_timer(self._reglages.periode_sec, self.publier,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

    def recevoir_commande(self, message):
        self._commande = message
        self._instant_commande = time.monotonic()

    def recevoir_etat(self, message):
        try:
            self._etat_course = json.loads(message.data)['etat']
        except (ValueError, KeyError, TypeError):
            self._etat_course = None
        if (self._etat_course in ('DEMARRAGE', 'RECHERCHE', 'APPROCHE', 'ALIGNEMENT', 'VISITE', 'DEGAGEMENT')
                and self._debut_course is None):
            self._debut_course = time.monotonic()

    def recevoir_arbitre(self, message):
        if self._debut_course is not None and message.state in (
                message.FINISHED, message.TIMED_OUT, message.TERMINATED):
            self._fin_arbitre = True

    def mouvement_autorise(self):
        if self._mode != 'course':
            return True
        return (self._debut_course is not None and not self._fin_arbitre
                and time.monotonic() - self._debut_course < self._budget
                and self._etat_course in ('RECHERCHE', 'APPROCHE', 'ALIGNEMENT', 'DEGAGEMENT'))

    def recevoir_scan(self, topic, message):
        self._scans[topic] = None
        if not horodatage_recent(nanosecondes_stamp(message.header.stamp),
                                 self.get_clock().now().nanoseconds,
                                 self._reglages.age_scan_max_sec):
            return
        if not message.ranges or not any(
                (math.isfinite(r) and message.range_min <= r <= message.range_max)
                or (r == math.inf and self._sources[topic].inf_is_valid)
                for r in message.ranges):
            return
        try:
            tf = self._tf.lookup_transform(self._repere, message.header.frame_id, Time())
        except TransformException:
            return
        t, q = tf.transform.translation, tf.transform.rotation
        points = []
        for index, distance in enumerate(message.ranges):
            if not math.isfinite(distance) or not message.range_min <= distance < message.range_max:
                continue
            angle = message.angle_min + index * message.angle_increment
            x, y, _ = tourner_vecteur(
                (q.x, q.y, q.z, q.w),
                (distance * math.cos(angle), distance * math.sin(angle), 0.0))
            points.append((t.x + x, t.y + y))
        instant = Time.from_msg(message.header.stamp).nanoseconds * 1e-9
        self._scans[topic] = (instant, points, time.monotonic())

    def publier(self):
        reglage = self._reglages
        sortie = Twist()
        maintenant = self.get_clock().now().nanoseconds * 1e-9
        instant_monotone = time.monotonic()
        sources_obligatoires = [
            self._scans[source.topic] for source in self._sources.values()
            if source.required]
        # Une source obligatoire absente ou périmée immobilise le robot.
        frais_obligatoires = donnees_fraiches(
            [scan[0] if scan is not None else None
             for scan in sources_obligatoires],
            maintenant, reglage.age_scan_max_sec)
        frais_obligatoires = frais_obligatoires and donnees_fraiches(
            [scan[2] if scan is not None else None
             for scan in sources_obligatoires],
            instant_monotone, reglage.age_scan_max_sec)
        # Une caméra optionnelle absente est ignorée : le lidar suffit.
        scans_utilises = []
        for source in self._sources.values():
            scan = self._scans[source.topic]
            if scan is None:
                continue
            if not donnees_fraiches(
                    [scan[2]], instant_monotone, reglage.age_scan_max_sec):
                continue
            scans_utilises.append(scan)
        commande_fraiche = donnees_fraiches(
            [self._instant_commande], instant_monotone,
            reglage.age_commande_max_sec)
        v, w = self._commande.linear.x, self._commande.angular.z
        raisons = []
        if not self.mouvement_autorise():
            raisons.append('course inactive ou terminée')
        # Scans : si aucune donnée exploitable (TF cassée, lidar absent), on
        # avertit mais on ne bloque pas tout : le robot reste pilotable et la
        # navigation se dégrade gracieusement au lieu de s'immobiliser.
        scans_disponibles = any(scan is not None for scan in sources_obligatoires)
        if not frais_obligatoires:
            if scans_disponibles:
                raisons.append('scan obligatoire absent ou périmé')
            else:
                self.get_logger().warning(
                    'Aucun scan exploitable (TF ou lidar) : garde obstacle '
                    'désactivée.', throttle_duration_sec=5.0)
        # En mode manuel, une commande périmée n'arrête pas le robot : le
        # pilote peut commander directement sur /cmd_vel (teleop) et la garde
        # ne doit pas écraser cette commande par un zéro.
        if not commande_fraiche:
            if self._mode == 'course':
                raisons.append('commande absente ou périmée')
            else:
                # Mode manuel : ne rien publier, laisser teleop passer.
                return
        if not (math.isfinite(v) and math.isfinite(w)):
            raisons.append('commande non finie')
        if not raisons:
            v = max(-self._navigation.max_linear_speed, min(self._navigation.max_linear_speed, v))
            w = max(-self._navigation.max_angular_speed, min(self._navigation.max_angular_speed, w))
            points = [point for scan in scans_utilises for point in scan[1]]
            if not points or trajectoire_libre(
                    points, v, w, self._navigation.footprint,
                    reglage.marge_m + self._navigation.footprint_padding,
                    reglage.horizon_sec, reglage.pas_sec,
                    tolerance_penetration=reglage.penetration_tolerance_m):
                sortie.linear.x, sortie.angular.z = v, w
            else:
                raisons.append('obstacle sur la trajectoire')
        if (v or w) and raisons:
            self.get_logger().warning(
                'Commande arrêtée : ' + ', '.join(raisons) + '.',
                throttle_duration_sec=2.0)
            for source in self._sources.values():
                if not source.required and self._scans[source.topic] is None:
                    self.get_logger().warning(
                        f'Source optionnelle {source.name} absente ou périmée : '
                        'ignorée.', throttle_duration_sec=5.0)
        self._sortie.publish(sortie)


def main(args=None):
    rclpy.init(args=args)
    noeud = GardeVitesse(load_config())
    try:
        rclpy.spin(noeud)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            noeud._sortie.publish(Twist())
        noeud.destroy_node()
        rclpy.try_shutdown()
