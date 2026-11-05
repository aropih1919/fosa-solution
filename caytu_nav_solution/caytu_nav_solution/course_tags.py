#!/usr/bin/env python3
"""Course de la phase finale PARC 2026 : visiter les tags dans l'ordre publié.

Lancé par course.launch.py, avec la navigation, le détecteur de tags, le
registre des tags et le démarrage vocal. Déroulement (guide officiel) :
  1. attend que Nav2 et la localisation soient prêts ;
  2. lit l'ordre des tags sur /parc/tag_order (arbitre PARC ; type détecté seul) ;
  3. attend le départ : mot « allez » (nœud mot_depart) ou commande
     `ros2 run caytu_nav_solution depart`. Rien ne bouge avant. L'arbitre est
     prévenu (/parc/start), avec le mot entendu pour le bonus vocal ;
  4. tour sur place de repérage si un tag de l'ordre n'est pas encore connu ;
  5. pour chaque tag : Nav2 jusqu'à une pose d'attente à ~1 m, face au tag,
     puis approche finale guidée par la détection du tag jusqu'à 0,5 m,
     aligné à quelques degrés ; arrêt, visite signalée à l'arbitre
     (/parc/tag_visit : identifiant + image JPEG), maintien ; recul ; un tag
     introuvable à temps est sauté ;
  6. signale la fin (/parc/finish si un tag a été sauté), ou s'arrête à la
     limite de 10 minutes, ou dès que l'arbitre clôt la course (/parc/run_status).
Toutes les valeurs viennent de caytu_config (section course).

Essais : -p ordre:=7,1,9,4 remplace le topic des organisateurs (interdit en
course : l'ordre ne doit jamais être saisi à la main) ; -p depart_immediat:=true
part sans attendre le signal de départ.
"""

import math
import signal
import sys

from action_msgs.msg import GoalStatus
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from caytu_config import load_config
from caytu_interfaces.msg import TagDetectionArray
from caytu_nav_solution.arbitre_parc import ArbitreParc

ETAT_TIMED_OUT = 4      # parc_msgs/RunStatus.TIMED_OUT
from caytu_nav_solution.course_logique import (
    choisir_point_exploration, commande_alignement, commande_approche, distance_libre,
    ecarts_visite, extraire_ordre, points_exploration, pose_devant_tag)
from caytu_nav_solution.nav_math import (
    Pose2D, lacet_depuis_quaternion, normaliser_angle, quaternion_depuis_lacet)
from caytu_nav_solution.statut_course import PublicateurStatutCourse

# Demi-largeur du couloir surveillé par le garde-fou lidar (robot : ±0,25 m).
DEMI_LARGEUR_COULOIR = 0.27
REGLAGE_MAX = 6.0       # s de réglage final devant le tag avant de mesurer quand même


def _pose_finie(pose):
    """Faux si une valeur de la pose est NaN ou infinie."""
    p, q = pose.position, pose.orientation
    return all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))


class FinDeCourse(Exception):
    """Limite de temps atteinte, ou course close par l'arbitre : on arrête tout."""

    def __init__(self, raison='temps_ecoule'):
        super().__init__(raison)
        self.raison = raison


class CourseTags(Node):

    def __init__(self, configuration=None):
        super().__init__('course_tags')
        self._configuration = configuration or load_config()
        self._course = self._configuration.course
        systeme = self._configuration.system

        self.declare_parameter('ordre', '')
        self.declare_parameter('depart_immediat', False)
        self.declare_parameter('cmd_vel_topic', self._configuration.navigation.cmd_vel_topic)
        self.declare_parameter('scan_topic', '/scan_clean')
        self.declare_parameter('image_topic', self._configuration.perception.top_color_topic)
        self.declare_parameter('order_topic', self._course.order_topic)
        self._repere_global = systeme.global_frame
        self._repere_robot = systeme.base_frame

        # État
        self._ordre = []
        self._tags_connus = {}          # id -> (x, y, z, lacet, confiance)
        self._derniere_vue = {}         # id -> (x, y, lacet, instant monotone)
        self._depart_recu = False
        self._mot_depart = ''           # mot entendu si départ vocal (bonus)
        self._derniere_image = None
        self._points_lidar = []         # points (x, y) dans base_footprint
        self._lidar_vers_robot = None   # (x, y, cap) du lidar dans base_footprint
        self._carte = None
        self._endroits_balayes = []
        self._debut = None
        self._abonnement_ordre = None
        self._vus_depuis_depart = set()
        self._vues_course = {}          # id -> vues de la course (x, y, lacet, t, distance)

        # ROS
        self._tf = Buffer()
        self._ecouteur_tf = TransformListener(self._tf, self, spin_thread=False)
        self._vitesse = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # Cycle de vie de Nav2 : vérifier qu'il est actif, le relancer sinon.
        self._nav_actif = self.create_client(
            Trigger, '/lifecycle_manager_navigation/is_active')
        self._nav_gestion = self.create_client(
            ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
        self._derniere_relance = float('-inf')
        self._statut = PublicateurStatutCourse(self, self._course)
        self._arbitre = ArbitreParc(self, self._course.referee)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TagDetectionArray, self._course.tags_topic,
                                 self.recevoir_tags_connus, latched)
        self.create_subscription(TagDetectionArray, '/fosa/tags_vus',
                                 self.recevoir_tags_vus, 10)
        # Accusé de réception du départ, verrouillé : `depart` et `mot_depart`
        # répètent leur signal tant qu'ils ne l'ont pas vu.
        self._accuse_depart = self.create_publisher(
            String, self._course.start_ack_topic, latched)
        self.create_subscription(Empty, self._course.start_topic, self.recevoir_depart, 10)
        self._etat_garde_fou = None
        self.create_subscription(CollisionMonitorState, self._course.safety_state_topic,
                                 self.recevoir_etat_garde_fou, 10)
        self._fin_arbitre_ignoree = False
        self.create_subscription(String, self._course.voice.start_topic,
                                 self.recevoir_depart_vocal, 10)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                 self.recevoir_scan, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self.recevoir_carte, latched)
        topic_image = self.get_parameter('image_topic').value
        self.create_subscription(
            CompressedImage if topic_image.endswith('/compressed') else Image,
            topic_image, self.recevoir_image, qos_profile_sensor_data)

        ordre_force = str(self.get_parameter('ordre').value or '').strip()
        if ordre_force:
            self._ordre = extraire_ordre(ordre_force, self._course.expected_ids)
            self.get_logger().warn(
                f'ESSAI : ordre imposé par paramètre {self._ordre} (interdit en course).')
        self._minuteur_ordre = self.create_timer(1.0, self.chercher_topic_ordre)

    # ================================================================ entrées
    def chercher_topic_ordre(self):
        """S'abonne au topic de l'ordre dès qu'il existe, avec son type et sa QoS."""
        if self._abonnement_ordre is not None or self._ordre:
            self._minuteur_ordre.cancel()
            return
        topic = self.get_parameter('order_topic').value
        types = dict(self.get_topic_names_and_types()).get(topic)
        if not types:
            return
        try:
            from rosidl_runtime_py.utilities import get_message
            classe = get_message(types[0])
        except (AttributeError, ModuleNotFoundError, ValueError) as erreur:
            self.get_logger().error(f'Type {types[0]} de {topic} inconnu ici : {erreur}')
            return
        qos = QoSProfile(depth=10)
        publicateurs = self.get_publishers_info_by_topic(topic)
        if publicateurs:
            qos.reliability = publicateurs[0].qos_profile.reliability
            qos.durability = publicateurs[0].qos_profile.durability
        self._abonnement_ordre = self.create_subscription(
            classe, topic, self.recevoir_ordre, qos)
        self.get_logger().info(f'Ordre des tags attendu sur {topic} ({types[0]}).')

    def recevoir_ordre(self, message):
        # parc_msgs/TagOrder : lire tag_ids seulement (l'horodatage contient
        # aussi des entiers). Liste vide tant qu'aucune course n'est armée.
        source = list(message.tag_ids) if hasattr(message, 'tag_ids') else message
        ordre = extraire_ordre(source, self._course.expected_ids)
        if ordre and ordre != self._ordre:
            if self._debut is not None and self._ordre:
                self.get_logger().warn(f'Nouvel ordre {ordre} ignoré : course en cours.')
                return
            self._ordre = ordre
            self.get_logger().info(f'Ordre des tags reçu : {ordre}')

    def recevoir_depart(self, _message):
        if not self._depart_recu:
            self.get_logger().info('Signal de départ reçu (commande).')
            self._accuse_depart.publish(String(data='commande'))
        self._depart_recu = True

    def recevoir_depart_vocal(self, message):
        if not self._depart_recu:
            self._mot_depart = (message.data or '').strip() or 'allez'
            self.get_logger().info(f'Signal de départ reçu (voix : « {self._mot_depart} »).')
            self._accuse_depart.publish(String(data=f'voix:{self._mot_depart}'))
        self._depart_recu = True

    def recevoir_etat_garde_fou(self, message):
        """Journal : pourquoi nav2_collision_monitor arrête ou ralentit le robot."""
        etat = (int(message.action_type), message.polygon_name)
        if etat == self._etat_garde_fou:
            return
        self._etat_garde_fou = etat
        if etat[0] == CollisionMonitorState.STOP:
            if etat[1] == 'invalid source':
                self.get_logger().error(
                    'Garde-fou : ARRÊT, une source n\'a pas de données fraîches '
                    '(lidar /scan_clean ou caméra). Vérifier `ros2 topic hz` ; simulateur '
                    'lent : garde_fou_cameras:=false.')
            else:
                self.get_logger().warn(f'Garde-fou : ARRÊT (obstacle dans {etat[1]}).')
        elif etat[0] == CollisionMonitorState.SLOWDOWN:
            self.get_logger().info(f'Garde-fou : ralenti ({etat[1]}).')
        else:
            self.get_logger().info('Garde-fou : libre.')

    def recevoir_tags_connus(self, message):
        for detection in message.detections:
            pose = detection.pose_map.pose
            q = pose.orientation
            if not _pose_finie(pose):
                continue
            self._tags_connus[int(detection.tag_id)] = (
                pose.position.x, pose.position.y, pose.position.z,
                lacet_depuis_quaternion(q.x, q.y, q.z, q.w), float(detection.confidence))

    def recevoir_tags_vus(self, message):
        maintenant = self._secondes()
        for detection in message.detections:
            pose = detection.pose_map.pose
            if not _pose_finie(pose):
                continue
            identifiant = int(detection.tag_id)
            self._vus_depuis_depart.add(identifiant)
            q = pose.orientation
            vue = (pose.position.x, pose.position.y,
                   lacet_depuis_quaternion(q.x, q.y, q.z, q.w), maintenant)
            self._derniere_vue[identifiant] = vue
            vues = self._vues_course.setdefault(identifiant, [])
            vues.append(vue + (float(detection.distance_m),))
            del vues[:-60]

    def recevoir_image(self, message):
        self._derniere_image = message

    def recevoir_carte(self, message):
        self._carte = message

    def recevoir_scan(self, message):
        if self._lidar_vers_robot is None:
            try:
                tf = self._tf.lookup_transform(self._repere_robot, message.header.frame_id, Time())
            except TransformException:
                return
            q = tf.transform.rotation
            self._lidar_vers_robot = (tf.transform.translation.x, tf.transform.translation.y,
                                      lacet_depuis_quaternion(q.x, q.y, q.z, q.w))
        lx, ly, lcap = self._lidar_vers_robot
        cos_c, sin_c = math.cos(lcap), math.sin(lcap)
        points = []
        angle = message.angle_min
        for distance in message.ranges:
            if message.range_min < distance < min(message.range_max, 2.0):
                px, py = distance * math.cos(angle), distance * math.sin(angle)
                points.append((lx + cos_c * px - sin_c * py, ly + sin_c * px + cos_c * py))
            angle += message.angle_increment
        self._points_lidar = points

    # ============================================================ utilitaires
    def traiter(self, secondes):
        fin = self._secondes() + secondes
        while rclpy.ok():
            restant = fin - self._secondes()
            if restant <= 0.0:
                break
            rclpy.spin_once(self, timeout_sec=min(0.05, restant))

    def _secondes(self):
        """Horloge de toutes les attentes de la course : celle du nœud.

        Temps simulé avec use_sim_time : un simulateur plus lent que le temps
        réel ne raccourcit pas les délais (recherche d'un tag, navigation,
        immobilité devant le tag). Sur le vrai robot, c'est le temps réel.
        """
        return self.get_clock().now().nanoseconds * 1e-9

    def temps_ecoule(self):
        if self._debut is None:
            return 0.0
        return (self.get_clock().now() - self._debut).nanoseconds * 1e-9

    def temps_restant(self):
        return (self._course.time_limit_sec - self._course.stop_margin_sec
                - self.temps_ecoule())

    def verifier_temps(self):
        if self._debut is not None and self.temps_restant() <= 0.0:
            raise FinDeCourse('temps_ecoule')
        if self._arbitre.course_terminee():
            # Le chronomètre de l'arbitre compte en temps RÉEL. Dans un simulateur
            # plus lent que le temps réel, ses 10 minutes ne sont que quelques
            # dizaines de secondes simulées : on continue (notre limite, en temps
            # simulé, s'applique). Sur le vrai robot, TIMED_OUT arrête la course.
            if self._arbitre.etat == ETAT_TIMED_OUT and self._temps_simule():
                if not self._fin_arbitre_ignoree:
                    self._fin_arbitre_ignoree = True
                    self.get_logger().warn(
                        'Arbitre : TIMED_OUT (chronomètre en temps réel) ignoré en temps '
                        'simulé. Pour les essais : referee time_limit_s:=20000.0.')
                return
            raise FinDeCourse(f'arbitre_{self._arbitre.nom_etat().lower()}')

    def _temps_simule(self):
        return bool(self.get_parameter('use_sim_time').value)

    def pose_robot(self):
        try:
            tf = self._tf.lookup_transform(self._repere_global, self._repere_robot, Time())
        except TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return Pose2D(t.x, t.y, lacet_depuis_quaternion(q.x, q.y, q.z, q.w))

    def commander(self, lineaire=0.0, angulaire=0.0):
        commande = Twist()
        commande.linear.x = float(lineaire)
        commande.angular.z = float(angulaire)
        self._vitesse.publish(commande)

    def arreter(self, repetitions=5):
        for _ in range(repetitions):
            self.commander()
            rclpy.spin_once(self, timeout_sec=0.02)

    def tag_fiable(self, identifiant):
        tag = self._tags_connus.get(identifiant)
        return tag is not None and tag[4] >= 1.0

    def geometrie_tag(self, identifiant):
        """(x, y, lacet, frais) du tag : vue récente, sinon médiane des vues de
        la course (les proches d'abord), sinon la position notée (indice)."""
        vue = self._derniere_vue.get(identifiant)
        if vue is not None and self._secondes() - vue[3] < self._course.fresh_detection_sec:
            return vue[0], vue[1], vue[2], True
        vues = self._vues_course.get(identifiant) or []
        if vues:
            proches = [v for v in vues if v[4] <= 2.5]
            choisies = proches if len(proches) >= 2 else vues
            xs = sorted(v[0] for v in choisies)
            ys = sorted(v[1] for v in choisies)
            lacet = math.atan2(sum(math.sin(v[2]) for v in choisies),
                               sum(math.cos(v[2]) for v in choisies))
            return xs[len(xs) // 2], ys[len(ys) // 2], lacet, False
        tag = self._tags_connus.get(identifiant)
        if tag is None:
            return None
        return tag[0], tag[1], tag[3], False

    # ================================================================== Nav2
    def _appeler(self, client, requete, delai):
        if not client.wait_for_service(timeout_sec=1.0):
            return None
        futur = client.call_async(requete)
        fin = self._secondes() + delai
        while rclpy.ok() and not futur.done() and self._secondes() < fin:
            rclpy.spin_once(self, timeout_sec=0.05)
        return futur.result() if futur.done() else None

    def nav2_actif(self):
        reponse = self._appeler(self._nav_actif, Trigger.Request(), 2.0)
        return reponse is not None and reponse.success

    def relancer_nav2(self):
        """Nav2 n'a pas démarré (ex. une costmap n'a pas eu la TF à temps) :
        on demande au gestionnaire de cycle de vie de tout redémarrer."""
        if self._secondes() - self._derniere_relance < 30.0:
            return
        self._derniere_relance = self._secondes()
        self.get_logger().warn('Nav2 inactif : redémarrage de ses nœuds.')
        for commande in (ManageLifecycleNodes.Request.RESET,
                         ManageLifecycleNodes.Request.STARTUP):
            requete = ManageLifecycleNodes.Request()
            requete.command = commande
            self._appeler(self._nav_gestion, requete, 30.0)

    def naviguer_vers(self, cible, delai_max=90.0, nouvel_essai=True):
        """Nav2 jusqu'à `cible` (Pose2D). Renvoie True si le but est atteint."""
        if not self._nav.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 n'accepte pas de but.")
            return False
        but = NavigateToPose.Goal()
        but.pose = PoseStamped()
        but.pose.header.frame_id = self._repere_global
        but.pose.header.stamp = self.get_clock().now().to_msg()
        but.pose.pose.position.x, but.pose.pose.position.y = cible.x, cible.y
        (but.pose.pose.orientation.x, but.pose.pose.orientation.y,
         but.pose.pose.orientation.z, but.pose.pose.orientation.w) = quaternion_depuis_lacet(
            cible.yaw)
        envoi = self._nav.send_goal_async(but)
        while rclpy.ok() and not envoi.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        poignee = envoi.result()
        if poignee is None or not poignee.accepted:
            # But refusé : Nav2 est peut-être inactif. On le relance une fois.
            if nouvel_essai and not self.nav2_actif():
                self.relancer_nav2()
                return self.naviguer_vers(cible, delai_max, nouvel_essai=False)
            return False
        resultat = poignee.get_result_async()
        debut = self._secondes()
        try:
            while rclpy.ok() and not resultat.done():
                rclpy.spin_once(self, timeout_sec=0.05)
                self.verifier_temps()
                if self._secondes() - debut > delai_max:
                    self.get_logger().warn('Navigation trop longue : abandon de ce but.')
                    poignee.cancel_goal_async()
                    self.traiter(0.5)
                    return False
        except FinDeCourse:
            poignee.cancel_goal_async()
            self.traiter(0.3)
            raise
        return resultat.result().status == GoalStatus.STATUS_SUCCEEDED

    # ============================================================= mouvements
    def tourner_sur_place(self, angle_total, arret_si=None):
        """Tour sur place ; s'arrête plus tôt si arret_si() devient vrai."""
        vitesse = self._course.scan_angular_speed
        pose = self.pose_robot()
        if pose is None:
            return
        cumul, cap_precedent = 0.0, pose.yaw
        debut = self._secondes()
        limite = abs(angle_total) / vitesse * 2.0 + 5.0
        while rclpy.ok() and cumul < abs(angle_total):
            self.verifier_temps()
            if arret_si is not None and arret_si():
                break
            if self._secondes() - debut > limite:
                break
            self.commander(0.0, math.copysign(vitesse, angle_total))
            self.traiter(0.05)
            pose = self.pose_robot()
            if pose is not None:
                cumul += abs(normaliser_angle(pose.yaw - cap_precedent))
                cap_precedent = pose.yaw
        self.arreter()
        if pose is not None and cumul >= 2.0 * math.pi - 0.2:
            self._endroits_balayes.append((pose.x, pose.y))

    def reculer(self, distance):
        if distance <= 0.0:
            return
        depart = self.pose_robot()
        if depart is None:
            return
        debut = self._secondes()
        while rclpy.ok() and self._secondes() - debut < distance / 0.08 + 3.0:
            pose = self.pose_robot()
            if pose is None or math.hypot(pose.x - depart.x, pose.y - depart.y) >= distance:
                break
            if distance_libre(self._points_lidar, DEMI_LARGEUR_COULOIR, avant=False) < (
                    self._course.front_clearance_m):
                self.get_logger().warn('Obstacle derrière : recul interrompu.')
                break
            self.commander(-0.08, 0.0)
            self.traiter(0.05)
        self.arreter()

    # ========================================================= recherche
    def tag_vu_en_course(self, identifiant):
        """Vrai si la caméra a vu ce tag depuis le départ.

        Les positions notées pendant les essais ne sont que des indices : la
        carte a pu être corrigée après coup, et elles peuvent être décalées
        de plusieurs dizaines de centimètres.
        """
        return (identifiant in self._vus_depuis_depart
                and self.geometrie_tag(identifiant) is not None)

    def vue_fraiche(self, identifiant):
        geometrie = self.geometrie_tag(identifiant)
        return geometrie is not None and geometrie[3]

    def chercher_autour(self, identifiant):
        """Cherche le tag depuis ici : vers sa position, ±50°, puis un tour."""
        def frais():
            return self.vue_fraiche(identifiant)
        self.traiter(1.0)
        if frais():
            return True
        robot = self.pose_robot()
        geometrie = self.geometrie_tag(identifiant)
        if robot is not None and geometrie is not None:
            vers_tag = normaliser_angle(
                math.atan2(geometrie[1] - robot.y, geometrie[0] - robot.x) - robot.yaw)
            if abs(vers_tag) > math.radians(10.0):
                self.tourner_sur_place(vers_tag, frais)
            self.traiter(1.0)
        for balayage in (math.radians(50), -math.radians(100), math.radians(50)):
            if frais():
                return True
            self.tourner_sur_place(balayage, frais)
            self.traiter(0.5)
        if not frais():
            self.tourner_sur_place(2.0 * math.pi, frais)
            self.traiter(0.5)
        return frais()

    def explorer_un_pas(self, identifiant):
        """Un pas de recherche : tour sur place ici, sinon vers un point neuf."""
        pose = self.pose_robot()
        if pose is None:
            self.traiter(0.5)
            return
        def trouve():
            return self.tag_vu_en_course(identifiant)
        ici_deja_vu = any(math.hypot(pose.x - x, pose.y - y) < 1.0
                          for x, y in self._endroits_balayes)
        if not ici_deja_vu:
            self.get_logger().info(f'Recherche du tag {identifiant} : tour sur place.')
            self.tourner_sur_place(2.0 * math.pi, trouve)
            return
        point = None
        if self._carte is not None:
            info = self._carte.info
            points = points_exploration(
                self._carte.data, info.width, info.height, info.resolution,
                info.origin.position.x, info.origin.position.y)
            point = choisir_point_exploration(points, pose, self._endroits_balayes)
        if point is None:
            self.get_logger().info(f'Recherche du tag {identifiant} : plus de point neuf, '
                                   'nouveau tour sur place.')
            self._endroits_balayes.clear()
            self.tourner_sur_place(2.0 * math.pi, trouve)
            return
        self.get_logger().info(
            f'Recherche du tag {identifiant} : exploration vers ({point[0]:.2f}, {point[1]:.2f}).')
        cap = math.atan2(point[1] - pose.y, point[0] - pose.x)
        if not self.naviguer_vers(Pose2D(point[0], point[1], cap), delai_max=45.0):
            self._endroits_balayes.append(point)       # inaccessible : on n'y revient pas
            return
        if not trouve():
            self.tourner_sur_place(2.0 * math.pi, trouve)

    # ============================================================== visite
    def approche_finale(self, identifiant):
        """Rejoint la pose à 0,5 m face au tag.

        Renvoie (réussi, écarts) ; écarts vaut None si le tag n'a pas été revu
        à l'arrivée : aucune visite n'est alors déclarée.
        """
        course = self._course
        tolerance_cap = math.radians(min(course.yaw_tolerance_deg, 12.0) * 0.75)
        debut = self._secondes()
        derniers_ecarts = None
        verifications = 0
        # Cible lissée : chaque vue du tag bouge de 1-2 cm et 1-3° ; sans
        # lissage, le robot hésite sans fin entre avancer et tourner.
        lisse = None
        derniere_vue_prise = None
        # Réglage final verrouillé : à moins de rayon_final de la pose visée, le
        # robot ne se retourne plus vers le point (il oscillait sur place, sans
        # fin, avec 3 cm et 3° de tolérance). Il fait face au tag puis règle la
        # distance en ligne droite ; au-delà de REGLAGE_MAX s, il mesure quand même.
        rayon_final = max(2.0 * course.distance_tolerance_m, 0.06)
        phase_finale = False
        debut_finale = None
        while rclpy.ok() and self._secondes() - debut < 45.0:
            self.verifier_temps()
            robot = self.pose_robot()
            geometrie = self.geometrie_tag(identifiant)
            if robot is None or geometrie is None:
                self.arreter(1)
                self.traiter(0.1)
                continue
            vue = self._derniere_vue.get(identifiant)
            if lisse is None:
                lisse = list(geometrie[:3])
            elif geometrie[3] and vue is not None and vue[3] != derniere_vue_prise:
                derniere_vue_prise = vue[3]
                lisse[0] += 0.3 * (geometrie[0] - lisse[0])
                lisse[1] += 0.3 * (geometrie[1] - lisse[1])
                lisse[2] = normaliser_angle(
                    lisse[2] + 0.3 * normaliser_angle(geometrie[2] - lisse[2]))
            tag_x, tag_y, lacet = lisse
            cible = pose_devant_tag(tag_x, tag_y, lacet, course.visit_distance_m)
            ecart_cible = math.hypot(cible.x - robot.x, cible.y - robot.y)
            if phase_finale and ecart_cible > 3.0 * rayon_final:
                phase_finale = False          # la vue a beaucoup corrigé le tag : on reprend
            if not phase_finale and ecart_cible <= rayon_final:
                phase_finale, debut_finale = True, self._secondes()
                self.get_logger().info(
                    f'Tag {identifiant} : réglage final (à {ecart_cible * 100:.0f} cm de la pose).')
            if phase_finale:
                lineaire, angulaire, termine = commande_alignement(
                    robot, tag_x, tag_y, lacet, course.visit_distance_m,
                    0.5 * course.approach_linear_speed, course.approach_angular_speed,
                    course.distance_tolerance_m * 0.6, tolerance_cap)
                if not termine and self._secondes() - debut_finale > REGLAGE_MAX:
                    termine = True            # le réglage traîne : on mesure et on signale
            else:
                lineaire, angulaire, termine = commande_approche(
                    robot, cible, course.approach_linear_speed, course.approach_angular_speed,
                    tolerance_position=course.distance_tolerance_m * 0.6,
                    tolerance_cap=tolerance_cap)
            if lineaire > 0.0 and distance_libre(
                    self._points_lidar, DEMI_LARGEUR_COULOIR) < course.front_clearance_m:
                self.get_logger().warn('Obstacle devant le pare-chocs : approche en pause.',
                                       throttle_duration_sec=2.0)
                lineaire = 0.0
            if lineaire < 0.0 and distance_libre(
                    self._points_lidar, DEMI_LARGEUR_COULOIR, avant=False) < (
                    course.front_clearance_m):
                lineaire = 0.0
            if not termine:
                self.commander(lineaire, angulaire)
                self.traiter(0.1)
                continue
            # Arrivé : on laisse le robot se poser, puis on mesure sur une vue
            # fraîche du tag (jamais sur une position notée).
            self.arreter()
            self.traiter(0.6)
            attente_vue = self._secondes() + 1.5
            while rclpy.ok() and not self.vue_fraiche(identifiant) and (
                    self._secondes() < attente_vue):
                self.traiter(0.1)
            robot = self.pose_robot()
            geometrie = self.geometrie_tag(identifiant)
            verifications += 1
            debut_finale = self._secondes()   # nouveau délai de réglage après chaque mesure
            if robot is None or geometrie is None or not geometrie[3]:
                if verifications >= 4:
                    break
                continue
            ecart_distance, ecart_angle, lateral = ecarts_visite(
                robot, geometrie[0], geometrie[1], geometrie[2], course.visit_distance_m)
            derniers_ecarts = (ecart_distance, ecart_angle, lateral)
            if (abs(ecart_distance) <= course.distance_tolerance_m
                    and abs(ecart_angle) <= course.yaw_tolerance_deg):
                return True, derniers_ecarts
            if verifications >= 4:
                break
        self.arreter()
        if derniers_ecarts is None:
            derniers_ecarts = self._mesure_de_secours(identifiant)
        return False, derniers_ecarts

    def _mesure_de_secours(self, identifiant):
        """Délai écoulé sans mesure : une vue fraîche suffit si le robot est
        vraiment devant le tag (à 15 cm et 12° près, règle officielle : ±12°)."""
        course = self._course
        self.traiter(0.6)
        attente_vue = self._secondes() + 1.5
        while rclpy.ok() and not self.vue_fraiche(identifiant) and (
                self._secondes() < attente_vue):
            self.traiter(0.1)
        robot = self.pose_robot()
        geometrie = self.geometrie_tag(identifiant)
        if robot is None or geometrie is None or not geometrie[3]:
            return None
        ecarts = ecarts_visite(robot, geometrie[0], geometrie[1], geometrie[2],
                               course.visit_distance_m)
        if abs(ecarts[0]) <= 0.15 and abs(ecarts[1]) <= 12.0:
            self.get_logger().warn(f'Tag {identifiant} : réglage fin non abouti, visite '
                                   'signalée sur la dernière mesure.')
            return ecarts
        return None

    def aller_pose_attente(self, identifiant):
        """Nav2 jusqu'à une pose à ~1 m face au tag (plusieurs distances, et
        normale tournée de ±20° si la pose est dans un obstacle)."""
        course = self._course
        for distance, rotation in ((course.staging_distance_m, 0.0),
                                   (course.staging_distance_m - 0.2, 0.0),
                                   (course.staging_distance_m + 0.25, 0.0),
                                   (course.staging_distance_m, math.radians(20.0)),
                                   (course.staging_distance_m, -math.radians(20.0))):
            geometrie = self.geometrie_tag(identifiant)
            attente = pose_devant_tag(geometrie[0], geometrie[1], geometrie[2] + rotation,
                                      distance)
            self.get_logger().info(
                f'Tag {identifiant} : pose d\'attente ({attente.x:.2f}, {attente.y:.2f}) '
                f'à {distance:.2f} m.')
            if self.naviguer_vers(attente):
                return True
            self.verifier_temps()
        robot = self.pose_robot()
        geometrie = self.geometrie_tag(identifiant)
        return robot is not None and math.hypot(
            robot.x - geometrie[0], robot.y - geometrie[1]) < 1.8

    def se_rapprocher(self, identifiant):
        """Va voir le tag de plus près, du côté du robot (sa position ou sa
        normale peuvent être fausses s'il n'a été vu que de loin)."""
        robot = self.pose_robot()
        geometrie = self.geometrie_tag(identifiant)
        if robot is None or geometrie is None:
            return False
        direction = math.atan2(robot.y - geometrie[1], robot.x - geometrie[0])
        for distance in (1.6, 2.2, 1.2):
            for decalage in (0.0, 0.5, -0.5):
                angle = direction + decalage
                point = Pose2D(geometrie[0] + distance * math.cos(angle),
                               geometrie[1] + distance * math.sin(angle),
                               normaliser_angle(angle + math.pi))
                self.get_logger().info(
                    f'Tag {identifiant} : approche pour le voir de plus près '
                    f'({point.x:.2f}, {point.y:.2f}).')
                if self.naviguer_vers(point, delai_max=45.0):
                    self.chercher_autour(identifiant)
                    return True
                self.verifier_temps()
        return False

    def visiter(self, identifiant, tags_suivants=0):
        """Visite d'un tag : 'visite', 'saute' (ou FinDeCourse).

        Budget de recherche : au moins search_timeout_sec, et tout le temps qui
        reste une fois réservé ~70 s par tag suivant (un tag sauté ne peut
        plus rapporter de points : autant chercher tant que c'est utile).
        """
        course = self._course
        budget = max(course.search_timeout_sec,
                     self.temps_restant() - 70.0 * tags_suivants)
        echeance = self._secondes() + budget
        indice_essaye = False
        tentatives_approche = 0
        while True:
            self.verifier_temps()
            if self._secondes() > echeance or tentatives_approche >= 3:
                self._statut.saute(identifiant, 'introuvable', self.temps_ecoule())
                return 'saute'

            # 1. Trouver le tag : position notée d'abord, puis exploration.
            if not self.tag_vu_en_course(identifiant):
                indice = self.geometrie_tag(identifiant)
                if indice is not None and not indice_essaye:
                    indice_essaye = True
                    self.get_logger().info(
                        f'Tag {identifiant} pas encore revu : vers sa position notée.')
                    attente = pose_devant_tag(indice[0], indice[1], indice[2],
                                              course.staging_distance_m)
                    self.naviguer_vers(attente, delai_max=60.0)
                    self.chercher_autour(identifiant)
                else:
                    self.explorer_un_pas(identifiant)
                continue

            # 2. Pose d'attente à ~1 m, face au tag, puis vue fraîche.
            if not self.aller_pose_attente(identifiant):
                self.get_logger().warn(f'Tag {identifiant} : pose d\'attente inaccessible.')
                tentatives_approche += 1
                if not self.se_rapprocher(identifiant):
                    self._vus_depuis_depart.discard(identifiant)   # repartir en recherche
                    self.explorer_un_pas(identifiant)
                continue
            self.get_logger().info(
                f'Tag {identifiant} : pose d\'attente atteinte ({self.temps_ecoule():.0f} s).')
            if not self.chercher_autour(identifiant):
                self.get_logger().warn(f'Tag {identifiant} pas revu depuis la pose '
                                       "d'attente : sa position était fausse, nouvelle recherche.")
                tentatives_approche += 1
                # Ces vues (lointaines) trompaient : on les oublie et on explore.
                self._vus_depuis_depart.discard(identifiant)
                self._vues_course.pop(identifiant, None)
                indice_essaye = True
                continue
            geometrie = self.geometrie_tag(identifiant)
            attente = pose_devant_tag(geometrie[0], geometrie[1], geometrie[2],
                                      course.staging_distance_m)
            robot = self.pose_robot()
            if robot is not None and math.hypot(attente.x - robot.x, attente.y - robot.y) > 0.5:
                # La vue fraîche corrige beaucoup la position : on se replace.
                # En deçà, l'approche finale rattrape l'écart elle-même.
                self.naviguer_vers(attente)
                self.chercher_autour(identifiant)
            self.get_logger().info(
                f'Tag {identifiant} : approche finale ({self.temps_ecoule():.0f} s).')

            # 3. Approche finale guidée par le tag.
            reussi, ecarts = self.approche_finale(identifiant)
            self.arreter()
            if ecarts is None:
                self.get_logger().warn(f'Tag {identifiant} pas revu à l\'arrivée : '
                                       'pas de visite déclarée.')
                tentatives_approche += 1
                self.reculer(course.backoff_m)
                continue
            ecart_distance, ecart_angle, lateral = ecarts
            self.get_logger().info(
                f'Tag {identifiant} : écart de distance {ecart_distance * 100:+.1f} cm, '
                f'alignement {ecart_angle:+.1f}°, latéral {lateral * 100:+.1f} cm'
                + ('' if reussi else ' — hors tolérance interne, visite signalée quand même'))

            # 4. Visite : identifiant + image, puis immobile pour les juges.
            self._statut.visite(identifiant, self._derniere_image, self.temps_ecoule(),
                                ecart_distance, ecart_angle)
            self._arbitre.signaler_visite(identifiant, self._derniere_image)
            # Arbitre : au moins 3 s sans bouger après le signalement.
            fin_maintien = self._secondes() + max(course.hold_sec, 3.5)
            while rclpy.ok() and self._secondes() < fin_maintien:
                self.verifier_temps()
                self.commander()
                self.traiter(0.1)
            self.reculer(course.backoff_m)
            return 'visite'

    # =============================================================== course
    def attendre_pret(self, delai=300.0):
        debut = self._secondes()
        while rclpy.ok() and self._secondes() - debut < delai:
            self.traiter(0.5)
            if self.pose_robot() is not None and self._nav.server_is_ready():
                if self.nav2_actif():
                    self.get_logger().info('Navigation prête et robot localisé.')
                    return True
                if self._secondes() - debut > 25.0:
                    self.relancer_nav2()
            self.get_logger().info('Attente de Nav2 et de la localisation...',
                                   throttle_duration_sec=10.0)
        return False

    def attendre_depart(self):
        if self.get_parameter('depart_immediat').value:
            self.get_logger().warn('ESSAI : départ immédiat, sans signal.')
            return
        self.get_logger().info(
            f'Prêt. Départ sur {self._course.start_topic} (mot de départ ou '
            '`ros2 run caytu_nav_solution depart`). Le robot reste immobile.')
        while rclpy.ok() and not self._depart_recu:
            self.traiter(0.2)
            connus = sorted(i for i in self._course.expected_ids if self.tag_fiable(i))
            self.get_logger().info(
                f'En attente du départ. Ordre : {self._ordre or "pas encore reçu"} ; '
                f'tags connus : {connus}.', throttle_duration_sec=15.0)

    def executer(self):
        if not self.attendre_pret():
            self.get_logger().error('Navigation indisponible : course impossible.')
            return 1
        self.attendre_depart()
        self._debut = self.get_clock().now()
        self.arreter()
        self._arbitre.signaler_depart(self._mot_depart)

        # L'ordre arrive normalement avant le départ ; sinon on l'attend un peu.
        echeance = self._secondes() + 30.0
        while rclpy.ok() and not self._ordre and self._secondes() < echeance:
            self.traiter(0.2)
        if not self._ordre:
            self._ordre = list(self._course.expected_ids)
            self.get_logger().error(
                f"Ordre jamais reçu sur {self.get_parameter('order_topic').value} : "
                f'ordre par défaut {self._ordre}.')
        ordre = list(self._ordre)
        self._statut.depart(ordre, self.temps_ecoule())

        raison = 'terminee'
        sautes = []
        try:
            # Tour de repérage, toujours : les positions notées pendant les
            # essais peuvent être décalées (corrections de la carte après
            # coup) ; on revoit les tags depuis la localisation de la course.
            self.get_logger().info('Tour de repérage des tags.')
            self._vus_depuis_depart = set()
            self._vues_course = {}
            self.tourner_sur_place(
                2.0 * math.pi, lambda: all(i in self._vus_depuis_depart for i in ordre))
            for rang, identifiant in enumerate(ordre):
                resultat = self.visiter(identifiant, tags_suivants=len(ordre) - rang - 1)
                if resultat != 'visite':
                    sautes.append(identifiant)
                self.get_logger().info(
                    f'Tag {identifiant} : {resultat} ({self.temps_ecoule():.0f} s).')
        except FinDeCourse as fin:
            raison = fin.raison
            self.get_logger().warn(
                'Limite de temps : arrêt de la course.' if raison == 'temps_ecoule'
                else f'Course close par l\'arbitre ({self._arbitre.nom_etat()}) : arrêt.')
        finally:
            self.arreter(10)
        if sautes:
            self.get_logger().warn(f'Tags sautés : {sautes}.')
        if raison == 'terminee':
            # Tous les tags signalés dans l'ordre : l'arbitre a déjà fini seul.
            # Sinon (tag sauté), on le prévient que le robot a terminé.
            self._arbitre.signaler_fin(abandon=False)
        self._statut.fin(raison, self.temps_ecoule())
        self.traiter(1.0)
        return 0


def _interrompre(_signum, _frame):
    raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _interrompre)
    signal.signal(signal.SIGTERM, _interrompre)
    noeud = None
    code = 1
    try:
        noeud = CourseTags()
        code = noeud.executer()
    except KeyboardInterrupt:
        pass
    finally:
        if noeud is not None:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            try:
                noeud.arreter(10)
            except Exception:  # noqa: BLE001 — l'arrêt du robot passe avant tout
                pass
            noeud.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
