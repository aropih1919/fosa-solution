"""Briques d'exécution réutilisables par la tâche et par l'assistant.

- `TaskTimer` : chronomètre officiel (horloge ROS, marge de sécurité) ;
- `BringupManager` : lancement et arrêt propre du bringup ROS ;
- `Nav2Navigator` : client Nav2 (but, retour d'état, annulation, costmaps).

Aucune valeur métier n'est codée ici : tout vient de `caytu_config`.
"""

import math
import os
import shutil
import signal
import subprocess
import tempfile
import time

from action_msgs.msg import GoalStatus
from caytu_config import ConfigurationError
from caytu_nav_solution.benchmark_config import (
    build_experiment_config,
    validate_experiment_config,
    write_experiment_config,
)
from caytu_nav_solution.nav_math import quaternion_depuis_lacet
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
import rclpy
from rclpy.action import ActionClient


class TaskTimer:
    """Chronomètre de la tâche, basé sur l'horloge ROS (simulée ou réelle)."""

    def __init__(self, now, time_limit_sec, stop_margin_sec):
        self._now = now
        self._time_limit_sec = float(time_limit_sec)
        self._stop_margin_sec = float(stop_margin_sec)
        self._start = None

    def start(self):
        """Démarre le chronomètre et retourne False si l'horloge n'est pas prête."""
        self._start = self._now()
        return self._start.nanoseconds != 0

    def elapsed(self):
        if self._start is None:
            return 0.0
        return (self._now() - self._start).nanoseconds * 1e-9

    def remaining(self):
        """Temps restant avant la limite officielle, marge déduite."""
        return self._time_limit_sec - self._stop_margin_sec - self.elapsed()

    def expired(self):
        return self.remaining() <= 0.0


def secteur_arriere_est_libre(scan, transformation, distance_requise,
                              demi_angle, echantillons_min, ecart_angulaire_max):
    """Accepte un recul uniquement si le scan couvre et dégage tout son secteur.

    Les rayons sont exprimés dans le repère du capteur ; la transformation vers
    `base_footprint` permet de vérifier le secteur arrière du robot, même si le
    lidar est décalé ou orienté différemment. Un rayon manquant ou invalide rend
    le résultat faux : l'absence d'observation n'est jamais assimilée au vide.
    """
    if (scan is None or len(scan.ranges) == 0 or scan.angle_increment <= 0.0
            or distance_requise <= 0.0 or echantillons_min < 1):
        return False

    translation = transformation.transform.translation
    rotation = transformation.transform.rotation
    decalage_capteur = math.hypot(translation.x, translation.y)
    portee_requise = distance_requise + decalage_capteur
    if scan.range_max < portee_requise:
        return False

    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    angles = []
    for index, mesure in enumerate(scan.ranges):
        angle_capteur = scan.angle_min + index * scan.angle_increment
        vx, vy = math.cos(angle_capteur), math.sin(angle_capteur)
        uv = (-z * vy, z * vx, x * vy - y * vx)
        uuv = (y * uv[2] - z * uv[1],
               z * uv[0] - x * uv[2],
               x * uv[1] - y * uv[0])
        direction_x = vx + 2.0 * (w * uv[0] + uuv[0])
        direction_y = vy + 2.0 * (w * uv[1] + uuv[1])
        angle_base = math.atan2(direction_y, direction_x)
        ecart = (angle_base - math.pi + math.pi) % (2.0 * math.pi) - math.pi
        if abs(ecart) > demi_angle:
            continue

        portee_requise_ray = portee_requise / math.cos(ecart)
        valeur = float(mesure)
        if math.isnan(valeur) or valeur < scan.range_min:
            return False
        if math.isinf(valeur):
            if valeur < 0.0 or scan.range_max < portee_requise_ray:
                return False
        elif valeur < portee_requise_ray or valeur > scan.range_max:
            return False
        angles.append(ecart)

    if len(angles) < echantillons_min:
        return False
    angles.sort()
    ecarts = [angles[0] + demi_angle]
    ecarts.extend(droite - gauche
                  for gauche, droite in zip(angles, angles[1:]))
    ecarts.append(demi_angle - angles[-1])
    return max(ecarts, default=float('inf')) <= ecart_angulaire_max


class BringupManager:
    """Lance `ros2 launch` dans un groupe de processus et l'arrête proprement."""

    def __init__(self, navigation, logger, popen=None, killpg=None,
                 getpgid=None):
        self._navigation = navigation
        self._logger = logger
        self._popen = popen or subprocess.Popen
        self._killpg = killpg or os.killpg
        self._getpgid = getpgid or os.getpgid
        self._process = None
        self._config_dir = None

    def start(self, *, map_yaml, use_sim_time, params_file, controller,
              localization, profile, task_params_path, spawn,
              bringup_package=None, bringup_launch=None):
        """Prépare le YAML d'essai puis lance le bringup complet."""
        params_file = str(params_file or '')
        if params_file:
            if not os.path.isfile(params_file):
                raise ConfigurationError(
                    f'nav2_params_file absent : {params_file}')
            import yaml
            with open(params_file, encoding='utf-8') as stream:
                configuration = yaml.safe_load(stream)
            if not isinstance(configuration, dict):
                raise ConfigurationError(
                    'nav2_params_file doit contenir un mapping YAML complet.')
            validate_experiment_config(
                configuration, controller, localization, profile)
        else:
            # Le laboratoire fusionne nav2_params + contrôleur + profil +
            # localisation dans un seul YAML, puis le passe au bringup.
            self._config_dir = tempfile.mkdtemp(prefix='fosa_nav_config_')
            params_file = os.path.join(self._config_dir, 'params.yaml')
            configuration = build_experiment_config(
                controller, localization, profile, spawn,
                use_sim_time=use_sim_time)
            write_experiment_config(configuration, params_file)

        # Liste de morceaux de commande, jamais une chaîne : aucun passage par
        # le shell, donc aucun caractère spécial interprété.
        command = [
            'ros2', 'launch',
            bringup_package or self._navigation.bringup_package,
            bringup_launch or self._navigation.bringup_launch,
            f'use_sim_time:={"true" if use_sim_time else "false"}',
            f'map:={map_yaml}',
            f'controller:={controller}',
            f'localization:={localization}',
            f'profile:={profile}',
            f'params_file:={params_file}',
            f'task_params_file:={task_params_path}',
            f'initial_pose_x:={spawn.x}',
            f'initial_pose_y:={spawn.y}',
            f'initial_pose_yaw:={spawn.yaw}',
        ]
        self._logger.info('Démarrage de la solution : ' + ' '.join(command))
        self._process = self._popen(command, start_new_session=True)

    def alive(self):
        """Indique si le bringup tourne ou n'a pas été démarré ici."""
        return self._process is None or self._process.poll() is None

    def stop(self):
        """Arrêt gradué SIGINT -> SIGTERM -> SIGKILL, puis nettoyage."""
        if self._process is not None and self._process.poll() is None:
            self._logger.info('Arrêt du bringup...')
            for signal_envoye, attente in zip(
                    (signal.SIGINT, signal.SIGTERM, signal.SIGKILL),
                    self._navigation.bringup_stop_timeouts):
                try:
                    self._killpg(
                        self._getpgid(self._process.pid), signal_envoye)
                except ProcessLookupError:
                    break
                try:
                    self._process.wait(timeout=attente)
                    break
                except subprocess.TimeoutExpired:
                    continue
        self.cleanup()

    def cleanup(self):
        """Supprime le YAML d'essai généré, s'il existe."""
        if self._config_dir is not None:
            shutil.rmtree(self._config_dir, ignore_errors=True)
            self._config_dir = None


def construire_but(goal_x, goal_y, frame, stamp, pose, spawn):
    """Message NavigateToPose orienté du robot vers le but."""
    reference = pose or spawn
    cap = math.atan2(goal_y - reference.y, goal_x - reference.x)

    but = PoseStamped()
    but.header.frame_id = frame
    but.header.stamp = stamp
    but.pose.position.x = goal_x
    but.pose.position.y = goal_y

    qx, qy, qz, qw = quaternion_depuis_lacet(cap)
    but.pose.orientation.x = qx
    but.pose.orientation.y = qy
    but.pose.orientation.z = qz
    but.pose.orientation.w = qw

    message = NavigateToPose.Goal()
    message.pose = but
    return message


def est_arrive(pose, goal_x, goal_y, rayon):
    """Vrai si le centre du robot est dans le rayon d'arrivée du but."""
    return pose is not None and pose.distance_vers(goal_x, goal_y) <= rayon


class Nav2Navigator:
    """Client Nav2 réutilisable : envoi de but, suivi, annulation, costmaps."""

    def __init__(self, node, navigation, timer, bringup, pose_robot,
                 spin_once=None, is_ok=None, success_radius=None,
                 arrival_recheck_radius=None):
        self._node = node
        self._navigation = navigation
        self._timer = timer
        self._bringup = bringup
        self._pose_robot = pose_robot
        # Les paramètres ROS peuvent surcharger les valeurs du YAML. Les
        # conserver explicitement évite de perdre ces overrides à l'extraction.
        self._success_radius = float(
            navigation.success_radius if success_radius is None
            else success_radius)
        self._arrival_recheck_radius = float(
            navigation.arrival_recheck_radius if arrival_recheck_radius is None
            else arrival_recheck_radius)
        self._spin = spin_once or (
            lambda timeout: rclpy.spin_once(node, timeout_sec=timeout))
        self._is_ok = is_ok or rclpy.ok

        self._client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self._clear_clients = [
            node.create_client(
                ClearEntireCostmap,
                navigation.global_costmap_clear_service),
            node.create_client(
                ClearEntireCostmap,
                navigation.local_costmap_clear_service),
        ]
        self._but_en_cours = None
        self._dernier_log_feedback = 0.0

    def _traiter(self, secondes):
        """Traite les messages ROS pendant une durée réelle donnée."""
        fin = time.monotonic() + secondes
        while self._is_ok():
            restant = fin - time.monotonic()
            if restant <= 0.0:
                break
            self._spin(min(0.05, restant))

    def server_is_ready(self):
        return self._client.server_is_ready()

    def attendre_pret(self, amcl_ready, localization, timeout_sec):
        """Attend que Nav2 accepte des buts et que le robot soit localisé."""
        debut = time.monotonic()
        dernier_log = debut

        while self._is_ok():
            self._traiter(0.2)
            serveur_pret = self.server_is_ready()
            robot_localise = (
                self._pose_robot() is not None
                and (localization != 'amcl' or amcl_ready()))

            if serveur_pret and robot_localise:
                self._node.get_logger().info(
                    f'Nav2 prêt et robot localisé après '
                    f'{time.monotonic() - debut:.1f} s réelles.')
                return True

            if not self._bringup.alive():
                self._node.get_logger().error(
                    "Le bringup s'est arrêté pendant le démarrage.")
                return False

            if (self._timer.remaining() <= 0.0
                    or time.monotonic() - debut > timeout_sec):
                self._node.get_logger().error(
                    "Nav2 n'est pas devenu disponible à temps.")
                return False

            if time.monotonic() - dernier_log > 5.0:
                dernier_log = time.monotonic()
                self._node.get_logger().info(
                    f"Attente de Nav2 (serveur d'action : "
                    f'{"oui" if serveur_pret else "non"}, '
                    f'localisation : {"oui" if robot_localise else "non"})...')

        return False

    def message_but(self, goal_x, goal_y, frame, spawn):
        return construire_but(
            goal_x, goal_y, frame,
            self._node.get_clock().now().to_msg(),
            self._pose_robot(), spawn)

    def est_arrive(self, goal_x, goal_y):
        return est_arrive(
            self._pose_robot(), goal_x, goal_y,
            self._success_radius)

    def recevoir_feedback(self, message_feedback):
        maintenant = time.monotonic()
        periode = self._navigation.feedback_log_period_sec
        if maintenant - self._dernier_log_feedback < periode:
            return
        self._dernier_log_feedback = maintenant

        distance_restante = getattr(
            message_feedback.feedback, 'distance_remaining', float('nan'))
        self._node.get_logger().info(
            f'En route : {distance_restante:.2f} m restants, '
            f'{self._timer.elapsed():.0f} s écoulées.')

    def vider_costmaps(self):
        for client in self._clear_clients:
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())

    def annuler_but(self):
        """Annule le but en cours et attend la temporisation configurée."""
        if not self._is_ok() or self._but_en_cours is None:
            return
        future_annulation = self._but_en_cours.cancel_goal_async()
        fin = time.monotonic() + self._navigation.cancel_timeout_sec
        while (self._is_ok() and not future_annulation.done()
               and time.monotonic() < fin):
            self._spin(0.05)
        self._but_en_cours = None

    def _attendre_future(self, future):
        """Traite les messages jusqu'à la fin de `future`.

        Retourne 'ok', 'temps_ecoule' ou 'arret'.
        """
        while self._is_ok() and not future.done():
            self._spin(0.05)
            if self._timer.remaining() <= 0.0:
                return 'temps_ecoule'
            if not self._bringup.alive():
                return 'arret'
        return 'ok'

    def naviguer_une_fois(self, goal_x, goal_y, frame, spawn):
        """Un envoi du but et l'interprétation de son résultat."""
        future_envoi = self._client.send_goal_async(
            self.message_but(goal_x, goal_y, frame, spawn),
            feedback_callback=self.recevoir_feedback)
        resultat = self._attendre_future(future_envoi)
        if resultat != 'ok':
            return resultat

        but = future_envoi.result() if future_envoi.done() else None
        if but is None or not but.accepted:
            return 'refuse'
        self._but_en_cours = but

        future_resultat = but.get_result_async()
        resultat = self._attendre_future(future_resultat)
        if resultat == 'temps_ecoule':
            self._node.get_logger().error(
                'Limite de temps atteinte : annulation du but.')
            self.annuler_but()
            return 'temps_ecoule'
        if resultat == 'arret':
            return 'arret'

        self._but_en_cours = None
        resultat_final = (future_resultat.result()
                          if future_resultat.done() else None)
        statut = (resultat_final.status if resultat_final is not None
                  else GoalStatus.STATUS_UNKNOWN)

        if statut == GoalStatus.STATUS_SUCCEEDED:
            pose = self._pose_robot()
            distance = (None if pose is None
                        else pose.distance_vers(goal_x, goal_y))
            limite = self._arrival_recheck_radius
            if distance is not None and distance > limite:
                self._node.get_logger().warn(
                    f"Nav2 annonce l'arrivée mais le robot est à "
                    f'{distance:.2f} m du but : nouvel envoi.')
                return 'reessayer'
            return 'succes'

        if self.est_arrive(goal_x, goal_y):
            return 'succes'

        self._node.get_logger().warn(
            f'Nav2 a interrompu la navigation (statut {statut}).')
        return 'reessayer'
