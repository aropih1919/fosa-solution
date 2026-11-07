"""Chronomètre et client Nav2 de la course.

- `ChronometreCourse` : budget sur l'horloge fournie, marge de sécurité ;
- `NavigateurNav2` : envoi de buts, rotation sur place, retour d'état,
  annulation, interruption et nettoyage des costmaps.

Aucune valeur métier n'est codée ici : tout vient de `caytu_config`.
"""

import math
import time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from caytu_nav_solution.nav_math import quaternion_depuis_lacet
from caytu_nav_solution.vie_nav2 import etats_inactifs, noeuds_attendus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose, Spin
from nav2_msgs.srv import ClearEntireCostmap
import rclpy
from rclpy.action import ActionClient


class ChronometreCourse:
    """Budget sur l'horloge fournie ; la course injecte le temps réel monotone."""

    def __init__(self, now, time_limit_sec, stop_margin_sec):
        self._now = now
        self._time_limit_sec = float(time_limit_sec)
        self._stop_margin_sec = float(stop_margin_sec)
        self._start = None

    def demarrer(self):
        """Démarre le chronomètre et retourne False si l'horloge n'est pas prête."""
        self._start = self._now()
        return self._start.nanoseconds != 0

    def ecoule(self):
        if self._start is None:
            return 0.0
        return (self._now() - self._start).nanoseconds * 1e-9

    def restant(self):
        """Temps restant avant la limite officielle, marge déduite."""
        return self._time_limit_sec - self._stop_margin_sec - self.ecoule()

    def expire(self):
        return self.restant() <= 0.0


def construire_but(goal_x, goal_y, frame, stamp, pose, spawn, cap=None):
    """Message NavigateToPose ; sans cap imposé, orienté du robot vers le but."""
    if cap is None:
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


class NavigateurNav2:
    """Client Nav2 réutilisable : envoi de but, suivi, annulation, costmaps."""

    def __init__(self, node, navigation, chronometre, pose_robot,
                  spin_once=None, is_ok=None, success_radius=None,
                  arrival_recheck_radius=None, interrompre=None, now=None):
        self._node = node
        self._navigation = navigation
        self._chronometre = chronometre
        self._pose_robot = pose_robot
        self._now = now or time.monotonic
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
        self._interrompre = interrompre or (lambda: False)

        self._client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self._client_rotation = ActionClient(node, Spin, 'spin')
        # Nav2 peut avoir ses serveurs d'action prêts alors que ses nœuds
        # lifecycle sont encore inactifs : vérifier l'état avant le premier but.
        self._lifecycle_clients = {
            nom: node.create_client(GetState, f'/{nom}/get_state')
            for nom in noeuds_attendus(navigation.mode)}
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

    def serveur_pret(self):
        return self._client.server_is_ready()

    def _etats_lifecycle(self):
        """États des nœuds Nav2, ou None si un service ne répond pas encore."""
        if not self._lifecycle_clients:
            return {}
        futures = {}
        for nom, client in self._lifecycle_clients.items():
            if not client.service_is_ready():
                return None
            futures[nom] = client.call_async(GetState.Request())
        fin = time.monotonic() + 0.5
        while (self._is_ok()
               and any(not future.done() for future in futures.values())
               and time.monotonic() < fin):
            self._spin(0.05)
        etats = {}
        for nom, future in futures.items():
            try:
                if not future.done() or future.result() is None:
                    return None
                etats[nom] = future.result().current_state.id
            except Exception:       # service disparu pendant l'appel
                return None
        return etats

    def attendre_pret(self, timeout_sec):
        """Attend que Nav2 accepte des buts et que le robot soit localisé."""
        debut = time.monotonic()
        dernier_log = debut

        while self._is_ok():
            self._traiter(0.2)
            serveur_pret = self.serveur_pret()
            rotation_prete = self._client_rotation.server_is_ready()
            robot_localise = self._pose_robot() is not None
            etats = self._etats_lifecycle()
            inactifs = (etats_inactifs(etats, self._navigation.mode)
                        if etats is not None else ())
            lifecycle_pret = etats is not None and not inactifs

            if (serveur_pret and rotation_prete and robot_localise
                    and lifecycle_pret):
                self._node.get_logger().info(
                    f'Nav2, lifecycle et rotation prêts, robot localisé après '
                    f'{time.monotonic() - debut:.1f} s réelles.')
                return True

            if (self._chronometre.restant() <= 0.0
                    or self._interrompre()
                    or time.monotonic() - debut > timeout_sec):
                self._node.get_logger().error(
                    "Nav2 n'est pas devenu disponible à temps.")
                if inactifs:
                    self._node.get_logger().error(
                        'Nœuds lifecycle non actifs : '
                        + ', '.join(inactifs) + '.')
                return False

            if time.monotonic() - dernier_log > 5.0:
                dernier_log = time.monotonic()
                details = (
                    f"Attente de Nav2 (serveur d'action : "
                    f'{"oui" if serveur_pret else "non"}, '
                    f'spin : {"oui" if rotation_prete else "non"}, '
                    f'localisation : {"oui" if robot_localise else "non"}, '
                    f'lifecycle : {"oui" if lifecycle_pret else "non"}')
                if inactifs:
                    details += f'; inactifs : {", ".join(inactifs)}'
                self._node.get_logger().info(details + ')...')

        return False

    def message_but(self, goal_x, goal_y, frame, spawn, cap=None):
        return construire_but(
            goal_x, goal_y, frame,
            self._node.get_clock().now().to_msg(),
            self._pose_robot(), spawn, cap)

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
            f'{self._chronometre.ecoule():.0f} s écoulées.')

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

    def _annuler_si_accepte(self, future_envoi):
        """But abandonné avant son acceptation : s'il est accepté juste après,
        on l'annule, sinon le robot repartirait (par exemple après la fin)."""
        fin = time.monotonic() + self._navigation.cancel_timeout_sec
        while self._is_ok() and not future_envoi.done() and time.monotonic() < fin:
            self._spin(0.05)
        poignee = future_envoi.result() if future_envoi.done() else None
        if poignee is not None and poignee.accepted:
            self._but_en_cours = poignee
            self.annuler_but()

    def _attendre_future(self, future, interrompre=None, surveiller=None):
        """Traite les messages jusqu'à la fin de `future`.

        Retourne 'ok', 'temps_ecoule' ou 'interrompu'.
        """
        while self._is_ok() and not future.done():
            self._spin(0.05)
            if self._chronometre.restant() <= 0.0:
                return 'temps_ecoule'
            if self._interrompre():
                return 'interrompu'
            if interrompre is not None and interrompre():
                return 'interrompu'
            if surveiller is not None:
                etat = surveiller()
                if etat:
                    return etat
        return 'ok'

    def naviguer_une_fois(self, goal_x, goal_y, frame, spawn, cap=None,
                          interrompre=None):
        """Un envoi du but et l'interprétation de son résultat.

        `cap` impose l'orientation finale ; `interrompre()` vrai annule le but
        en cours (cible trouvée pendant une recherche, budget épuisé...).
        """
        if not self._client.server_is_ready():
            return 'refuse'
        future_envoi = self._client.send_goal_async(
            self.message_but(goal_x, goal_y, frame, spawn, cap),
            feedback_callback=self.recevoir_feedback)
        resultat = self._attendre_future(future_envoi, interrompre)
        if resultat != 'ok':
            self._annuler_si_accepte(future_envoi)
            return resultat

        but = future_envoi.result() if future_envoi.done() else None
        if but is None or not but.accepted:
            return 'refuse'
        self._but_en_cours = but

        surveillance = SurveillanceBut(
            self._now, self._pose_robot,
            self._navigation.goal_stall_sec,
            self._navigation.goal_stall_distance_m,
            self._navigation.goal_stall_rotation_rad,
            self._navigation.goal_timeout_sec)
        future_resultat = but.get_result_async()
        resultat = self._attendre_future(
            future_resultat, interrompre, surveiller=surveillance.update)
        if resultat == 'temps_ecoule':
            self._node.get_logger().error(
                'Limite de temps atteinte : annulation du but.')
            self.annuler_but()
            return 'temps_ecoule'
        if resultat == 'interrompu':
            self.annuler_but()
            return 'interrompu'
        if resultat in ('immobile', 'duree'):
            self._node.get_logger().warn(
                'But Nav2 bloqué (robot immobile ou délai dépassé) : '
                'annulation et nettoyage des costmaps.')
            self.vider_costmaps()
            self.annuler_but()
            return 'bloque'

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

    def tourner(self, angle, delai_sec, interrompre=None):
        """Rotation sur place par le behavior Spin de Nav2 (collisions vérifiées).

        Retourne 'succes', 'echec', 'interrompu', 'temps_ecoule'
        ou 'indisponible'.
        """
        fin = time.monotonic() + 2.0
        while not self._client_rotation.server_is_ready():
            if time.monotonic() > fin or not self._is_ok():
                return 'indisponible'
            self._spin(0.05)

        but = Spin.Goal()
        but.target_yaw = float(angle)
        but.time_allowance = Duration(
            sec=int(delai_sec), nanosec=int((delai_sec % 1.0) * 1e9))
        future_envoi = self._client_rotation.send_goal_async(but)
        resultat = self._attendre_future(future_envoi, interrompre)
        if resultat != 'ok':
            self._annuler_si_accepte(future_envoi)
            return resultat
        poignee = future_envoi.result() if future_envoi.done() else None
        if poignee is None or not poignee.accepted:
            return 'echec'

        self._but_en_cours = poignee
        surveillance = SurveillanceBut(
            self._now, self._pose_robot,
            self._navigation.goal_stall_sec,
            10.0,  # le spin ne doit pas être jugé sur la translation
            self._navigation.goal_stall_rotation_rad,
            delai_sec + self._navigation.cancel_timeout_sec)
        future_resultat = poignee.get_result_async()
        resultat = self._attendre_future(
            future_resultat, interrompre, surveiller=surveillance.update)
        if resultat == 'temps_ecoule':
            self.annuler_but()
            return 'temps_ecoule'
        if resultat == 'interrompu':
            self.annuler_but()
            return 'interrompu'
        if resultat in ('immobile', 'duree'):
            self._node.get_logger().warn(
                'Rotation Nav2 bloquée : annulation et nettoyage des costmaps.')
            self.vider_costmaps()
            self.annuler_but()
            return 'bloque'
        self._but_en_cours = None
        final = future_resultat.result() if future_resultat.done() else None
        statut = final.status if final is not None else GoalStatus.STATUS_UNKNOWN
        return 'succes' if statut == GoalStatus.STATUS_SUCCEEDED else 'echec'


class SurveillanceBut:
    """Détecte un but Nav2 qui n'agit plus : immobile ou délai dépassé.

    Pur et sans ROS : la fonction d'horloge et la pose du robot sont injectées,
    ce qui permet de le tester sans nœud ROS.
    """

    def __init__(self, now, pose_robot, duree_immobilite_sec,
                 distance_m, rotation_rad, duree_max_sec):
        self._now = now
        self._pose_robot = pose_robot
        self._duree_immobilite_sec = float(duree_immobilite_sec)
        self._distance_m = float(distance_m)
        self._rotation_rad = float(rotation_rad)
        self._duree_max_sec = float(duree_max_sec)
        self._depart = now()
        self._reference = pose_robot()
        self._dernier_mouvement = self._depart

    def _a_bouge(self, pose):
        if pose is None or self._reference is None:
            return False
        distance = math.hypot(
            pose.x - self._reference.x, pose.y - self._reference.y)
        rotation = abs(math.atan2(
            math.sin(pose.yaw - self._reference.yaw),
            math.cos(pose.yaw - self._reference.yaw)))
        return (distance >= self._distance_m
                or rotation >= self._rotation_rad)

    def update(self):
        """Retourne 'immobile', 'duree' ou None."""
        maintenant = self._now()
        pose = self._pose_robot()
        # Pas de localisation : on ne conclut pas à l'immobilité (le robot
        # peut bouger sans que la pose soit disponible). On réarme le compteur.
        if pose is None:
            self._dernier_mouvement = maintenant
            if maintenant - self._depart >= self._duree_max_sec:
                return 'duree'
            return None
        if self._a_bouge(pose):
            self._reference = pose
            self._dernier_mouvement = maintenant
        if maintenant - self._depart >= self._duree_max_sec:
            return 'duree'
        if maintenant - self._dernier_mouvement >= self._duree_immobilite_sec:
            return 'immobile'
        return None
