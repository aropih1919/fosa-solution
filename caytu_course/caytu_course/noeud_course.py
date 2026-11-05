#!/usr/bin/env python3
"""Nœud de course de la phase réelle PARC 2026.

Déroulement (guide officiel) :
  1. attendre l'ordre des tags publié par les organisateurs ;
  2. attendre le départ (commande `./run.sh depart` ou mot « allez ») : aucun
     mouvement avant ;
  3. pour chaque tag, dans l'ordre : chercher (tours sur place, points de la
     carte SLAM), approcher avec Nav2, s'aligner à 0,5 m face au tag, signaler
     la visite avec une image, garder la position ; la stratégie
     (`strategie.py`) replanifie en continu et fait sauter un tag quand le
     tenter coûterait plus de points qu'il n'en rapporte (jamais revisité) ;
  4. signaler la fin et s'arrêter, au plus tard à 10 minutes.
"""

import json
import math
import signal
import sys
import time
from pathlib import Path

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from caytu_config import ConfigurationError, load_config
from caytu_course.alignement import (
    ReglagesAlignement,
    commande_alignement,
    vitesse_sure,
)
from caytu_course.entrees_course import EntreesCourse
from caytu_course.geometrie_tags import erreurs_visite, pose_approche, visite_valide
from caytu_course.memoire_tags import MemoireTags
from caytu_course.politique_blocage import doit_conserver_cible
from caytu_course.recherche import points_recherche
from caytu_course.sequence import SequenceTags
from caytu_course.strategie import ModeleStrategie, TagRestant, decider
from caytu_course.statut import PublicateurArbitre, qos_statut
from caytu_nav_solution.nav_math import Pose2D, lacet_depuis_quaternion
from caytu_nav_solution.navigation_runtime import (
    ChronometreCourse,
    NavigateurNav2,
)

PERIODE_BOUCLE = 0.1


class NoeudCourse(Node):

    def __init__(self, configuration):
        super().__init__('course_parc')
        self._configuration = configuration
        self._course = configuration.course
        self._navigation = configuration.navigation
        self._etat = None
        self._en_attente = True
        self._tours = []           # positions où un tour sur place a été fait
        self._points_essayes = []  # points de recherche déjà tentés
        self._echecs = {}          # visites ratées par tag
        self._debut_cible = {}     # instant où chaque tag est devenu la cible
        self._plan = None          # dernier plan de la stratégie
        self._plan_instant = None
        self._garder = True
        self._modele_strategie = self._modele()

        self._memoire_tf = Buffer()
        self._ecouteur_tf = TransformListener(
            self._memoire_tf, self, spin_thread=True)
        self._memoire = self._charger_memoire()
        self._entrees = EntreesCourse(
            self, configuration, self._memoire, self._memoire_tf,
            lambda: self._en_attente)
        self._statut = PublicateurArbitre(self, self._course.referee)
        self._publicateur_etat = self.create_publisher(
            String, self._course.state_topic, qos_statut(1))
        self._publicateur_vitesse = self.create_publisher(
            Twist, self._navigation.cmd_vel_topic, 10)

        self._chrono = ChronometreCourse(
            self.get_clock().now, self._course.time_limit_sec,
            self._course.stop_margin_sec)
        self._navigateur = NavigateurNav2(
            self, self._navigation, self._chrono, self.pose_robot)
        self._reglages = self._reglages_alignement()
        self._sequence = None

    # -------------------------------------------------------------- outils
    def _charger_memoire(self):
        distance_max = self._configuration.perception.apriltag.max_distance_m
        fichier = self._course.memorized_tags_file
        if not fichier:
            return MemoireTags(distance_max)
        chemin = Path(fichier).expanduser()
        try:
            donnees = yaml.safe_load(chemin.read_text(encoding='utf-8')) or {}
            memoire = MemoireTags.importer(donnees, distance_max)
        except (OSError, yaml.YAMLError, KeyError, TypeError, ValueError) as erreur:
            self.get_logger().error(
                f'Poses de tags mémorisées illisibles ({chemin}) : {erreur}')
            return MemoireTags(distance_max)
        self.get_logger().info(f'Tags mémorisés chargés : {memoire.ids()}.')
        return memoire

    def _reglages_alignement(self):
        course, alignement = self._course, self._course.alignment
        return ReglagesAlignement(
            tolerance_cap=math.radians(course.heading_tolerance_deg),
            gain_lineaire=alignement.linear_gain,
            gain_angulaire=alignement.angular_gain,
            max_lineaire=alignement.max_linear_mps,
            max_angulaire=alignement.max_angular_rps,
            min_lineaire=alignement.min_linear_mps,
            min_angulaire=alignement.min_angular_rps,
            precision_point=alignement.precision_point_m,
            precision_cap=math.radians(alignement.precision_heading_deg),
            erreur_distance_max=alignement.max_distance_error_m,
            erreur_laterale_max=alignement.max_lateral_error_m)

    def publier_etat(self, etat, cible=None, detail=''):
        if (etat, cible, detail) == self._etat:
            return
        self._etat = (etat, cible, detail)
        self._publicateur_etat.publish(String(data=json.dumps({
            'etat': etat, 'cible': cible, 'detail': detail,
            'temps_ecoule': round(self._chrono.ecoule(), 1),
        }, ensure_ascii=False)))
        self.get_logger().info(
            f'[{etat}]' + (f' tag {cible}' if cible is not None else '')
            + (f' : {detail}' if detail else ''))

    def pose_robot(self):
        try:
            tf = self._memoire_tf.lookup_transform(
                self._configuration.system.global_frame,
                self._configuration.system.base_frame, Time())
        except TransformException:
            return None
        q = tf.transform.rotation
        return Pose2D(tf.transform.translation.x, tf.transform.translation.y,
                      lacet_depuis_quaternion(q.x, q.y, q.z, q.w))

    def traiter_pendant(self, secondes):
        fin = time.monotonic() + secondes
        while rclpy.ok():
            restant = fin - time.monotonic()
            if restant <= 0.0:
                break
            rclpy.spin_once(self, timeout_sec=min(0.05, restant))

    def publier_vitesse(self, lineaire, angulaire):
        commande = Twist()
        commande.linear.x = float(lineaire)
        commande.angular.z = float(angulaire)
        self._publicateur_vitesse.publish(commande)

    def arreter_robot(self, repetitions=3):
        for _ in range(repetitions):
            self.publier_vitesse(0.0, 0.0)
            self.traiter_pendant(0.02)

    def _observer_blocage(self, tag_id):
        """Reste immobile un temps borné avant de renvoyer la même cible."""
        duree = self._course.blocked_observation_sec
        fin = time.monotonic() + duree
        self.arreter_robot()
        self.publier_etat(
            'OBSERVATION_OBSTACLE', tag_id,
            f'immobile pendant {duree:.1f} s avant reprise de la cible')
        while rclpy.ok() and not self._temps_ecoule():
            restant = fin - time.monotonic()
            if restant <= 0.0:
                return True
            self.publier_vitesse(0.0, 0.0)
            self.traiter_pendant(min(PERIODE_BOUCLE, restant))
        return False

    def _temps_ecoule(self):
        return self._chrono.expire()

    def _temps(self):
        """Secondes depuis le départ, sur l'horloge ROS (comme le chrono)."""
        return self._chrono.ecoule()

    # ------------------------------------------------------------- course
    def executer(self):
        self._attendre_ordre_et_depart()
        if not rclpy.ok():
            return
        self._en_attente = False
        self._chrono.demarrer()
        self.publier_etat('DEMARRAGE', detail=self._entrees.source_depart)
        # Le départ est signalé à l'arbitre dès qu'il est donné, même avant
        # l'ordre : son chronomètre démarre à ce signal. Un départ publié
        # directement sur /parc/start est déjà connu de l'arbitre.
        if not self._entrees.depart_de_l_arbitre:
            self._statut.signaler_depart(
                self._entrees.source_depart == 'voix', self._entrees.mot_entendu)
        self._attendre_ordre_apres_depart()
        if self._entrees.ordre is None:
            self._terminer('ordre_absent')
            return
        self._sequence = SequenceTags(self._entrees.ordre)
        self._attendre_nav2()

        while rclpy.ok() and not self._sequence.terminee():
            if self._temps_ecoule():
                break
            cible = self._sequence.cible()
            try:
                resultat = self._traiter_tag(cible)
            except KeyboardInterrupt:
                raise
            except Exception as erreur:  # la stratégie borne les reprises
                self.get_logger().error(f'Erreur pendant le tag {cible} : {erreur!r}')
                self._navigateur.annuler_but()
                self.arreter_robot()
                self.traiter_pendant(1.0)
                continue
            if resultat == 'visite':
                self._sequence.marquer_visite(cible)
            elif resultat == 'saute':
                self.get_logger().warning(
                    f'Tag {cible} sauté : le plan {list(self._plan.tags)} rapporte plus.')
                self._sequence.sauter()
            self._plan_instant = None   # nouvelle cible : replanifier
        self._terminer('terminee' if self._sequence.terminee() else 'temps_ecoule')

    def _attendre_ordre_et_depart(self):
        while rclpy.ok() and self._entrees.source_depart is None:
            etat = 'ATTENTE_ORDRE' if self._entrees.ordre is None else 'ATTENTE_DEPART'
            self.publier_etat(etat)
            rclpy.spin_once(self, timeout_sec=PERIODE_BOUCLE)

    def _attendre_ordre_apres_depart(self):
        """Départ par commande avant l'ordre : on attend l'ordre, chrono lancé."""
        while rclpy.ok() and self._entrees.ordre is None and not self._temps_ecoule():
            self.publier_etat('ATTENTE_ORDRE', detail='départ déjà donné')
            rclpy.spin_once(self, timeout_sec=PERIODE_BOUCLE)

    def _attendre_nav2(self):
        while rclpy.ok() and not self._temps_ecoule():
            self.publier_etat('DEMARRAGE', detail='attente de Nav2 et de la carte')
            if self._navigateur.attendre_pret(
                    self._course.nav2_ready_timeout_sec):
                return
            self.get_logger().error('Nav2 ou la localisation SLAM ne répond pas.')

    def _traiter_tag(self, tag_id):
        self._debut_cible.setdefault(tag_id, self._temps())

        def abandonner():
            return not self._garder_cible(tag_id)

        while rclpy.ok():
            if self._temps_ecoule():
                return 'temps_ecoule'
            if abandonner():
                return 'saute'
            if self._memoire.connait(tag_id):
                resultat = self._visiter(tag_id, abandonner)
                if resultat in ('visite', 'temps_ecoule'):
                    return resultat
                if resultat == 'perdu':
                    self.get_logger().warning(
                        f'Tag {tag_id} introuvable à sa pose mémorisée : recherche.')
                    self._memoire.oublier(tag_id)
                elif resultat == 'echec':
                    self._echecs[tag_id] = self._echecs.get(tag_id, 0) + 1
                    self._plan_instant = None   # replanifier tout de suite
                    self.traiter_pendant(0.5)
            else:
                self._chercher(tag_id, abandonner)
        return 'temps_ecoule'

    # ---------------------------------------------------------- stratégie
    def _modele(self):
        strategie = self._course.strategy
        return ModeleStrategie(
            points_par_tag=strategie.points_per_tag,
            vitesse_mps=strategie.travel_speed_mps,
            facteur_detour=strategie.detour_factor,
            temps_visite_s=strategie.visit_time_sec,
            temps_recherche_s=strategie.search_time_sec,
            trajet_inconnu_m=strategie.unknown_leg_m,
            marge=strategie.time_margin,
            reussite_connu=strategie.success_known,
            reussite_inconnu=strategie.success_unknown,
            baisse_par_echec=strategie.failure_decay,
            gain_min_saut=strategie.min_skip_gain_points)

    def _tags_restants(self):
        """Tags restants avec ce que la course en sait (point d'approche, échecs)."""
        cible = self._sequence.cible()
        restants = []
        for tag_id in self._sequence.restants():
            pose = self._memoire.pose(tag_id)
            position = None
            if pose is not None:
                x, y, _ = pose_approche(
                    pose.x, pose.y, pose.cap, self._course.approach_distance_m)
                position = (x, y)
            passe = (self._temps() - self._debut_cible[tag_id]
                     if tag_id == cible and tag_id in self._debut_cible else 0.0)
            restants.append(TagRestant(
                tag_id, position, self._echecs.get(tag_id, 0), passe))
        return tuple(restants)

    def _garder_cible(self, tag_id):
        """Replanifie au plus toutes les `replan_period_sec` (horizon glissant)."""
        maintenant = self._temps()
        periode = self._course.strategy.replan_period_sec
        if self._plan_instant is None or maintenant - self._plan_instant >= periode:
            pose = self.pose_robot()
            position = (pose.x, pose.y) if pose is not None else None
            garder_strategie, plan = decider(
                position, self._tags_restants(), self._chrono.restant(),
                self._modele_strategie)
            echecs = self._echecs.get(tag_id, 0)
            garder = doit_conserver_cible(
                garder_strategie, echecs, self._course.same_target_retries)
            if garder and not garder_strategie:
                self.get_logger().warning(
                    f'La cible {tag_id} reste prioritaire après {echecs} échec(s) ; '
                    f'reprises configurées : {self._course.same_target_retries}.')
            if self._plan is None or plan.tags != self._plan.tags:
                self.get_logger().info(
                    f'Plan : {list(plan.tags)} (durée estimée {plan.duree_s:.0f} s, '
                    f'{self._chrono.restant():.0f} s restantes, '
                    f'{plan.valeur:.0f} points espérés).')
            self._plan, self._plan_instant, self._garder = plan, maintenant, garder
        return self._garder

    # ---------------------------------------------------------- recherche
    def _interruption_recherche(self, tag_id, abandonner):
        return lambda: (self._memoire.connait(tag_id)
                        or abandonner() or self._temps_ecoule())

    def _chercher(self, tag_id, abandonner):
        """Un pas de recherche : tour sur place, sinon aller au point suivant."""
        interrompre = self._interruption_recherche(tag_id, abandonner)
        pose = self.pose_robot()
        if pose is None:
            self.publier_etat('RECHERCHE', tag_id, 'pose inconnue')
            self.traiter_pendant(0.5)
            return
        rayon = self._course.search_visited_radius_m
        if all(math.dist((pose.x, pose.y), tour) > rayon for tour in self._tours):
            self.publier_etat('RECHERCHE', tag_id, 'tour sur place')
            resultat = self._navigateur.tourner(
                2.0 * math.pi, self._course.search_spin_time_sec, interrompre)
            if resultat != 'interrompu':
                self._tours.append((pose.x, pose.y))
            if resultat in ('indisponible', 'echec'):
                self.get_logger().warning(f'Tour sur place impossible ({resultat}).')
            return

        point = self._prochain_point(pose)
        if point is None:
            # Toute la carte a été parcourue : nouveau passage.
            self.publier_etat('RECHERCHE', tag_id, 'nouveau passage')
            self._tours.clear()
            self._points_essayes.clear()
            self.traiter_pendant(0.5)
            return
        self.publier_etat('RECHERCHE', tag_id, f'vers ({point[0]:.2f}, {point[1]:.2f})')
        self._points_essayes.append(point)
        resultat = self._navigateur.naviguer_une_fois(
            point[0], point[1], self._configuration.system.global_frame,
            Pose2D(0.0, 0.0, 0.0), interrompre=interrompre)
        if resultat == 'refuse':
            self.traiter_pendant(0.5)

    def _prochain_point(self, pose):
        grille = self._entrees.grille()
        if grille is None:
            return None
        # Le repère map part du point de départ : la zone autorisée est un
        # cercle autour de l'origine.
        points = points_recherche(
            grille, self._course.search_clearance_m, self._course.search_grid_step_m,
            self._tours + self._points_essayes, self._course.search_visited_radius_m,
            (pose.x, pose.y), centre=(0.0, 0.0),
            rayon_max=self._course.search_max_radius_m)
        return points[0] if points else None

    # ------------------------------------------------------------- visite
    def _but_pour_centre(self, x, y, cap):
        """Pose de base_link quand le centre du robot est en (x, y)."""
        decalage = self._navigation.center_offset_x
        return x - decalage * math.cos(cap), y - decalage * math.sin(cap)

    def _pose_tag_courante(self, tag_id):
        """(x, y, cap de la normale, distance) du tag dans map, ou None.

        Si le tag est vu en ce moment, sa pose dans le repère du robot est
        composée avec la pose actuelle du robot : pas d'accumulation d'erreurs
        de TF ni de corrections SLAM passées. Sinon, la mémoire.
        """
        vue = self._entrees.detection_recente(
            tag_id, self._course.detection_max_age_sec)
        robot = self.pose_robot()
        if vue is not None and robot is not None:
            cos_cap, sin_cap = math.cos(robot.yaw), math.sin(robot.yaw)
            return (robot.x + cos_cap * vue.x - sin_cap * vue.y,
                    robot.y + sin_cap * vue.x + cos_cap * vue.y,
                    robot.yaw + vue.cap, vue.distance)
        memoire = self._memoire.pose(tag_id)
        if memoire is None:
            return None
        return memoire.x, memoire.y, memoire.cap, memoire.distance_min

    def _visiter(self, tag_id, abandonner):
        """Approche Nav2 puis alignement visuel ; retourne visite/perdu/echec/temps_ecoule.

        Nav2 ne vise jamais le point de visite lui-même : il s'arrête à
        `approach_distance_m`, l'asservissement visuel fait le reste.
        """
        course = self._course
        renavigations = 0
        forcer_nav2 = False
        while rclpy.ok() and renavigations <= course.alignment.max_renavigations:
            if self._temps_ecoule():
                return 'temps_ecoule'
            if abandonner():
                return 'echec'
            tag = self._pose_tag_courante(tag_id)
            if tag is None:
                return 'perdu'
            portee = course.approach_distance_m + course.preapproach_margin_m
            if forcer_nav2 or tag[3] > portee:
                resultat = self._approcher(tag_id, tag, abandonner)
                if resultat in ('temps_ecoule', 'perdu'):
                    return resultat
                if resultat == 'echec':
                    return 'echec'
                if resultat == 'reessayer':
                    renavigations += 1
                    continue
            # Dernier essai : on accepte un écart latéral double plutôt que de
            # perdre la visite (le cap reste aligné sur la normale du tag).
            dernier_essai = renavigations >= course.alignment.max_renavigations
            tolerance_laterale = course.lateral_tolerance_m * (2.0 if dernier_essai else 1.0)
            resultat = self._aligner(tag_id, abandonner, tolerance_laterale)
            if resultat == 'aligne':
                return self._valider_visite(tag_id)
            if resultat in ('perdu', 'temps_ecoule'):
                return resultat
            renavigations += 1
            forcer_nav2 = resultat == 'renaviguer'
        return 'echec'

    def _approcher(self, tag_id, tag, abandonner):
        """Nav2 jusqu'au point de pré-approche, face au tag.

        Retourne 'arrive', 'reessayer', 'perdu', 'echec' ou 'temps_ecoule'.
        """
        course = self._course
        tag_x, tag_y, tag_cap, _ = tag
        x, y, cap = pose_approche(tag_x, tag_y, tag_cap, course.approach_distance_m)
        x, y = self._but_pour_centre(x, y, cap)
        self.publier_etat('APPROCHE', tag_id, f'{course.approach_distance_m:.2f} m du tag')
        resultat = self._navigateur.naviguer_une_fois(
            x, y, self._configuration.system.global_frame, Pose2D(0.0, 0.0, 0.0),
            cap=cap, interrompre=abandonner)
        if resultat == 'temps_ecoule':
            return resultat
        if resultat == 'interrompu':
            return 'echec'
        if resultat == 'refuse':
            if not self._observer_blocage(tag_id):
                return 'temps_ecoule'
            return 'reessayer'
        courante = self._pose_tag_courante(tag_id)
        if courante is not None and courante[3] <= (
                course.approach_distance_m + course.preapproach_margin_m):
            return 'arrive'   # même si Nav2 a abandonné : le tag est à portée
        if resultat == 'reessayer':
            if not self._observer_blocage(tag_id):
                return 'temps_ecoule'
            return 'reessayer'
        # Arrivé mais tag pas revu de près : la pose mémorisée est fausse.
        return 'perdu'

    def _aligner(self, tag_id, abandonner, tolerance_laterale):
        course = self._course
        x_avant = max(x for x, _ in self._navigation.footprint)
        x_arriere = min(x for x, _ in self._navigation.footprint)
        demi_largeur = max(abs(y) for _, y in self._navigation.footprint)
        debut = dernier_vu = dernier_journal = self._temps()
        stables = 0
        au_point = False
        erreurs = None
        self.publier_etat('ALIGNEMENT', tag_id)
        while rclpy.ok():
            maintenant = self._temps()
            if self._temps_ecoule():
                self.arreter_robot()
                return 'temps_ecoule'
            if abandonner() or maintenant - debut > course.alignment.timeout_sec:
                self.arreter_robot()
                return ('aligne' if self._position_acceptable(erreurs, tolerance_laterale)
                        else 'echec')
            vue = self._entrees.detection_recente(tag_id, course.detection_max_age_sec)
            if vue is None:
                # Une pose mémorisée n'autorise jamais un pivot à l'aveugle.
                self.publier_vitesse(0.0, 0.0)
                if maintenant - dernier_vu > course.tag_lost_timeout_sec:
                    self.arreter_robot()
                    self.get_logger().warning(
                        f'Tag {tag_id} invisible depuis {course.tag_lost_timeout_sec:.0f} s '
                        'pendant l\'alignement.')
                    return 'perdu'
                self.traiter_pendant(PERIODE_BOUCLE)
                continue
            dernier_vu = maintenant
            erreurs = erreurs_visite(
                vue.x, vue.y, vue.cap, self._navigation.center_offset_x,
                course.visit_distance_m)
            commande = commande_alignement(erreurs, self._reglages, au_point)
            au_point = commande.au_point
            if commande.etat == 'renaviguer' or maintenant - dernier_journal >= 1.0:
                dernier_journal = maintenant
                self.get_logger().info(
                    f'Alignement tag {tag_id} : distance {erreurs.distance:+.3f} m, '
                    f'latéral {erreurs.laterale:+.3f} m, cap '
                    f'{math.degrees(erreurs.cap):+.1f}° -> {commande.etat} '
                    f'(v={commande.lineaire:+.2f}, w={commande.angulaire:+.2f})')
            if commande.etat == 'renaviguer':
                self.arreter_robot()
                return 'renaviguer'
            if commande.lineaire < 0.0 and not course.alignment.allow_reverse:
                self.arreter_robot()
                self.get_logger().warning(
                    'Recul de l’alignement désactivé : retour à une approche Nav2 sûre.')
                return 'renaviguer'
            if commande.etat == 'aligne':
                self.publier_vitesse(0.0, 0.0)
                stables += 1
                if stables >= course.stable_cycles:
                    return 'aligne'
            else:
                stables = 0
                self.publier_vitesse(*self._vitesse_gardee(
                    commande, x_avant, x_arriere, demi_largeur))
            self.traiter_pendant(PERIODE_BOUCLE)
        return 'echec'

    def _vitesse_gardee(self, commande, x_avant, x_arriere, demi_largeur):
        """Bloque tout mouvement si le scan est inconnu ou le trajet occupé."""
        lineaire = commande.lineaire
        if lineaire < 0.0 and not self._course.alignment.allow_reverse:
            self.get_logger().warning(
                'Recul désactivé par configuration : commande ignorée.',
                throttle_duration_sec=2.0)
            return 0.0, 0.0
        points = self._entrees.points_scan_robot()
        if points is None:
            self.get_logger().warning(
                'Scan absent, périmé ou sans TF exacte : alignement immobile.',
                throttle_duration_sec=2.0)
            return 0.0, 0.0
        vitesse = vitesse_sure(
            commande, points, self._navigation.footprint, x_avant, x_arriere,
            demi_largeur, self._course.front_guard_m,
            self._entrees.scan_couvre_tout_autour(),
            self._course.alignment.allow_reverse)
        if ((commande.lineaire, commande.angulaire) != (0.0, 0.0)
                and vitesse == (0.0, 0.0)):
            self.get_logger().warning(
                'Trajet ou balayage du pivot non dégagé : observation immobile.',
                throttle_duration_sec=2.0)
        return vitesse

    def _position_acceptable(self, erreurs, tolerance_laterale):
        """Objectif de précision non atteint à temps : tolérances d'acceptation."""
        if erreurs is None:
            return False
        course = self._course
        acceptable = visite_valide(
            erreurs, course.distance_tolerance_m, tolerance_laterale,
            math.radians(course.heading_tolerance_deg))
        if acceptable:
            self.get_logger().warning(
                'Précision non atteinte à temps : position acceptée '
                f'(distance {erreurs.distance:+.3f} m, cap {math.degrees(erreurs.cap):+.1f}°).')
        return acceptable

    def _valider_visite(self, tag_id):
        self.arreter_robot()
        self._statut.signaler_visite(tag_id, self._entrees.image_preuve(tag_id))
        self.publier_etat('VISITE', tag_id, f'maintien {self._course.hold_sec:.0f} s')
        fin = self._temps() + self._course.hold_sec
        while rclpy.ok() and self._temps() < fin and not self._temps_ecoule():
            self.publier_vitesse(0.0, 0.0)
            self.traiter_pendant(0.2)
        return 'visite'

    # ---------------------------------------------------------------- fin
    def _terminer(self, motif):
        self._navigateur.annuler_but()
        self.arreter_robot(5)
        resume = (self._sequence.resume() if self._sequence is not None
                  else {'ordre': [], 'visites': [], 'sautes': []})
        resume['temps_ecoule_s'] = round(self._chrono.ecoule(), 1)
        self._statut.signaler_fin(motif, resume)
        self._exporter_memoire()
        self.publier_etat('TERMINEE', detail=motif)

    def _exporter_memoire(self):
        fichier = self._course.tags_output_file
        if not fichier:
            return
        chemin = Path(fichier).expanduser()
        try:
            chemin.parent.mkdir(parents=True, exist_ok=True)
            chemin.write_text(yaml.safe_dump(
                self._memoire.exporter(), allow_unicode=True), encoding='utf-8')
            self.get_logger().info(f'Poses des tags vus enregistrées : {chemin}')
        except OSError as erreur:
            self.get_logger().error(f'Export des tags impossible : {erreur}')

    def arret_urgence(self):
        """Arrêt sur Ctrl-C ou SIGTERM : plus aucune commande, robot immobile."""
        self._navigateur.annuler_but()
        self.arreter_robot(10)


def _interruption(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    # On garde la main sur les signaux pour envoyer des vitesses nulles avant
    # que rclpy ne ferme son contexte.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _interruption)
    signal.signal(signal.SIGTERM, _interruption)
    # Session SSH coupée sans tmux : arrêt propre plutôt que mort brutale.
    signal.signal(signal.SIGHUP, _interruption)
    try:
        configuration = load_config()
    except ConfigurationError as erreur:
        print(f'course_parc : configuration invalide : {erreur}', file=sys.stderr)
        sys.exit(1)

    noeud = NoeudCourse(configuration)
    try:
        noeud.executer()
        # Course terminée : le nœud reste actif (état et statut disponibles).
        while rclpy.ok():
            rclpy.spin_once(noeud, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    except Exception as erreur:
        # Un Ctrl-C reçu pendant un appel C++ de rclpy arrive sous cette forme.
        noeud.get_logger().error(f'Arrêt de la course : {erreur!r}')
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
        if rclpy.ok():
            noeud.arret_urgence()
        noeud.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
