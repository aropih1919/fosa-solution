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

from datetime import datetime
import json
import math
import signal
import sys
import time

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
    obstacle_derriere,
    obstacle_devant,
)
from caytu_course.entrees_course import EntreesCourse
from caytu_course import recit
from caytu_course.geometrie_tags import erreurs_visite, pose_approche, visite_valide
from caytu_course.lancement import (
    NOM_CARTE,
    carte_preparee,
    dossier_preparation,
    fichier_tags,
)
from caytu_course.memoire_tags import MemoireTags
from caytu_course.recherche import points_recherche
from caytu_course.sequence import SequenceTags
from caytu_course.strategie import (
    ModeleStrategie,
    TagRestant,
    decider,
    duree_tag,
    planifier,
)
from caytu_course.statut import PublicateurArbitre, qos_evenements, qos_statut
from caytu_nav_solution.nav_math import Pose2D, lacet_depuis_quaternion
from caytu_nav_solution.navigation_runtime import (
    ChronometreCourse,
    NavigateurNav2,
)

PERIODE_BOUCLE = 0.1
DELAI_SERVICE_SLAM = 30.0


class NoeudCourse(Node):

    def __init__(self, configuration):
        super().__init__('course_parc')
        self._configuration = configuration
        self._course = configuration.course
        self._navigation = configuration.navigation
        self._mode = self.declare_parameter('mode', 'course').value
        self._etat = None
        self._en_attente = True
        self._tours = []           # positions où un tour sur place a été fait
        self._rotations_rates = []  # positions où le tour sur place a échoué
        self._rotation_indisponible = False  # behavior Spin absent de Nav2
        self._points_essayes = []  # points de recherche déjà tentés
        self._echecs = {}          # visites ratées par tag
        self._debut_cible = {}     # instant où chaque tag est devenu la cible
        self._plan = None          # dernier plan de la stratégie
        self._plan_instant = None
        self._garder = True
        self._position_plan = None
        self._modele_strategie = self._modele()
        self._tags_reperes = set()     # tags déjà annoncés dans le récit
        self._raison_echec = None      # dernière raison d'échec d'une visite
        self._dernieres_erreurs = None  # placement mesuré à la fin de l'alignement
        self._raisons = {}             # tag non visité -> pourquoi
        self._passages = 0             # parcours complets de la carte
        self._echecs_trajet = 0        # trajets de recherche ratés de suite
        self._preparation_enregistree = False
        self._origine_recherche = None
        self._debut_ros = self.get_clock().now()
        self._publicateur_evenements = self.create_publisher(
            String, self._course.events_topic, qos_evenements())

        self._memoire_tf = Buffer()
        self._ecouteur_tf = TransformListener(
            self._memoire_tf, self, spin_thread=False)
        self._memoire = self._charger_memoire()
        self._tags_reperes.update(self._memoire.ids())
        # Minuteur : les tags repérés sont annoncés aussi pendant un trajet Nav2.
        self.create_timer(0.5, self._annoncer_tags_reperes)
        self._entrees = EntreesCourse(
            self, configuration, self._memoire, self._memoire_tf,
            lambda: self._en_attente, ecouter_arbitre=self._mode == 'course')
        self._statut = PublicateurArbitre(self, self._course.referee)
        self._publicateur_etat = self.create_publisher(
            String, self._course.state_topic, qos_statut(1))
        self._publicateur_vitesse = self.create_publisher(
            Twist, self._navigation.securite.topic_entree, 10)

        duree = (self._course.preparation.max_duration_sec
                 if self._mode in ('preparation', 'preparation_manuelle')
                 else self._course.time_limit_sec)
        self._chrono = ChronometreCourse(
            # L'arbitre officiel utilise le temps réel, même dans Gazebo.
            lambda: Time(nanoseconds=time.monotonic_ns()),
            duree, self._course.stop_margin_sec)
        self._navigateur = NavigateurNav2(
            self, self._navigation, self._chrono, self.pose_robot,
            interrompre=self._temps_ecoule)
        self._reglages = self._reglages_alignement()
        self._sequence = None

    # -------------------------------------------------------------- outils
    def _charger_memoire(self):
        """Match sur la carte de la préparation : les tags enregistrés sont
        dans le même repère. Sinon (préparation, carte en direct) : vide."""
        distance_max = self._configuration.perception.apriltag.max_distance_m
        chemin = fichier_tags(self._configuration)
        if (self._mode != 'course' or carte_preparee(self._configuration) is None
                or not chemin.is_file()):
            return MemoireTags(distance_max)
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
            tolerance_laterale=course.lateral_tolerance_m,
            tolerance_distance=course.distance_tolerance_m,
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

    def evenement(self, genre, texte, **donnees):
        """Une étape de la course, en clair (topic des événements et journal)."""
        donnees.update(type=genre, texte=texte,
                       temps_ecoule=round(self._chrono.ecoule(), 1),
                       temps_restant=round(self._chrono.restant(), 1))
        self._publicateur_evenements.publish(
            String(data=json.dumps(donnees, ensure_ascii=False)))
        self.get_logger().info(f'» {texte}')

    def _annoncer_tags_reperes(self):
        cible = self._sequence.cible() if self._sequence is not None else None
        for tag_id in self._memoire.ids():
            if tag_id in self._tags_reperes:
                continue
            self._tags_reperes.add(tag_id)
            pose = self._memoire.pose(tag_id)
            self.evenement('tag_repere', recit.phrase_tag_repere(
                tag_id, pose.distance_min, cible), tag=tag_id,
                x=round(pose.x, 2), y=round(pose.y, 2))

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
        age = (self.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
        if not 0.0 <= age <= self._navigation.securite.age_scan_max_sec:
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

    def _temps_ecoule(self):
        # L'arbitre peut terminer la course (FINISHED, TIMED_OUT, TERMINATED) :
        # toutes les boucles d'attente s'arrêtent alors comme sur un temps écoulé.
        return self._chrono.expire() or self._entrees.arbitre_termine

    def _temps(self):
        """Horloge physique pour l'asservissement ; le budget du match est réel."""
        return (self.get_clock().now() - self._debut_ros).nanoseconds * 1e-9

    # ------------------------------------------------------------- course
    def executer(self):
        if self._mode == 'preparation':
            self.preparer()
            return
        if self._mode == 'preparation_manuelle':
            self.preparer_en_guidage()
            return
        if not self._statut.pret():
            raise RuntimeError('parc_msgs absent : impossible de signaler la course.')
        self.evenement('donnees', (
            'Navigation sans carte : obstacles mesurés en odom, tags découverts en direct.'
            if self._navigation.mode == 'sans_carte' else recit.phrase_donnees_match(
                carte_preparee(self._configuration), self._memoire.ids())))
        self._attendre_ordre_et_depart()
        if not rclpy.ok():
            return
        if self._entrees.arbitre_termine:
            self._terminer(self._entrees.motif_arbitre)
            return
        self._en_attente = False
        self._chrono.demarrer()
        self.publier_etat('DEMARRAGE', detail=self._entrees.source_depart)
        self.evenement('depart', recit.phrase_depart(
            self._entrees.source_depart, self._entrees.mot_entendu))
        # Le départ est signalé à l'arbitre dès qu'il est donné, même avant
        # l'ordre : son chronomètre démarre à ce signal. Un départ publié
        # directement sur /parc/start est déjà connu de l'arbitre.
        if not self._entrees.depart_de_l_arbitre:
            self._statut.signaler_depart(
                self._entrees.source_depart == 'voix', self._entrees.mot_entendu)
        self._attendre_ordre_apres_depart()
        if self._entrees.ordre is None:
            self._terminer(self._entrees.motif_arbitre or 'ordre_absent')
            return
        self._sequence = SequenceTags(self._entrees.ordre)
        self.evenement('ordre', recit.phrase_ordre(self._sequence.ordre),
                       ordre=list(self._sequence.ordre))
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
                if not self._sequence.terminee():
                    self._degager('laisser la place devant le tag avant le suivant')
            elif resultat == 'saute':
                self._sequence.sauter()
            self._plan_instant = None   # nouvelle cible : replanifier
        self._terminer(self._entrees.motif_arbitre or (
            'terminee' if self._sequence.terminee() else 'temps_ecoule'))

    def _attendre_ordre_et_depart(self):
        while (rclpy.ok() and self._entrees.source_depart is None
               and not self._entrees.arbitre_termine):
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
            self.publier_etat('DEMARRAGE', detail='attente de Nav2 et des repères')
            if self._navigateur.attendre_pret(
                    self._course.nav2_ready_timeout_sec):
                pose = self.pose_robot()
                if pose is not None:
                    self._origine_recherche = (pose.x, pose.y)
                return
            self.get_logger().error('Nav2 ou les transformations du robot ne répondent pas.')

    def _traiter_tag(self, tag_id):
        if tag_id not in self._debut_cible:
            self._debut_cible[tag_id] = self._temps()
            pose = self._memoire.pose(tag_id)
            self.evenement('cible', recit.phrase_cible(
                tag_id, pose is not None, pose.distance_min if pose else None), tag=tag_id)

        def abandonner():
            return not self._garder_cible(tag_id)

        while rclpy.ok():
            if self._temps_ecoule():
                return 'temps_ecoule'
            if abandonner():
                self._expliquer_saut(tag_id)
                return 'saute'
            if self._memoire.connait(tag_id):
                resultat = self._visiter(tag_id, abandonner)
                if resultat in ('visite', 'temps_ecoule'):
                    return resultat
                if resultat == 'perdu':
                    self.evenement('perdu', f'Tag {tag_id} introuvable à sa position '
                                   'mémorisée : on le cherche à nouveau.', tag=tag_id)
                    self._memoire.oublier(tag_id)
                    self._tags_reperes.discard(tag_id)
                elif resultat == 'echec' and self._garder:
                    self._echecs[tag_id] = self._echecs.get(tag_id, 0) + 1
                    self.evenement('echec', recit.phrase_echec(
                        tag_id, self._raison_echec or 'alignement', self._echecs[tag_id]),
                        tag=tag_id, raison=self._raison_echec)
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
            garder, plan = decider(position, self._tags_restants(),
                                   self._chrono.restant(), self._modele_strategie)
            if self._plan is None or plan.tags != self._plan.tags:
                self.evenement('plan', recit.phrase_plan(plan, self._chrono.restant()),
                               plan=list(plan.tags), duree=round(plan.duree_s))
            self._plan, self._plan_instant, self._garder = plan, maintenant, garder
            self._position_plan = position
        return self._garder

    def _expliquer_saut(self, tag_id):
        restants = self._tags_restants()
        cout, _ = duree_tag(self._position_plan, restants[0], self._modele_strategie)
        avec = planifier(self._position_plan, restants, self._chrono.restant(),
                         self._modele_strategie, avec_premier=True)
        texte = recit.phrase_saut(tag_id, cout, self._chrono.restant(), self._plan, avec)
        self._raisons[tag_id] = (f'sauté par la stratégie (≈ {cout:.0f} s nécessaires, '
                                 f'{self._chrono.restant():.0f} s restantes ; '
                                 f'{list(self._plan.tags)} rapportait plus)')
        self.evenement('saut', texte, tag=tag_id, cout=round(cout),
                       restant=round(self._chrono.restant()), plan=list(self._plan.tags))

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
        deja_tourne = any(
            math.dist((pose.x, pose.y), tour) <= rayon
            for tour in self._tours + self._rotations_rates)
        if not self._rotation_indisponible and not deja_tourne:
            self.publier_etat('RECHERCHE', tag_id, 'tour sur place')
            resultat = self._navigateur.tourner(
                2.0 * math.pi, self._course.search_spin_time_sec, interrompre)
            if resultat == 'succes':
                self._tours.append((pose.x, pose.y))
            elif resultat in ('indisponible', 'echec'):
                # Ne pas boucler sur un spin impossible : marquer la position
                # comme tentée pour passer à la navigation vers un point.
                self._rotations_rates.append((pose.x, pose.y))
                self.get_logger().warning(
                    f'Tour sur place impossible ({resultat}) : '
                    'recherche d\'un point de passage.')
                self._rotation_indisponible = resultat == 'indisponible'
            return

        if self._entrees.grille() is None:
            self.publier_etat('RECHERCHE', tag_id, 'carte en attente')
            self.traiter_pendant(0.5)
            return
        point = self._prochain_point(pose)
        if point is None:
            # Sans candidat sûr, refaire le même tour indéfiniment ne fait pas
            # progresser la préparation : borner les cycles sans passage.
            self._passages += 1
            self.get_logger().warning(
                f'Aucun point de recherche praticable après un balayage '
                f'({self._passages}/2).')
            self.publier_etat('RECHERCHE', tag_id, 'aucun passage praticable')
            self._tours.clear()
            self._points_essayes.clear()
            self.traiter_pendant(0.5)
            return
        self.publier_etat('RECHERCHE', tag_id, f'vers ({point[0]:.2f}, {point[1]:.2f})')
        self._points_essayes.append(point)
        resultat = self._navigateur.naviguer_une_fois(
            point[0], point[1], self._configuration.system.global_frame,
            Pose2D(0.0, 0.0, 0.0), interrompre=interrompre)
        if resultat in ('refuse', 'reessayer'):
            self._echecs_trajet += 1
            if self._echecs_trajet >= self._course.unstuck_after_failures:
                self._degager(f'{self._echecs_trajet} trajets ratés de suite')
                self._echecs_trajet = 0
            else:
                self.traiter_pendant(0.5)
        else:
            self._echecs_trajet = 0

    def _prochain_point(self, pose):
        grille = self._entrees.grille()
        if grille is None:
            return None
        # L'odométrie du robot partagé ne démarre pas nécessairement à zéro.
        if self._origine_recherche is None:
            self._origine_recherche = (pose.x, pose.y)
        points = points_recherche(
            grille, self._course.search_clearance_m, self._course.search_grid_step_m,
            self._tours + self._points_essayes, self._course.search_visited_radius_m,
            (pose.x, pose.y), centre=self._origine_recherche,
            rayon_max=self._course.search_max_radius_m,
            clairance_inconnu_m=self._course.search_frontier_margin_m)
        if not points:
            # L'arène officielle a des meubles rapprochés : la recherche de
            # secours tolère une frontière inconnue plus proche, mais garde la
            # marge obstacles au-dessus du rayon d'encombrement du robot.
            pas_secours = max(grille.resolution, self._course.search_grid_step_m / 2.0)
            points = points_recherche(
                grille, self._course.search_fallback_clearance_m, pas_secours,
                self._tours + self._points_essayes,
                self._course.search_visited_radius_m, (pose.x, pose.y),
                centre=self._origine_recherche, rayon_max=self._course.search_max_radius_m,
                clairance_inconnu_m=self._course.search_fallback_frontier_margin_m)
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
        # Distance depuis la position ACTUELLE du robot : distance_min est celle
        # de la meilleure vue (souvent faite en préparation, ailleurs dans l'arène).
        distance = (math.hypot(memoire.x - robot.x, memoire.y - robot.y)
                    if robot is not None else memoire.distance_min)
        return memoire.x, memoire.y, memoire.cap, distance

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
            self._raison_echec = 'nav2'
            self.traiter_pendant(0.5)
            return 'reessayer'
        # Le tag doit être REVU de près (la mémoire seule ne prouve rien) ; la
        # caméra a jusqu'à une seconde pour le détecter après l'arrêt.
        portee = course.approach_distance_m + course.preapproach_margin_m
        fin = self._temps() + 1.0      # horloge ROS : Gazebo peut être lent
        while rclpy.ok() and self._temps() < fin and not self._temps_ecoule():
            vue = self._entrees.detection_recente(tag_id, course.detection_max_age_sec)
            if vue is not None and vue.distance <= portee:
                return 'arrive'   # même si Nav2 a abandonné : le tag est à portée
            self.traiter_pendant(PERIODE_BOUCLE)
        if resultat == 'reessayer':
            self._raison_echec = 'nav2'
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
                self._raison_echec = 'alignement'
                self._dernieres_erreurs = erreurs
                return ('aligne' if self._position_acceptable(erreurs, tolerance_laterale)
                        else 'echec')
            vue = self._entrees.detection_recente(tag_id, course.detection_max_age_sec)
            if vue is None:
                # Tourner vers la pose mémorisée avant de conclure à une perte.
                self.publier_vitesse(0.0, self._rotation_vers_memoire(tag_id))
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
                self._raison_echec = 'biais'
                return 'renaviguer'
            if commande.etat == 'aligne':
                self.publier_vitesse(0.0, 0.0)
                stables += 1
                if stables >= course.stable_cycles:
                    self._dernieres_erreurs = erreurs
                    return 'aligne'
            else:
                stables = 0
                self.publier_vitesse(*self._vitesse_gardee(
                    commande, x_avant, x_arriere, demi_largeur))
            self.traiter_pendant(PERIODE_BOUCLE)
        return 'echec'

    def _degager(self, raison):
        """Recule lentement de `unstuck_distance_m`, garde arrière active.

        Sert à quitter un endroit étroit (devant un tag, contre un meuble) où
        Nav2 n'arrive plus à repartir.
        """
        distance = self._course.unstuck_distance_m
        vitesse = self._course.alignment.max_linear_mps
        x_arriere = min(x for x, _ in self._navigation.footprint)
        demi_largeur = max(abs(y) for _, y in self._navigation.footprint)
        depart = self.pose_robot()
        if depart is None:
            return
        self.evenement('degagement', f'Recul de {distance * 100:.0f} cm pour se dégager '
                        f'({raison}).')
        self.publier_etat('DEGAGEMENT', detail=raison)
        fin = self._temps() + 3.0 * distance / vitesse
        while rclpy.ok() and self._temps() < fin and not self._temps_ecoule():
            pose = self.pose_robot()
            if pose is not None and math.hypot(pose.x - depart.x, pose.y - depart.y) >= distance:
                break
            if obstacle_derriere(self._entrees.points_scan_robot(), x_arriere, demi_largeur,
                                 self._course.front_guard_m):
                self.evenement('degagement', 'Obstacle derrière : recul arrêté.')
                break
            self.publier_vitesse(-vitesse, 0.0)
            self.traiter_pendant(PERIODE_BOUCLE)
        self.arreter_robot()

    def _vitesse_gardee(self, commande, x_avant, x_arriere, demi_largeur):
        """Annule l'avance (ou le recul) si un obstacle occupe le couloir."""
        lineaire = commande.lineaire
        if lineaire == 0.0:
            return 0.0, commande.angulaire
        points = self._entrees.points_scan_robot()
        garde = self._course.front_guard_m
        bloque = (obstacle_devant(points, x_avant, demi_largeur, garde) if lineaire > 0.0
                  else obstacle_derriere(points, x_arriere, demi_largeur, garde))
        if bloque:
            self.get_logger().warning(
                'Obstacle sur le trajet : déplacement bloqué.', throttle_duration_sec=2.0)
            return 0.0, commande.angulaire
        return lineaire, commande.angulaire

    def _rotation_vers_memoire(self, tag_id):
        """Vitesse angulaire pour regarder vers le tag mémorisé (0 si inconnu)."""
        tag, robot = self._memoire.pose(tag_id), self.pose_robot()
        if tag is None or robot is None:
            return 0.0
        relevement = math.atan2(math.sin(
            math.atan2(tag.y - robot.y, tag.x - robot.x) - robot.yaw), math.cos(
            math.atan2(tag.y - robot.y, tag.x - robot.x) - robot.yaw))
        if abs(relevement) <= math.radians(self._course.heading_tolerance_deg):
            return 0.0
        vitesse = min(self._reglages.max_angulaire,
                      max(self._reglages.min_angulaire, abs(relevement)))
        return math.copysign(vitesse, relevement)

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
        if self._temps_ecoule() or self._chrono.restant() < self._course.hold_sec:
            return 'temps_ecoule'
        vue = self._entrees.detection_recente(tag_id, self._course.detection_max_age_sec)
        if vue is None:
            return 'perdu'
        erreurs = erreurs_visite(vue.x, vue.y, vue.cap, self._navigation.center_offset_x,
                                self._course.visit_distance_m)
        if not visite_valide(erreurs, self._course.distance_tolerance_m,
                             self._course.lateral_tolerance_m,
                             math.radians(self._course.heading_tolerance_deg)):
            return 'echec'
        if not self._statut.signaler_visite(tag_id, self._entrees.image_preuve(tag_id)):
            self._raison_echec = 'image_absente'
            return 'echec'
        self._dernieres_erreurs = erreurs
        if erreurs is not None:
            self.evenement('visite', recit.phrase_visite(
                tag_id, len(self._sequence.visites) + 1, len(self._sequence.ordre),
                self._course.visit_distance_m + erreurs.distance,
                math.degrees(erreurs.cap)), tag=tag_id)
        self.publier_etat('VISITE', tag_id, f'maintien {self._course.hold_sec:.0f} s')
        fin = time.monotonic() + self._course.hold_sec
        # FINISHED arrive dès le signalement du dernier tag. Garder la pose
        # pendant le délai réglementaire ; seul un timeout ou un arrêt du juge
        # interrompt ce maintien.
        while (rclpy.ok() and time.monotonic() < fin
               and not self._entrees.arbitre_interrompt_visite):
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
        if self._sequence is not None and not self._sequence.terminee():
            cible = self._sequence.cible()
            self._raisons.setdefault(cible, (
                'temps écoulé pendant la recherche' if not self._memoire.connait(cible)
                else 'temps écoulé pendant l’approche ou le placement'))
        lignes = recit.resume_fin(resume['ordre'], resume['visites'], self._raisons)
        if motif == 'ordre_absent':
            lignes.insert(0, "Aucun ordre reçu de l'arbitre : rien n'a pu être visité.")
        elif motif == 'arbitre_termine':
            lignes.insert(0, "L'arbitre a mis fin à la course : robot arrêté.")
        self.evenement('fin', ' '.join(lignes), lignes=lignes, motif=motif)
        self.publier_etat('TERMINEE', detail=motif)

    # -------------------------------------------------------- préparation
    def preparer(self):
        """Exploration autonome avant le match : carte neuve et tags mesurés.

        Fin quand toute la carte a été parcourue et les 4 tags mesurés de
        près, ou à la limite de temps. Les tags sont réécrits à chaque étape :
        un arrêt anticipé ne perd rien ; la carte est enregistrée à la fin.
        """
        preparation = self._course.preparation
        dossier = self._nouveau_dossier_preparation()
        self._en_attente = False
        self._chrono.demarrer()
        self.evenement('preparation', recit.phrase_preparation(
            dossier, preparation.max_duration_sec))
        self._attendre_nav2()
        tentatives = {}
        while rclpy.ok() and not self._temps_ecoule():
            self._enregistrer_tags()
            a_mesurer = [
                tag for tag in self._course.tag_ids if self._memoire.connait(tag)
                and self._memoire.pose(tag).distance_min > preparation.close_view_m
                and tentatives.get(tag, 0) < preparation.approach_attempts]
            if a_mesurer:
                tag = a_mesurer[0]
                tentatives[tag] = tentatives.get(tag, 0) + 1
                self._mesurer_de_pres(tag)
                continue
            if self._exploration_terminee():
                break
            self._chercher(None, self._temps_ecoule)
        self.arreter_robot()
        self.enregistrer_preparation()

    def preparer_en_guidage(self):
        """Enregistre carte et tags sans envoyer de commande de déplacement."""
        preparation = self._course.preparation
        dossier = self._nouveau_dossier_preparation()
        self._en_attente = False
        self._chrono.demarrer()
        self.evenement(
            'preparation_manuelle',
            f'Cartographie guidée : vous pilotez le robot. '
            f'La carte et les tags observés seront enregistrés dans {dossier}.')
        while rclpy.ok() and not self._temps_ecoule():
            self._enregistrer_tags()
            self.traiter_pendant(0.5)
        self.enregistrer_preparation()

    def _exploration_terminee(self):
        """Fin dès que les 4 tags sont mesurés de près : explorer davantage
        n'apporte rien au match et risque de coincer le robot. Sinon, deux
        parcours complets de la carte sans les trouver tous."""
        proche = self._course.preparation.close_view_m
        mesures = all(
            self._memoire.connait(tag) and self._memoire.pose(tag).distance_min <= proche
            for tag in self._course.tag_ids)
        return mesures or self._passages >= 2

    def _mesurer_de_pres(self, tag_id):
        pose = self._memoire.pose(tag_id)
        self.evenement('mesure', f'Tag {tag_id} vu de loin ({pose.distance_min:.1f} m) : '
                       'on s\'en approche pour mesurer précisément sa position.', tag=tag_id)
        resultat = self._approcher(
            tag_id, (pose.x, pose.y, pose.cap, pose.distance_min), self._temps_ecoule)
        self.traiter_pendant(1.0)
        nouvelle = self._memoire.pose(tag_id)
        if nouvelle is not None and nouvelle.distance_min <= self._course.preparation.close_view_m:
            self.evenement('mesure', f'Tag {tag_id} mesuré de près, à {nouvelle.distance_min:.1f} m : '
                           'position précise enregistrée.', tag=tag_id)
        elif nouvelle is not None:
            self.evenement('mesure', f'Tag {tag_id} pas revu de près ({recit.RESULTATS_APPROCHE.get(resultat, resultat)}) : '
                           f'meilleure vue à {nouvelle.distance_min:.1f} m, nouvelle tentative plus tard.',
                           tag=tag_id)
        self._degager('quitter les abords du tag avant la suite')

    def _nouveau_dossier_preparation(self):
        """Archive la préparation précédente : la carte de mardi peut être fausse
        vendredi, chaque préparation repart de zéro."""
        dossier = dossier_preparation(self._configuration)
        if dossier.exists() and any(dossier.iterdir()):
            archive = dossier.with_name(
                f'{dossier.name}_{datetime.now():%Y%m%d_%H%M%S}')
            dossier.rename(archive)
            self.get_logger().info(f'Préparation précédente archivée : {archive}')
        dossier.mkdir(parents=True, exist_ok=True)
        return dossier

    def _enregistrer_tags(self):
        chemin = fichier_tags(self._configuration)
        try:
            chemin.write_text(yaml.safe_dump(
                self._memoire.exporter(), allow_unicode=True), encoding='utf-8')
        except OSError as erreur:
            self.get_logger().error(f'Enregistrement des tags impossible : {erreur}',
                                    throttle_duration_sec=10.0)

    def enregistrer_preparation(self):
        """Tags puis carte slam_toolbox (posegraph pour le match, image pour voir)."""
        if (self._mode not in ('preparation', 'preparation_manuelle')
                or self._preparation_enregistree):
            return
        self._preparation_enregistree = True
        self._enregistrer_tags()
        base = dossier_preparation(self._configuration) / NOM_CARTE
        from slam_toolbox.srv import SaveMap, SerializePoseGraph
        from std_msgs.msg import String as Texte
        reponse = self._appeler_slam(
            SerializePoseGraph, '/slam_toolbox/serialize_map',
            SerializePoseGraph.Request(filename=str(base)))
        serialisee = reponse is not None and reponse.result == 0
        self._appeler_slam(SaveMap, '/slam_toolbox/save_map',
                           SaveMap.Request(name=Texte(data=str(base))))
        manquants = [tag for tag in self._course.tag_ids if not self._memoire.connait(tag)]
        lignes = [
            f'Carte enregistrée : {base}.posegraph' if serialisee
            else 'ÉCHEC de l’enregistrement de la carte : relancer une préparation.',
            f'Tags enregistrés : {", ".join(map(str, self._memoire.ids())) or "aucun"} '
            f'({fichier_tags(self._configuration)}).']
        if manquants:
            lignes.append(f'Tags non trouvés : {", ".join(map(str, manquants))} '
                          '(le match les cherchera).')
        lignes.append('Pour le match : replacer le robot à la position de départ de la '
                      'préparation, puis ./run.sh course.')
        self.evenement('fin', ' '.join(lignes), lignes=lignes, motif='preparation')
        self.publier_etat('TERMINEE', detail='preparation')

    def _appeler_slam(self, type_service, nom, requete):
        client = self.create_client(type_service, nom)
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service {nom} indisponible.')
            return None
        future = client.call_async(requete)
        fin = time.monotonic() + DELAI_SERVICE_SLAM
        while rclpy.ok() and not future.done() and time.monotonic() < fin:
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.result() if future.done() else None

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
        # Le match garde son statut vivant. Une préparation s'arrête après
        # avoir enregistré la carte et les tags.
        if noeud._mode not in ('preparation', 'preparation_manuelle'):
            while rclpy.ok():
                rclpy.spin_once(noeud, timeout_sec=0.5)
    except KeyboardInterrupt:
        if noeud._mode in ('preparation', 'preparation_manuelle'):
            noeud.get_logger().warning(
                'Arrêt demandé : les tags sont conservés, mais SLAM doit encore '
                'tourner pour sauvegarder la carte avec ./run.sh sauver-carte.')
    except Exception as erreur:
        # Un Ctrl-C reçu pendant un appel C++ de rclpy arrive sous cette forme.
        noeud.get_logger().error(f'Arrêt de la course : {erreur!r}')
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
        if rclpy.ok():
            noeud.arret_urgence()
            # Au signal, launch arrête SLAM en même temps que ce nœud : ne pas
            # appeler son service de sauvegarde après qu'il a disparu.
            if (noeud._mode in ('preparation', 'preparation_manuelle')
                    and not noeud._preparation_enregistree):
                noeud._enregistrer_tags()
        noeud.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
