#!/usr/bin/env python3
"""Point d'entrée officiel PARC 2026 — Engineers League, tâche de navigation.

Commande d'évaluation (après `ros2 launch parc_robot_bringup task.launch.py`) :

    ros2 run caytu_nav_solution task_solution.py

Ce nœud exécute TOUTE la solution, sans aucune intervention manuelle :
  1. lit le point de départ et le but dans task_params.yaml (repère Gazebo) ;
  2. vérifie que la carte des murs correspond bien au monde chargé ;
  3. démarre la perception, la localisation et Nav2 (launch du bringup) ;
  4. envoie le but à Nav2 et le renvoie après chaque échec, tant que la limite
     de 10 minutes n'est pas atteinte ;
  5. arrête le robot sur le but et ferme proprement tout ce qu'il a lancé.

Le laboratoire de navigation peut choisir un couple profil / contrôleur /
localisation. Le YAML complet correspondant est généré au démarrage et passé
au bringup : aucune valeur d'essai n'est codée dans les nœuds.
"""

import math
import os
import signal
import sys
import tempfile
import time

from caytu_config import load_config
from caytu_nav_solution.benchmark_config import (
    ConfigurationError,
    validate_localization_map,
    validate_scan_map_agreement,
    validate_selection,
)
from caytu_nav_solution.nav_math import (
    accord_scan_carte,
    charger_carte_pgm,
    lacet_depuis_quaternion,
    Pose2D,
)
from caytu_nav_solution.navigation_runtime import (
    BringupManager,
    Nav2Navigator,
    secteur_arriere_est_libre,
    TaskTimer,
)
from caytu_nav_solution.task_params import (
    load_task_params,
    TaskParams,
)
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

# Code de sortie du programme : le shell et le jury peuvent le lire.
# 0 = but atteint ; 1 = échec ; 2 = temps écoulé.
EXIT_SUCCESS, EXIT_FAILURE, EXIT_TIMEOUT = 0, 1, 2

# Position du lidar dans base_footprint (URDF officiel) : 2,1 cm derrière le centre.
DECALAGE_LIDAR_X = -0.021


class SolutionTache(Node):

    def __init__(self, parameter_overrides=None, configuration=None):
        super().__init__('task_solution', parameter_overrides=parameter_overrides or [])

        self._configuration = configuration or load_config()
        self._declarer_parametres()
        self._lire_parametres()
        self._initialiser_etat()
        self._connecter_ros()

        self.get_logger().info(
            f'Départ ({self._task.spawn.x:.3f}, {self._task.spawn.y:.3f}, '
            f'cap {self._task.spawn.yaw:.3f}) -> but ({self._task.goal_x:.3f}, '
            f'{self._task.goal_y:.3f}), lus dans {self._task.path}')

    # ------------------------------------------------------------ réglages
    def _declarer_parametres(self):
        navigation = self._configuration.navigation
        systeme = self._configuration.system

        # Toutes les valeurs viennent de caytu_config/config/fosa.yaml.
        self.declare_parameter('time_limit_sec', navigation.time_limit_sec)
        self.declare_parameter('stop_margin_sec', navigation.stop_margin_sec)
        self.declare_parameter('success_radius', navigation.success_radius)
        self.declare_parameter(
            'arrival_recheck_radius', navigation.arrival_recheck_radius)
        self.declare_parameter('retry_pause_sec', navigation.retry_pause_sec)
        self.declare_parameter(
            'nav2_startup_timeout_sec', navigation.nav2_startup_timeout_sec)

        # Lancement du reste de la solution par ce nœud (exigence PARC).
        self.declare_parameter('launch_bringup', navigation.launch_bringup)
        self.declare_parameter('bringup_package', navigation.bringup_package)
        self.declare_parameter('bringup_launch', navigation.bringup_launch)
        self.declare_parameter('controller', navigation.controller)
        self.declare_parameter('localization', navigation.localization)
        self.declare_parameter('profile', navigation.profile)
        self.declare_parameter('nav2_params_file', '')

        # Carte des murs : auto (vérifiée avec le lidar), always, never.
        # (Pas « on/off » : YAML les lirait comme des booléens.)
        self.declare_parameter('static_map', navigation.static_map)
        self.declare_parameter('map_yaml', navigation.map_yaml)
        self.declare_parameter(
            'map_check_timeout_sec', navigation.map_check_timeout_sec)
        self.declare_parameter(
            'map_check_min_agreement', navigation.map_check_min_agreement)

        self.declare_parameter('task_params_file', '')
        self.declare_parameter('global_frame', systeme.global_frame)
        self.declare_parameter('base_frame', systeme.base_frame)
        self.declare_parameter('scan_topic', navigation.scan_topic)
        self.declare_parameter('amcl_pose_topic', navigation.amcl_pose_topic)
        self.declare_parameter('cmd_vel_topic', navigation.cmd_vel_topic)

    def _lire_parametres(self):
        # Lecture immédiate du fichier de départ/but : si le fichier manque, on
        # s'arrête tout de suite avec un message clair.
        self._task: TaskParams = load_task_params(
            self.get_parameter('task_params_file').value or None)
        self._repere_global = self.get_parameter('global_frame').value
        self._repere_robot = self.get_parameter('base_frame').value

        self._controleur = str(
            self.get_parameter('controller').value).lower()
        self._localisation = str(
            self.get_parameter('localization').value).lower()
        self._profil = str(self.get_parameter('profile').value).lower()
        validate_selection(self._controleur, self._localisation, self._profil)

    def _initialiser_etat(self):
        self._dernier_scan = None           # sert uniquement à vérifier la carte
        self._reception_dernier_scan = None
        self._pose_amcl = None              # dernière pose publiée par AMCL

    def _connecter_ros(self):
        # Buffer = mémoire des TF ; TransformListener = l'abonnement qui la remplit.
        # spin_thread=False : on traite les messages nous-mêmes dans nos boucles.
        self._memoire_tf = Buffer()
        self._ecouteur_tf = TransformListener(
            self._memoire_tf, self, spin_thread=False)

        navigation = self._configuration.navigation
        self._chronometre = TaskTimer(
            self.get_clock().now,
            self.get_parameter('time_limit_sec').value,
            self.get_parameter('stop_margin_sec').value)
        self._bringup = BringupManager(navigation, self.get_logger())
        self._navigateur = Nav2Navigator(
            self, navigation, self._chronometre, self._bringup,
            self.pose_robot,
            success_radius=self.get_parameter('success_radius').value,
            arrival_recheck_radius=self.get_parameter(
                'arrival_recheck_radius').value)

        # Canal pour envoyer une vitesse au robot (avancer / tourner).
        self._publicateur_vitesse = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)

        # On s'abonne à /scan uniquement pour vérifier la carte au démarrage.
        self._abonnement_scan = self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self.recevoir_scan, qos_profile_sensor_data)

        # En mode AMCL, la pose initiale doit être confirmée par le localiseur
        # avant d'envoyer un but.
        if self._localisation == 'amcl':
            self._abonnement_pose_amcl = self.create_subscription(
                PoseWithCovarianceStamped,
                self.get_parameter('amcl_pose_topic').value,
                self.recevoir_pose_amcl, 10)

    # ------------------------------------------------------------- utilitaires
    def recevoir_scan(self, message):
        # Le scan sert à la cohérence de carte puis à valider le secteur arrière
        # avant et pendant un éventuel recul.
        self._dernier_scan = message
        self._reception_dernier_scan = time.monotonic()

    def recevoir_pose_amcl(self, message):
        self._pose_amcl = message

    def traiter_messages_pendant(self, secondes_reelles):
        """Traite les messages ROS pendant une durée réelle donnée."""
        fin = time.monotonic() + secondes_reelles
        while rclpy.ok():
            restant = fin - time.monotonic()
            if restant <= 0.0:
                break
            # On ne peut pas utiliser time.sleep : pendant une pause, le nœud
            # ne traiterait aucun message (TF, résultats d'action...).
            rclpy.spin_once(self, timeout_sec=min(0.05, restant))

    def temps_ecoule(self):
        """Temps écoulé depuis le lancement de la solution (horloge du nœud)."""
        return self._chronometre.elapsed()

    def temps_restant(self):
        """Temps restant avant la limite officielle, marge de sécurité déduite."""
        return self._chronometre.remaining()

    def pose_robot(self):
        """Pose du robot dans map, ou None si la TF n'est pas encore connue."""
        try:
            tf = self._memoire_tf.lookup_transform(
                self._repere_global, self._repere_robot, Time())
        except TransformException:
            # Au tout début, l'odométrie n'a pas encore parlé : ce n'est pas
            # grave, on répond None au lieu de planter.
            return None

        translation = tf.transform.translation
        rotation = tf.transform.rotation
        return Pose2D(
            translation.x,
            translation.y,
            lacet_depuis_quaternion(
                rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def distance_vers_but(self):
        """Distance en ligne droite entre le robot et le but (None si inconnue)."""
        pose = self.pose_robot()
        if pose is None:
            return None
        return pose.distance_vers(self._task.goal_x, self._task.goal_y)

    def _pose_odom(self):
        """Pose robot dans odom, repère continu utilisé pour surveiller un recul."""
        try:
            tf = self._memoire_tf.lookup_transform(
                self._configuration.system.odom_frame,
                self._repere_robot, Time())
        except TransformException:
            return None
        translation = tf.transform.translation
        rotation = tf.transform.rotation
        return Pose2D(
            translation.x, translation.y,
            lacet_depuis_quaternion(
                rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def _secteur_arriere_libre(self, distance_requise):
        """Exige un scan frais, transformable et dense derrière le robot."""
        navigation = self._configuration.navigation
        if (self._dernier_scan is None or self._reception_dernier_scan is None
                or time.monotonic() - self._reception_dernier_scan
                > navigation.obstacle_rear_scan_max_age_sec):
            return False
        if self._dernier_scan.header.stamp.sec == 0:
            return False
        maintenant_ros = self.get_clock().now().nanoseconds
        if maintenant_ros > 0:
            age_scan = (maintenant_ros - Time.from_msg(
                self._dernier_scan.header.stamp).nanoseconds) * 1e-9
            if not 0.0 <= age_scan <= navigation.obstacle_rear_scan_max_age_sec:
                return False
        try:
            transformation = self._memoire_tf.lookup_transform(
                self._repere_robot, self._dernier_scan.header.frame_id,
                Time.from_msg(self._dernier_scan.header.stamp))
        except TransformException:
            return False
        return secteur_arriere_est_libre(
            self._dernier_scan, transformation, distance_requise,
            navigation.obstacle_rear_sector_half_angle_rad,
            navigation.obstacle_rear_min_samples,
            navigation.obstacle_rear_max_angular_gap_rad)

    def _recul_surveille(self):
        """Effectue un seul recul lent, borné et surveillé à chaque cycle."""
        navigation = self._configuration.navigation
        if not navigation.obstacle_retreat_enabled:
            return False, False

        distance = navigation.obstacle_retreat_distance_m
        marge_securite = navigation.obstacle_retreat_clearance_m
        if not self._secteur_arriere_libre(distance + marge_securite):
            self.get_logger().warning(
                'Recul refusé : espace arrière non couvert, scan trop ancien '
                'ou dégagement insuffisant.')
            return False, False
        pose_depart = self._pose_odom()
        if pose_depart is None:
            self.get_logger().warning('Recul refusé : pose odom indisponible.')
            return False, False

        self.get_logger().warning(
            f'Recul surveillé autorisé : {distance:.2f} m à '
            f'{navigation.obstacle_retreat_speed_mps:.2f} m/s.')
        fin = time.monotonic() + navigation.obstacle_retreat_timeout_sec
        commande = Twist()
        commande.linear.x = -navigation.obstacle_retreat_speed_mps
        succes = False
        try:
            while rclpy.ok() and time.monotonic() < fin:
                if self.temps_restant() <= 0.0:
                    break
                pose = self._pose_odom()
                if pose is None:
                    break
                dx, dy = pose.x - pose_depart.x, pose.y - pose_depart.y
                progression = -(dx * math.cos(pose_depart.yaw)
                                + dy * math.sin(pose_depart.yaw))
                derive_laterale = abs(
                    -dx * math.sin(pose_depart.yaw)
                    + dy * math.cos(pose_depart.yaw))
                if (derive_laterale > navigation.obstacle_retreat_max_lateral_m
                        or progression < -navigation.obstacle_min_progress_m):
                    self.get_logger().warning(
                        'Recul interrompu : trajectoire odom inattendue.')
                    break
                if progression >= distance:
                    succes = True
                    break
                restant = distance - max(progression, 0.0)
                if not self._secteur_arriere_libre(
                        restant + marge_securite):
                    self.get_logger().warning(
                        'Recul interrompu : dégagement arrière perdu ou scan '
                        'devenu insuffisant.')
                    break
                self._publicateur_vitesse.publish(commande)
                rclpy.spin_once(self, timeout_sec=0.05)
        finally:
            self.arreter_robot()

        if not succes:
            self.get_logger().warning(
                'Recul surveillé interrompu avant sa distance cible.')
        return True, succes

    def bringup_est_vivant(self):
        # Si aucun programme n'a été lancé (launch_bringup=false), le bringup
        # est considéré comme vivant pour ne pas bloquer la mission.
        return self._bringup.alive()

    # ------------------------------------------------------------------ carte
    def choisir_carte(self):
        """Retourne le YAML de carte à donner à Nav2 (murs réels ou carte vide)."""
        mode = str(self.get_parameter('static_map').value).lower()
        if mode not in ('auto', 'always', 'never'):
            raise ConfigurationError(f'static_map inconnu : {mode}')

        # AMCL exige une carte statique cohérente : elle ne peut pas être vide.
        carte_obligatoire = self._localisation == 'amcl'
        chemin_carte = self.get_parameter('map_yaml').value

        if not chemin_carte:
            # Aucun chemin fourni : on prend la carte livrée avec le paquet.
            from ament_index_python.packages import get_package_share_directory
            chemin_carte = os.path.join(
                get_package_share_directory(
                    self.get_parameter('bringup_package').value),
                self._configuration.navigation.default_map)

        if mode == 'never':
            if carte_obligatoire:
                raise ConfigurationError(
                    'AMCL ne peut pas démarrer avec static_map=never : une carte '
                    'statique connue et cohérente est obligatoire.')
            self.get_logger().info('Carte des murs désactivée (static_map=never).')
            return self.ecrire_carte_vide()

        if mode == 'always' and not carte_obligatoire:
            self.get_logger().info('Carte des murs imposée (static_map=always).')
            return chemin_carte

        # Mode auto : la carte décrit le café officiel. Si le monde chargé est
        # différent, elle gênerait le planificateur ; on le vérifie en comparant
        # un scan lidar pris au point de départ avec les murs de la carte.
        try:
            carte = self.charger_carte(chemin_carte)
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as erreur:
            if carte_obligatoire:
                raise ConfigurationError(
                    f'AMCL indisponible : carte statique illisible ({erreur}).'
                ) from erreur
            self.get_logger().warn(
                f'Carte illisible ({erreur}) : navigation sans carte.')
            return self.ecrire_carte_vide()

        if carte_obligatoire:
            validate_localization_map(
                chemin_carte,
                self._task.spawn,
                Pose2D(self._task.goal_x, self._task.goal_y),
            )

        scan = self.attendre_premier_scan()
        if scan is None:
            if carte_obligatoire:
                raise ConfigurationError(
                    "AMCL indisponible : aucun scan lidar n'est arrivé pour valider "
                    'la cohérence de la carte.')
            # Pas de lidar : on ne peut rien vérifier. On garde la carte du
            # café, car c'est le cas normal de l'évaluation.
            self.get_logger().warn(
                'Pas de scan reçu pour vérifier la carte : carte du café conservée.')
            return chemin_carte

        # La position du lidar n'est pas exactement celle du robot : il est
        # décalé de DECALAGE_LIDAR_X.
        pose_lidar = self._task.spawn.composer(
            Pose2D(DECALAGE_LIDAR_X, 0.0, 0.0))
        accord, utilises = accord_scan_carte(
            carte, pose_lidar, scan.angle_min, scan.angle_increment, scan.ranges)

        seuil = float(self.get_parameter('map_check_min_agreement').value)
        if carte_obligatoire:
            validate_scan_map_agreement(accord, utilises, seuil)
        # utilises >= 30 : avec moins de 30 rayons, le résultat n'est pas fiable.
        if utilises >= 30 and accord < seuil:
            self.get_logger().warn(
                f'Seulement {accord * 100:.0f} % des {utilises} retours lidar tombent '
                'sur un mur de la carte : le monde ne semble pas être le café '
                'officiel, navigation sans carte statique.')
            return self.ecrire_carte_vide()

        self.get_logger().info(
            f'Carte du café validée : {accord * 100:.0f} % des {utilises} retours '
            'lidar coïncident avec ses murs.')
        return chemin_carte

    def charger_carte(self, chemin_carte):
        """Lit le YAML et le PGM d'une carte, et renvoie la grille d'occupation."""
        with open(chemin_carte, encoding='utf-8') as flux:
            meta = yaml.safe_load(flux)

        image = meta['image']
        if not os.path.isabs(image):
            # Le nom de l'image peut être relatif : on le cherche dans le même
            # dossier que le YAML.
            image = os.path.join(os.path.dirname(chemin_carte), image)

        with open(image, 'rb') as flux:
            return charger_carte_pgm(
                flux.read(),
                float(meta['resolution']),
                float(meta['origin'][0]),
                float(meta['origin'][1]),
            )

    def attendre_premier_scan(self):
        """Attend un premier scan, au maximum map_check_timeout_sec (temps réel)."""
        delai = time.monotonic() + float(
            self.get_parameter('map_check_timeout_sec').value)
        while (rclpy.ok() and self._dernier_scan is None
               and time.monotonic() < delai):
            rclpy.spin_once(self, timeout_sec=0.05)
        return self._dernier_scan

    def ecrire_carte_vide(self):
        """Écrit une carte entièrement libre couvrant largement départ et but."""
        reglages = self._configuration.navigation
        resolution = reglages.blank_map_resolution
        marge = reglages.blank_map_margin

        x_min = min(self._task.spawn.x, self._task.goal_x) - marge
        y_min = min(self._task.spawn.y, self._task.goal_y) - marge
        x_max = max(self._task.spawn.x, self._task.goal_x) + marge
        y_max = max(self._task.spawn.y, self._task.goal_y) + marge

        largeur = int(math.ceil((x_max - x_min) / resolution))
        hauteur = int(math.ceil((y_max - y_min) / resolution))

        # mkdtemp crée un dossier temporaire unique : deux exécutions en
        # parallèle ne s'écrasent pas l'une l'autre.
        dossier = tempfile.mkdtemp(prefix='fosa_map_')
        with open(os.path.join(dossier, 'blank_map.pgm'), 'wb') as flux:
            # Format PGM « P5 » : un en-tête en texte, puis un octet par pixel.
            flux.write(f'P5\n{largeur} {hauteur}\n255\n'.encode('ascii'))
            # 254 = presque blanc = case libre.
            flux.write(bytes([254]) * (largeur * hauteur))

        chemin_yaml = os.path.join(dossier, 'blank_map.yaml')
        with open(chemin_yaml, 'w', encoding='utf-8') as flux:
            flux.write(
                'image: blank_map.pgm\nmode: trinary\n'
                f'resolution: {resolution}\n'
                f'origin: [{x_min:.3f}, {y_min:.3f}, 0.0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        return chemin_yaml

    # ---------------------------------------------------------------- bringup
    def demarrer_bringup(self, chemin_carte):
        if not self.get_parameter('launch_bringup').value:
            self.get_logger().info(
                'launch_bringup=false : le bringup doit déjà tourner.')
            return

        self._bringup.start(
            map_yaml=chemin_carte,
            use_sim_time=self.get_parameter('use_sim_time').value,
            params_file=self.get_parameter('nav2_params_file').value,
            controller=self._controleur,
            localization=self._localisation,
            profile=self._profil,
            task_params_path=self._task.path,
            spawn=self._task.spawn,
            bringup_package=self.get_parameter('bringup_package').value,
            bringup_launch=self.get_parameter('bringup_launch').value,
        )

    def arreter_bringup(self):
        self._bringup.stop()

    def arreter_robot(self):
        """Envoie plusieurs commandes nulles : le robot reste où il est.

        Indispensable : le plugin DiffDrive de Gazebo garde la dernière vitesse
        reçue tant qu'on ne lui en envoie pas une autre.
        """
        if not rclpy.ok():
            return
        repetitions = self._configuration.navigation.stop_repeat_count
        for _ in range(repetitions):
            self._publicateur_vitesse.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

    def sequence_arret(self):
        """Arrêt ordonné, quel que soit le motif (arrivée, échec, Ctrl-C)."""
        # Nav2 doit cesser de commander le robot avant qu'on l'arrête.
        self._navigateur.annuler_but()
        self.arreter_robot()
        self.arreter_bringup()
        # Dernière commande nulle une fois que plus rien ne peut en publier.
        self.arreter_robot()

    # ------------------------------------------------------------------- Nav2
    def attendre_horloge(self):
        """Démarre le chronomètre dès que l'horloge (simulée) est disponible."""
        delai = time.monotonic() + self._configuration.navigation.clock_wait_sec
        while rclpy.ok() and time.monotonic() < delai:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.get_clock().now().nanoseconds > 0:
                break

        if not self._chronometre.start():
            self.get_logger().warn(
                'Horloge /clock absente : la simulation tourne-t-elle ?')

    def attendre_nav2(self):
        """Attend que Nav2 accepte des buts et que le robot soit localisé."""
        return self._navigateur.attendre_pret(
            lambda: self._pose_amcl is not None,
            self._localisation,
            float(self.get_parameter('nav2_startup_timeout_sec').value))

    def est_arrive(self):
        return self._navigateur.est_arrive(
            self._task.goal_x, self._task.goal_y)

    def naviguer_une_fois(self):
        """Un envoi du but, avec réessais gérés par le navigateur."""
        return self._navigateur.naviguer_une_fois(
            self._task.goal_x, self._task.goal_y, self._repere_global,
            self._task.spawn)

    # ------------------------------------------------------------------- tâche
    def executer_mission(self):
        self.attendre_horloge()
        chemin_carte = self.choisir_carte()

        # Garder le scan pendant la mission : le recul est interdit si le
        # secteur arrière n'est pas mesuré et libre.
        self.demarrer_bringup(chemin_carte)

        if not self.attendre_nav2():
            return EXIT_FAILURE

        return self.boucle_navigation()

    def boucle_navigation(self):
        tentatives = 0
        refus = 0
        retries_sans_progres = 0
        reculs_tentes = 0

        while rclpy.ok():
            distance_avant = self.distance_vers_but()
            if self.est_arrive():
                resultat = 'succes'
            elif self.temps_restant() <= 0.0:
                resultat = 'temps_ecoule'
            else:
                if refus == 0:
                    tentatives += 1
                    distance = self.distance_vers_but()
                    self.get_logger().info(
                        f'Envoi du but, tentative {tentatives}'
                        + (f' ({distance:.2f} m à parcourir).'
                           if distance is not None else '.'))
                resultat = self.naviguer_une_fois()

            if resultat == 'refuse':
                # Nav2 existe mais n'est pas encore actif : on insiste, sans
                # toucher aux costmaps ni compter une tentative.
                if refus % 10 == 0:
                    self.get_logger().info(
                        "Nav2 n'accepte pas encore de but, nouvel essai...")
                refus += 1
                self.traiter_messages_pendant(0.5)
                continue
            refus = 0

            if resultat == 'succes':
                self.arreter_robot()
                distance = self.distance_vers_but()
                self.get_logger().info(
                    f'BUT ATTEINT en {self.temps_ecoule():.1f} s, '
                    f'{tentatives} tentative(s)'
                    + (f', centre du robot à {distance:.2f} m du centre du but.'
                       if distance is not None else '.'))
                return EXIT_SUCCESS

            if resultat == 'temps_ecoule':
                self.arreter_robot()
                distance = self.distance_vers_but()
                self.get_logger().error(
                    'Limite de temps atteinte'
                    + (f' à {distance:.2f} m du but.'
                       if distance is not None else '.'))
                return EXIT_TIMEOUT

            if resultat == 'arret':
                self.get_logger().error(
                    "Le bringup s'est arrêté : navigation impossible.")
                return EXIT_FAILURE

            # Conserver les observations dans les costmaps et renvoyer la même
            # destination. Une absence de progrès déclenche une période
            # d'observation, pas une boucle de manœuvres répétées.
            distance_apres = self.distance_vers_but()
            navigation = self._configuration.navigation
            if distance_avant is not None and distance_apres is not None:
                progres = distance_avant - distance_apres
                if progres >= navigation.obstacle_min_progress_m:
                    retries_sans_progres = 0
                else:
                    retries_sans_progres += 1

            if retries_sans_progres >= navigation.obstacle_retries_before_retreat:
                recul_effectue = False
                if (navigation.obstacle_retreat_enabled
                        and reculs_tentes < navigation.obstacle_max_retreat_attempts):
                    autorise, succes_recul = self._recul_surveille()
                    if autorise:
                        reculs_tentes += 1
                        recul_effectue = succes_recul
                        if succes_recul:
                            retries_sans_progres = 0
                if not recul_effectue:
                    self.get_logger().warning(
                        'Pas de progrès : robot immobile en observation, '
                        'destination conservée et nouvelle planification à venir.')
                self.traiter_messages_pendant(
                    navigation.obstacle_observation_sec)
            else:
                self.traiter_messages_pendant(
                    float(self.get_parameter('retry_pause_sec').value))

        return EXIT_FAILURE


def _transformer_ctrl_c_en_interruption(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    # On garde la main sur Ctrl-C et SIGTERM : rclpy fermerait sinon son contexte
    # avant qu'on ait pu envoyer une vitesse nulle au robot.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _transformer_ctrl_c_en_interruption)
    signal.signal(signal.SIGTERM, _transformer_ctrl_c_en_interruption)

    # La configuration est chargée avant le nœud : une erreur de YAML doit
    # arrêter le programme avec un message clair, pas un traceback ROS.
    try:
        configuration = load_config()
    except ConfigurationError as erreur:
        print(f'task_solution: configuration invalide : {erreur}', file=sys.stderr)
        sys.exit(EXIT_FAILURE)

    # La tâche tourne dans Gazebo : temps simulé par défaut, sans option à passer.
    # Une valeur donnée en ligne de commande reste prioritaire.
    overrides = []
    if not any('use_sim_time' in argument for argument in sys.argv):
        overrides.append(Parameter(
            'use_sim_time', Parameter.Type.BOOL,
            configuration.system.use_sim_time))

    noeud = None
    code = EXIT_FAILURE
    try:
        noeud = SolutionTache(
            parameter_overrides=overrides, configuration=configuration)
        code = noeud.executer_mission()
    except KeyboardInterrupt:
        pass
    except Exception as erreur:
        # On attrape tout pour toujours pouvoir arrêter le robot.
        if noeud is not None:
            noeud.get_logger().error(f'Erreur inattendue : {erreur!r}')
        else:
            print(
                f'task_solution: erreur au démarrage : {erreur!r}',
                file=sys.stderr)
    finally:
        if noeud is not None:
            # Un second Ctrl-C pendant le nettoyage ne doit pas l'interrompre.
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            try:
                noeud.sequence_arret()
            except Exception as erreur:
                print(
                    f"task_solution: erreur pendant l'arrêt : {erreur!r}",
                    file=sys.stderr)
                noeud.arreter_bringup()
            noeud.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(code)


if __name__ == '__main__':
    main()
