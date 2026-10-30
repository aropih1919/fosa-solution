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
"""

import math
import os
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from caytu_nav_solution.calculs_nav import (
    Pose2D,
    accord_scan_carte,
    charger_carte_pgm,
    lacet_depuis_quaternion,
    quaternion_depuis_lacet,
)
from caytu_nav_solution.parametres_tache import (
    ParametresTache,
    charger_parametres_tache,
)

# Code de sortie du programme : le shell et le jury peuvent le lire.
# 0 = but atteint ; 1 = échec ; 2 = temps écoulé.
EXIT_SUCCESS, EXIT_FAILURE, EXIT_TIMEOUT = 0, 1, 2

# Position du lidar dans base_footprint (URDF officiel) : 2,1 cm derrière le centre.
DECALAGE_LIDAR_X = -0.021


class SolutionTache(Node):

    def __init__(self, parameter_overrides=None):
        super().__init__('task_solution', parameter_overrides=parameter_overrides or [])

        self._declarer_parametres()
        self._lire_parametres()
        self._initialiser_etat()
        self._connecter_ros()

        self.get_logger().info(
            f'Départ ({self._parametres.depart.x:.3f}, {self._parametres.depart.y:.3f}, '
            f'cap {self._parametres.depart.yaw:.3f}) -> but ({self._parametres.but_x:.3f}, '
            f'{self._parametres.but_y:.3f}), lus dans {self._parametres.chemin}')

    # ------------------------------------------------------------ réglages
    def _declarer_parametres(self):
        # Durée officielle de la tâche et marge gardée pour s'arrêter proprement.
        self.declare_parameter('time_limit_sec', 600.0)
        self.declare_parameter('stop_margin_sec', 1.0)

        # Le robot est considéré arrivé si son centre est à moins de ce rayon du
        # centre du but, même si Nav2 ne l'a pas encore annoncé (cercle : 0,6 m).
        self.declare_parameter('success_radius', 0.20)

        # Garde-fou : si Nav2 annonce l'arrivée alors que le robot est encore à
        # plus de cette distance du centre du but, le but est renvoyé.
        self.declare_parameter('arrival_recheck_radius', 0.35)
        self.declare_parameter('retry_pause_sec', 1.0)
        self.declare_parameter('nav2_startup_timeout_sec', 180.0)   # temps réel

        # Lancement du reste de la solution par ce nœud (exigence PARC).
        self.declare_parameter('launch_bringup', True)
        self.declare_parameter('bringup_package', 'caytu_nav_bringup')
        self.declare_parameter('bringup_launch', 'solution_bringup.launch.py')
        self.declare_parameter('controller', 'rpp')                 # rpp | dwb

        # Carte des murs : auto (vérifiée avec le lidar), always, never.
        # (Pas « on/off » : YAML les lirait comme des booléens.)
        self.declare_parameter('static_map', 'auto')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('map_check_timeout_sec', 4.0)        # temps réel
        self.declare_parameter('map_check_min_agreement', 0.35)

        self.declare_parameter('task_params_file', '')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter(
            'cmd_vel_topic', '/robot_base_controller/cmd_vel_unstamped')

    def _lire_parametres(self):
        # Lecture immédiate du fichier de départ/but : si le fichier manque, on
        # s'arrête tout de suite avec un message clair.
        self._parametres = charger_parametres_tache(
            self.get_parameter('task_params_file').value or None)
        self._repere_global = self.get_parameter('global_frame').value
        self._repere_robot = self.get_parameter('base_frame').value

    def _initialiser_etat(self):
        self._processus_bringup = None      # le programme `ros2 launch` lancé
        self._instant_depart = None         # instant de départ (horloge ROS)
        self._dernier_scan = None           # sert uniquement à vérifier la carte
        self._dernier_log_feedback = 0.0    # date du dernier message de progression
        self._but_en_cours = None           # but Nav2 en cours, pour l'annuler

    def _connecter_ros(self):
        # Buffer = mémoire des TF ; TransformListener = l'abonnement qui la remplit.
        # spin_thread=False : on traite les messages nous-mêmes dans nos boucles.
        self._memoire_tf = Buffer()
        self._ecouteur_tf = TransformListener(
            self._memoire_tf, self, spin_thread=False)

        # Client de l'action « va à cette position » de Nav2.
        self._client_nav2 = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Canal pour envoyer une vitesse au robot (avancer / tourner).
        self._publicateur_vitesse = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)

        # Deux services pour vider les costmaps entre deux tentatives.
        self._clients_vidage_costmap = [
            self.create_client(
                ClearEntireCostmap,
                '/global_costmap/clear_entirely_global_costmap'),
            self.create_client(
                ClearEntireCostmap,
                '/local_costmap/clear_entirely_local_costmap'),
        ]

        # On s'abonne à /scan uniquement pour vérifier la carte au démarrage.
        self._abonnement_scan = self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self.recevoir_scan, qos_profile_sensor_data)

    # ------------------------------------------------------------- utilitaires
    def recevoir_scan(self, message):
        # On garde seulement le dernier scan : le contrôle de carte n'a besoin
        # que d'une photo à un instant donné, pas de l'historique.
        self._dernier_scan = message

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
        if self._instant_depart is None:
            return 0.0
        difference = self.get_clock().now() - self._instant_depart
        return difference.nanoseconds * 1e-9

    def temps_restant(self):
        """Temps restant avant la limite officielle, marge de sécurité déduite."""
        return (float(self.get_parameter('time_limit_sec').value)
                - float(self.get_parameter('stop_margin_sec').value)
                - self.temps_ecoule())

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
        return pose.distance_vers(self._parametres.but_x, self._parametres.but_y)

    def bringup_est_vivant(self):
        # poll() renvoie None tant que le programme tourne, un code quand il est
        # terminé. Si aucun programme n'a été lancé (launch_bringup=false), on
        # considère qu'il est « vivant » pour ne pas bloquer la mission.
        return (self._processus_bringup is None
                or self._processus_bringup.poll() is None)

    # ------------------------------------------------------------------ carte
    def choisir_carte(self):
        """Retourne le YAML de carte à donner à Nav2 (murs réels ou carte vide)."""
        mode = str(self.get_parameter('static_map').value).lower()
        chemin_carte = self.get_parameter('map_yaml').value

        if not chemin_carte:
            # Aucun chemin fourni : on prend la carte livrée avec le paquet.
            from ament_index_python.packages import get_package_share_directory
            chemin_carte = os.path.join(
                get_package_share_directory(
                    self.get_parameter('bringup_package').value),
                'maps', 'cafe_map.yaml')

        if mode == 'never':
            self.get_logger().info('Carte des murs désactivée (static_map=never).')
            return self.ecrire_carte_vide()

        if mode == 'always':
            self.get_logger().info('Carte des murs imposée (static_map=always).')
            return chemin_carte

        # Mode auto : la carte décrit le café officiel. Si le monde chargé est
        # différent, elle gênerait le planificateur ; on le vérifie en comparant
        # un scan lidar pris au point de départ avec les murs de la carte.
        try:
            carte = self.charger_carte(chemin_carte)
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as erreur:
            self.get_logger().warn(
                f'Carte illisible ({erreur}) : navigation sans carte.')
            return self.ecrire_carte_vide()

        scan = self.attendre_premier_scan()
        if scan is None:
            # Pas de lidar : on ne peut rien vérifier. On garde la carte du
            # café, car c'est le cas normal de l'évaluation.
            self.get_logger().warn(
                'Pas de scan reçu pour vérifier la carte : carte du café conservée.')
            return chemin_carte

        # La position du lidar n'est pas exactement celle du robot : il est
        # décalé de DECALAGE_LIDAR_X.
        pose_lidar = self._parametres.depart.composer(
            Pose2D(DECALAGE_LIDAR_X, 0.0, 0.0))
        accord, utilises = accord_scan_carte(
            carte, pose_lidar, scan.angle_min, scan.angle_increment, scan.ranges)

        seuil = float(self.get_parameter('map_check_min_agreement').value)
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
        resolution = 0.05          # taille d'une case en mètres (5 cm)
        marge = 15.0               # 15 m de marge autour du départ et du but

        x_min = min(self._parametres.depart.x, self._parametres.but_x) - marge
        y_min = min(self._parametres.depart.y, self._parametres.but_y) - marge
        x_max = max(self._parametres.depart.x, self._parametres.but_x) + marge
        y_max = max(self._parametres.depart.y, self._parametres.but_y) + marge

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

        utiliser_temps_simulation = self.get_parameter('use_sim_time').value
        # On construit une LISTE de morceaux de commande, et non une seule
        # grande chaîne : une liste ne passe pas par le shell, donc un caractère
        # spécial dans un chemin ne peut pas être interprété comme une commande.
        commande = [
            'ros2', 'launch',
            self.get_parameter('bringup_package').value,
            self.get_parameter('bringup_launch').value,
            f'use_sim_time:={"true" if utiliser_temps_simulation else "false"}',
            f'map:={chemin_carte}',
            f'controller:={self.get_parameter("controller").value}',
            f'task_params_file:={self._parametres.chemin}',
        ]
        self.get_logger().info('Démarrage de la solution : ' + ' '.join(commande))

        # Groupe de processus dédié : permet d'arrêter tout le bringup d'un coup.
        self._processus_bringup = subprocess.Popen(
            commande, start_new_session=True)

    def arreter_bringup(self):
        if (self._processus_bringup is None
                or self._processus_bringup.poll() is not None):
            return

        self.get_logger().info('Arrêt du bringup...')
        # On essaie d'abord un arrêt propre, puis plus fort si nécessaire.
        for signal_envoye, attente in (
                (signal.SIGINT, 10.0),
                (signal.SIGTERM, 5.0),
                (signal.SIGKILL, 2.0)):
            try:
                os.killpg(
                    os.getpgid(self._processus_bringup.pid), signal_envoye)
            except ProcessLookupError:
                return
            try:
                self._processus_bringup.wait(timeout=attente)
                return
            except subprocess.TimeoutExpired:
                continue

    def arreter_robot(self):
        """Envoie plusieurs commandes nulles : le robot reste où il est.

        Indispensable : le plugin DiffDrive de Gazebo garde la dernière vitesse
        reçue tant qu'on ne lui en envoie pas une autre.
        """
        if not rclpy.ok():
            return
        for _ in range(10):
            self._publicateur_vitesse.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

    def sequence_arret(self):
        """Arrêt ordonné, quel que soit le motif (arrivée, échec, Ctrl-C)."""
        if rclpy.ok() and self._but_en_cours is not None:
            # Nav2 doit cesser de commander le robot avant qu'on l'arrête.
            future_annulation = self._but_en_cours.cancel_goal_async()
            fin = time.monotonic() + 1.0
            while (rclpy.ok() and not future_annulation.done()
                   and time.monotonic() < fin):
                rclpy.spin_once(self, timeout_sec=0.05)
            self._but_en_cours = None

        self.arreter_robot()
        self.arreter_bringup()
        # Dernière commande nulle une fois que plus rien ne peut en publier.
        self.arreter_robot()

    # ------------------------------------------------------------------- Nav2
    def attendre_horloge(self):
        """Démarre le chronomètre dès que l'horloge (simulée) est disponible."""
        delai = time.monotonic() + 10.0
        while rclpy.ok() and time.monotonic() < delai:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.get_clock().now().nanoseconds > 0:
                break

        self._instant_depart = self.get_clock().now()
        if self._instant_depart.nanoseconds == 0:
            self.get_logger().warn(
                'Horloge /clock absente : la simulation tourne-t-elle ?')

    def attendre_nav2(self):
        """Attend que Nav2 accepte des buts et que le robot soit localisé."""
        delai = float(self.get_parameter('nav2_startup_timeout_sec').value)
        debut = time.monotonic()
        dernier_log = debut

        while rclpy.ok():
            self.traiter_messages_pendant(0.2)
            serveur_pret = self._client_nav2.server_is_ready()
            robot_localise = self.pose_robot() is not None

            if serveur_pret and robot_localise:
                self.get_logger().info(
                    f'Nav2 prêt et robot localisé après '
                    f'{time.monotonic() - debut:.1f} s réelles.')
                return True

            if not self.bringup_est_vivant():
                self.get_logger().error(
                    'Le bringup s\'est arrêté pendant le démarrage.')
                return False

            if (self.temps_restant() <= 0.0
                    or time.monotonic() - debut > delai):
                self.get_logger().error(
                    'Nav2 n\'est pas devenu disponible à temps.')
                return False

            if time.monotonic() - dernier_log > 5.0:
                dernier_log = time.monotonic()
                self.get_logger().info(
                    f'Attente de Nav2 (serveur d\'action : '
                    f'{"oui" if serveur_pret else "non"}, '
                    f'localisation : {"oui" if robot_localise else "non"})...')

        return False

    def message_but(self):
        pose = self.pose_robot() or self._parametres.depart
        cap = math.atan2(
            self._parametres.but_y - pose.y,
            self._parametres.but_x - pose.x)

        but = PoseStamped()
        but.header.frame_id = self._repere_global
        but.header.stamp = self.get_clock().now().to_msg()
        but.pose.position.x = self._parametres.but_x
        but.pose.position.y = self._parametres.but_y

        qx, qy, qz, qw = quaternion_depuis_lacet(cap)
        but.pose.orientation.x = qx
        but.pose.orientation.y = qy
        but.pose.orientation.z = qz
        but.pose.orientation.w = qw

        message = NavigateToPose.Goal()
        message.pose = but
        return message

    def recevoir_feedback(self, message_feedback):
        maintenant = time.monotonic()
        if maintenant - self._dernier_log_feedback < 5.0:
            return
        self._dernier_log_feedback = maintenant

        distance_restante = getattr(
            message_feedback.feedback, 'distance_remaining', float('nan'))
        self.get_logger().info(
            f'En route : {distance_restante:.2f} m restants, '
            f'{self.temps_ecoule():.0f} s écoulées.')

    def vider_costmaps(self):
        for client in self._clients_vidage_costmap:
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())

    def est_arrive(self):
        distance = self.distance_vers_but()
        return (distance is not None
                and distance <= float(
                    self.get_parameter('success_radius').value))

    def attendre_future(self, future):
        """Traite les messages ROS jusqu'à ce que `future` soit terminé.

        Retourne :
          - 'ok'           : la future est terminée (ou rclpy s'arrête) ;
          - 'temps_ecoule' : la limite de temps est atteinte ;
          - 'arret'        : le bringup s'est arrêté.
        """
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.temps_restant() <= 0.0:
                return 'temps_ecoule'
            if not self.bringup_est_vivant():
                return 'arret'
        return 'ok'

    def annuler_but(self, but):
        """Annule le but en cours et attend au maximum 2 secondes."""
        future_annulation = but.cancel_goal_async()
        fin = time.monotonic() + 2.0
        while (rclpy.ok() and not future_annulation.done()
               and time.monotonic() < fin):
            rclpy.spin_once(self, timeout_sec=0.05)

    def naviguer_une_fois(self):
        """Un envoi du but.

        Résultats possibles :
          - 'succes'       : but atteint ;
          - 'reessayer'    : échec de Nav2, on renvoie le but ;
          - 'refuse'       : Nav2 n'est pas encore actif, on insiste ;
          - 'temps_ecoule' : limite de temps atteinte ;
          - 'arret'        : le bringup s'est arrêté.
        """
        future_envoi = self._client_nav2.send_goal_async(
            self.message_but(), feedback_callback=self.recevoir_feedback)
        resultat = self.attendre_future(future_envoi)
        if resultat != 'ok':
            return resultat

        but = future_envoi.result() if future_envoi.done() else None
        if but is None or not but.accepted:
            return 'refuse'
        self._but_en_cours = but

        future_resultat = but.get_result_async()
        resultat = self.attendre_future(future_resultat)
        if resultat == 'temps_ecoule':
            self.get_logger().error(
                'Limite de temps atteinte : annulation du but.')
            self.annuler_but(but)
            return 'temps_ecoule'
        if resultat == 'arret':
            return 'arret'

        self._but_en_cours = None
        resultat_final = (future_resultat.result()
                          if future_resultat.done() else None)
        statut = (resultat_final.status if resultat_final is not None
                  else GoalStatus.STATUS_UNKNOWN)

        if statut == GoalStatus.STATUS_SUCCEEDED:
            distance = self.distance_vers_but()
            limite = float(self.get_parameter('arrival_recheck_radius').value)
            if distance is not None and distance > limite:
                self.get_logger().warn(
                    f'Nav2 annonce l\'arrivée mais le robot est à '
                    f'{distance:.2f} m du but : nouvel envoi.')
                return 'reessayer'
            return 'succes'

        if self.est_arrive():
            return 'succes'

        self.get_logger().warn(
            f'Nav2 a interrompu la navigation (statut {statut}).')
        return 'reessayer'

    # ------------------------------------------------------------------- tâche
    def executer_mission(self):
        self.attendre_horloge()
        chemin_carte = self.choisir_carte()

        # Le scan ne sert plus à rien une fois la carte choisie.
        self.destroy_subscription(self._abonnement_scan)
        self.demarrer_bringup(chemin_carte)

        if not self.attendre_nav2():
            return EXIT_FAILURE

        return self.boucle_navigation()

    def boucle_navigation(self):
        tentatives = 0
        refus = 0

        while rclpy.ok():
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
                        'Nav2 n\'accepte pas encore de but, nouvel essai...')
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
                    'Le bringup s\'est arrêté : navigation impossible.')
                return EXIT_FAILURE

            # Échec récupérable : on repart avec des costmaps propres.
            self.vider_costmaps()
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

    # La tâche tourne dans Gazebo : temps simulé par défaut, sans option à passer.
    # Une valeur donnée en ligne de commande reste prioritaire.
    overrides = []
    if not any('use_sim_time' in argument for argument in sys.argv):
        overrides.append(Parameter('use_sim_time', Parameter.Type.BOOL, True))

    noeud = None
    code = EXIT_FAILURE
    try:
        noeud = SolutionTache(parameter_overrides=overrides)
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
                    f'task_solution: erreur pendant l\'arrêt : {erreur!r}',
                    file=sys.stderr)
                noeud.arreter_bringup()
            noeud.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(code)


if __name__ == '__main__':
    main()
