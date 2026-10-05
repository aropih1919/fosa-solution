#!/usr/bin/env python3
"""Benchmark d'un run de navigation PARC 2026 (moniteur passif).

Ce script ne pilote PAS le robot. Il écoute ce qui se passe pendant un run et
mesure les 3 critères officiels :
  1. contacts (évitement d'obstacles)
  2. distance finale entre le centre du robot et le but
  3. durée d'exécution (en temps SIMULÉ, pas en temps réel)

Il ajoute des diagnostics : recoveries, statut Nav2, covariance d'AMCL, écart
entre la position estimée par AMCL et la vraie position Gazebo, gel du simulateur.

La position utilisée pour juger l'arrivée est la VRAIE pose Gazebo (topic
/sitoe_robot/pose), comme le fera l'évaluation officielle. Sans ce topic, on
retombe sur AMCL (moins précis).

Utilisation (terminal où le workspace est sourcé) :
    python3 tools/benchmark.py --label baseline_sans_camera
    python3 tools/benchmark.py --label test_rapide --stop-at 120   # run court

Le résultat est écrit dans tools/results/run_<date>_<label>.json
"""

import argparse
import json
import math
import os
import subprocess
import time
import xml.etree.ElementTree as ET
from datetime import datetime

import tf2_ros
import yaml
import rclpy
from action_msgs.msg import GoalStatusArray
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Bool

import bench_core as core

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Rayon utilisé si on ne peut pas le lire dans le modèle du but (à vérifier !)
DEFAULT_GOAL_RADIUS = 0.5

# Nombre maximum de points gardés dans la courbe de covariance du JSON
MAX_SERIES_POINTS = 1500


# --------------------------------------------------------------------------
# Lecture de la configuration de la tâche
# --------------------------------------------------------------------------

def load_task_params():
    """Lit goal_x, goal_y et la pose de départ dans task_params.yaml."""
    path = os.path.join(
        get_package_share_directory('parc_robot_bringup'), 'config', 'task_params.yaml'
    )
    with open(path, encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    return data['/**']['ros__parameters']


def read_goal_radius():
    """Cherche le rayon du cercle vert dans models/goal_location/model.sdf.

    Retourne (rayon, source). Le rayon est None si le fichier est introuvable.
    """
    candidates = []
    try:
        share = get_package_share_directory('parc_robot_bringup')
        candidates.append(os.path.join(share, 'models', 'goal_location', 'model.sdf'))
    except PackageNotFoundError:
        pass
    # Dépôt source : fosa-solution/ et parc_robot_bringup/ sont côte à côte
    candidates.append(os.path.join(
        SCRIPT_DIR, '..', '..', 'parc_robot_bringup', 'models', 'goal_location', 'model.sdf'
    ))

    for path in candidates:
        if not os.path.isfile(path):
            continue
        try:
            tree = ET.parse(path)
            for element in tree.iter('radius'):
                return float(element.text), path
        except (ET.ParseError, ValueError):
            continue
    return None, None


def get_git_commit():
    """Commit courant, avec '-dirty' si des fichiers suivis sont modifiés."""
    try:
        commit = subprocess.run(
            ['git', 'rev-parse', '--short', 'HEAD'],
            cwd=SCRIPT_DIR, capture_output=True, text=True, timeout=5,
        ).stdout.strip()
        changes = subprocess.run(
            ['git', 'status', '--porcelain', '--untracked-files=no'],
            cwd=SCRIPT_DIR, capture_output=True, text=True, timeout=5,
        ).stdout.strip()
        if not commit:
            return 'inconnu'
        return commit + ('-dirty' if changes else '')
    except Exception:
        return 'inconnu'


# --------------------------------------------------------------------------
# Le noeud de mesure
# --------------------------------------------------------------------------

class BenchmarkNode(Node):

    def __init__(self, args, goal_x, goal_y, goal_radius, goal_radius_source):
        super().__init__(
            'benchmark_monitor',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self.args = args
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_radius = goal_radius
        self.goal_radius_source = goal_radius_source
        self.ignore_keywords = [k.strip().lower() for k in args.ignore_keywords.split(',')]

        # Temps
        self.wall_start = time.time()
        self.sim_start = None
        self.last_sim = None
        self.last_sim_change_wall = time.time()
        self.max_freeze_wall = 0.0
        self.last_print_wall = 0.0
        self.last_print_sim = None

        # Positions : la vraie (Gazebo), celle d'AMCL (TF), et celle retenue
        self.truth_pose = None
        self.amcl_pose = None
        self.pose = None
        self.pose_source_used = None
        self.first_pose = None
        self.last_path_pose = None
        self.path_length = 0.0
        self.center_distance = None
        self.edge_distance = None
        self.min_center_distance = None
        self.t_first_motion = None

        # Covariance d'AMCL et erreur de localisation (AMCL contre vraie pose)
        self.cov_first = None
        self.cov_last = None
        self.cov_max = [0.0, 0.0, 0.0]
        self.cov_samples = 0
        self.cov_series = []
        self.err_first = None
        self.err_last = None            # (xy, yaw)
        self.err_max_xy = 0.0
        self.err_sum_xy = 0.0
        self.err_count = 0
        self.err_warned = False
        self.at_first_loss = None

        # Nav2
        self.goal_ids = set()
        self.t_first_goal = None
        self.last_status = None
        self.status_history = []
        self.terminal_since = None
        self.recoveries = 0
        self.distance_remaining = None

        # Localisation (/localization_ready)
        self.loc_ready = None
        self.loc_ready_at = None
        self.loc_loss_count = 0

        # Contacts
        self.contact_subs = {}
        self.contacts_total = core.ContactCounter(args.contact_gap)
        self.contacts_by_sensor = {}
        self.contact_events = []
        self.contact_pairs_seen = set()

        # Découverte des topics
        self.truth_sub = None
        self.last_discovery_wall = 0.0
        self.discovery_done = False

        # Fin du run
        self.finished = False
        self.report = None
        self.report_path = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._setup_subscriptions()

        # Timer en temps MUR : il continue de tourner même si Gazebo gèle
        self.create_timer(0.2, self.on_tick, clock=Clock())

    # ---------------- abonnements ----------------

    def _setup_subscriptions(self):
        # Le topic de statut de l'action est "transient local" : on reçoit le dernier état
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', self.on_status, status_qos
        )
        self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            '/navigate_to_pose/_action/feedback',
            self.on_feedback,
            10,
        )
        self.create_subscription(Bool, '/localization_ready', self.on_localization, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl_pose, 10)

    def discover_topics(self):
        """Cherche la vraie pose et les capteurs de contact (topics non connus à l'avance)."""
        if self.discovery_done:
            return
        now = time.time()
        if now - self.last_discovery_wall < 2.0:
            return
        self.last_discovery_wall = now

        graph = self.get_topic_names_and_types()
        self._discover_truth(graph)
        self._discover_contacts(graph)

        # On arrête de chercher après 20 s
        if now - self.wall_start > 20.0:
            self.discovery_done = True
            if self.truth_sub is None:
                self.get_logger().warn(
                    f'Topic de vraie pose {self.args.truth_topic} introuvable : '
                    'la position viendra d\'AMCL (moins précis).'
                )
            if not self.contact_subs:
                self.get_logger().warn(
                    'AUCUN topic de collision trouvé : les contacts ne seront pas mesurés.'
                )

    def _discover_truth(self, graph):
        if self.truth_sub is not None:
            return
        for name, types in graph:
            if name != self.args.truth_topic or not types:
                continue
            try:
                msg_class = get_message(types[0])
            except Exception as error:
                self.get_logger().warn(f'Type inconnu pour {name} ({types[0]}) : {error}')
                return
            self.truth_sub = self.create_subscription(msg_class, name, self.on_truth, 10)
            self.get_logger().info(f'Vraie pose Gazebo : {name} ({types[0]})')
            return

    def _discover_contacts(self, graph):
        for name, types in graph:
            if 'collision' not in name or name in self.contact_subs or not types:
                continue
            try:
                msg_class = get_message(types[0])
            except Exception as error:
                self.get_logger().warn(f'Type inconnu pour {name} ({types[0]}) : {error}')
                continue
            self.contact_subs[name] = self.create_subscription(
                msg_class, name, lambda msg, topic=name: self.on_contacts(topic, msg), 10
            )
            self.contacts_by_sensor[name] = core.ContactCounter(self.args.contact_gap)
            self.get_logger().info(f'Capteur de contact trouvé : {name} ({types[0]})')

    # ---------------- callbacks ----------------

    def sim_now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def elapsed_now(self):
        if self.sim_start is None:
            return None
        return self.sim_now() - self.sim_start

    def on_truth(self, msg):
        pose = core.extract_pose(msg)
        if pose is not None:
            self.truth_pose = pose

    def on_amcl_pose(self, msg):
        elapsed = self.elapsed_now()
        if elapsed is None or self.finished:
            return

        cov = msg.pose.covariance
        values = [cov[0], cov[7], cov[35]]   # xx, yy, yaw
        self.cov_samples += 1
        self.cov_last = values
        if self.cov_first is None:
            self.cov_first = list(values)
        for i in range(3):
            self.cov_max[i] = max(self.cov_max[i], values[i])

        if len(self.cov_series) < MAX_SERIES_POINTS:
            err_xy = None if self.err_last is None else round(self.err_last[0], 3)
            self.cov_series.append([
                round(elapsed, 1), round(values[0], 4), round(values[1], 4),
                round(values[2], 4), err_xy,
            ])

    def on_status(self, msg):
        if not msg.status_list:
            return

        for status in msg.status_list:
            goal_id = bytes(status.goal_info.goal_id.uuid).hex()
            if goal_id not in self.goal_ids:
                self.goal_ids.add(goal_id)
                if self.t_first_goal is None:
                    self.t_first_goal = self.elapsed_now()

        last = msg.status_list[-1].status
        if last != self.last_status:
            self.last_status = last
            self.status_history.append({
                't': self._round(self.elapsed_now()),
                'status': core.STATUS_NAMES.get(last, str(last)),
            })
            self.get_logger().info(
                f'Nav2 : statut du goal = {core.STATUS_NAMES.get(last, last)}'
            )

    def on_feedback(self, msg):
        feedback = msg.feedback
        self.recoveries = max(self.recoveries, feedback.number_of_recoveries)
        self.distance_remaining = feedback.distance_remaining

    def on_localization(self, msg):
        if self.loc_ready is not None and self.loc_ready and not msg.data:
            self.loc_loss_count += 1
            self.get_logger().warn('Localisation perdue (/localization_ready est passé à false).')
            if self.at_first_loss is None:
                self.at_first_loss = {
                    't': self._round(self.elapsed_now()),
                    'cov_xx_yy_yaw': [round(v, 3) for v in self.cov_last] if self.cov_last else None,
                    'loc_error_xy': self._round(self.err_last[0]) if self.err_last else None,
                    'loc_error_yaw': self._round(self.err_last[1]) if self.err_last else None,
                }
                self.get_logger().warn(f'État au moment de la perte : {self.at_first_loss}')
        if msg.data and self.loc_ready_at is None:
            self.loc_ready_at = self._round(self.elapsed_now())
        self.loc_ready = msg.data

    def on_contacts(self, topic, msg):
        if self.sim_start is None or self.finished:
            return

        sim_now = self.sim_now()
        for contact in getattr(msg, 'contacts', []):
            name1 = getattr(getattr(contact, 'collision1', None), 'name', '')
            name2 = getattr(getattr(contact, 'collision2', None), 'name', '')
            real = core.is_real_contact(name1, name2, self.ignore_keywords)

            if self.args.debug_contacts and (name1, name2) not in self.contact_pairs_seen:
                self.contact_pairs_seen.add((name1, name2))
                verdict = 'COMPTÉ' if real else 'ignoré'
                print(f'[contact {verdict}] {topic} : {name1}  <->  {name2}')

            if not real:
                continue

            new_episode = self.contacts_total.add(sim_now)
            self.contacts_by_sensor[topic].add(sim_now)
            if new_episode and len(self.contact_events) < 30:
                self.contact_events.append({
                    't': self._round(sim_now - self.sim_start),
                    'sensor': topic,
                    'collision1': name1,
                    'collision2': name2,
                })
                self.get_logger().warn(
                    f'CONTACT #{self.contacts_total.episodes} : {name1} <-> {name2}'
                )

    # ---------------- boucle principale ----------------

    def on_tick(self):
        if self.finished:
            return

        sim_now = self.sim_now()
        wall_now = time.time()
        self.check_freeze(sim_now, wall_now)

        # Attendre que l'horloge de simulation démarre
        if self.sim_start is None:
            if sim_now > 0.0:
                self.sim_start = sim_now
                self.get_logger().info('Horloge simulée détectée : chronomètre démarré.')
            elif wall_now - self.wall_start > 120.0:
                self.finish('NO_CLOCK', 'Pas de /clock reçu : Gazebo est-il en lecture (Play) ?')
            return

        self.discover_topics()
        self.update_pose(sim_now)
        self.print_progress(sim_now, wall_now)
        self.check_end(sim_now, wall_now)

    def check_freeze(self, sim_now, wall_now):
        frozen_for = wall_now - self.last_sim_change_wall
        if self.sim_start is not None:
            self.max_freeze_wall = max(self.max_freeze_wall, frozen_for)
        if self.last_sim is None or sim_now != self.last_sim:
            self.last_sim = sim_now
            self.last_sim_change_wall = wall_now

    def read_amcl_pose(self):
        """Position estimée par AMCL : TF map -> base_footprint."""
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        t = transform.transform.translation
        q = transform.transform.rotation
        return (t.x, t.y, core.yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def update_pose(self, sim_now):
        self.amcl_pose = self.read_amcl_pose()
        mode = self.args.pose_source

        # Erreur de localisation : AMCL contre la vraie pose
        if self.amcl_pose is not None and self.truth_pose is not None:
            self.update_localization_error()

        # Choix de la position utilisée pour juger le run
        if mode in ('auto', 'truth') and self.truth_pose is not None:
            self.pose = self.truth_pose
            self.pose_source_used = 'truth'
        elif mode == 'auto' and self.truth_pose is None and not self.discovery_done:
            return   # on attend la vraie pose avant de se rabattre sur AMCL
        elif mode in ('auto', 'tf') and self.amcl_pose is not None:
            self.pose = self.amcl_pose
            self.pose_source_used = 'tf'
        else:
            return

        x, y, yaw = self.pose

        if self.first_pose is None:
            self.first_pose = (x, y)
            self.last_path_pose = (x, y)

        # Distance parcourue (on ignore les petits tremblements)
        step = math.hypot(x - self.last_path_pose[0], y - self.last_path_pose[1])
        if step > 0.01:
            self.path_length += step
            self.last_path_pose = (x, y)

        # Première mise en mouvement
        if self.t_first_motion is None:
            moved = math.hypot(x - self.first_pose[0], y - self.first_pose[1])
            if moved > 0.10:
                self.t_first_motion = sim_now - self.sim_start

        self.center_distance = math.hypot(self.goal_x - x, self.goal_y - y)
        self.edge_distance = core.distance_to_robot_body(self.goal_x, self.goal_y, x, y, yaw)
        if self.min_center_distance is None or self.center_distance < self.min_center_distance:
            self.min_center_distance = self.center_distance

    def update_localization_error(self):
        xy_error, yaw_error = core.pose_error(self.amcl_pose, self.truth_pose)
        self.err_last = (xy_error, yaw_error)
        if self.err_first is None:
            self.err_first = xy_error
        self.err_max_xy = max(self.err_max_xy, xy_error)
        self.err_sum_xy += xy_error
        self.err_count += 1

        # Un grand écart dès le début veut dire : repère map différent du monde Gazebo
        if not self.err_warned and self.err_first > 0.5:
            self.err_warned = True
            self.get_logger().warn(
                f'ATTENTION : AMCL est à {self.err_first:.2f} m de la vraie pose dès le départ. '
                'Le repère map est peut-être décalé par rapport au monde Gazebo '
                '(ou AMCL est mal initialisé).'
            )

    def check_end(self, sim_now, wall_now):
        elapsed = sim_now - self.sim_start

        # 1. Arrivée : une partie du robot est dans le cercle
        if self.edge_distance is not None and self.edge_distance <= self.goal_radius:
            self.finish('SUCCESS')
            return

        # 2. Run court demandé avec --stop-at
        if self.args.stop_at is not None and elapsed >= self.args.stop_at:
            self.finish('CHECKPOINT', f'Arrêt volontaire à {self.args.stop_at:.0f} s simulées.')
            return

        # 3. Temps écoulé
        if elapsed >= self.args.time_limit:
            self.finish('TIMEOUT')
            return

        # 4. Simulateur gelé ou en pause trop longtemps
        frozen_for = wall_now - self.last_sim_change_wall
        if frozen_for >= self.args.freeze_abort:
            self.finish(
                'SIM_FROZEN',
                f"L'horloge simulée n'avance plus depuis {frozen_for:.0f} s (temps réel) : "
                'Gazebo gelé ou en pause.',
            )
            return

        # 5. Pas de position du robot
        if self.pose is None and wall_now - self.wall_start > 60.0:
            self.finish('NO_POSE', 'Aucune position du robot reçue (vraie pose ou TF map -> base_footprint).')
            return

        # 6. Nav2 a terminé son goal alors que le robot n'est pas arrivé
        if self.args.stop_on_nav_end and self.last_status in core.TERMINAL_STATUSES:
            if self.terminal_since is None:
                self.terminal_since = sim_now
            elif sim_now - self.terminal_since >= self.args.nav_end_grace:
                name = core.STATUS_NAMES.get(self.last_status)
                self.finish(
                    'NAV_ENDED_EARLY',
                    f'Nav2 a terminé le goal (statut {name}) sans que le robot atteigne le cercle.',
                )
        else:
            self.terminal_since = None

    def print_progress(self, sim_now, wall_now):
        if wall_now - self.last_print_wall < 10.0:
            return

        # Facteur temps réel sur les 10 dernières secondes
        rtf_text = 'n/a'
        if self.last_print_sim is not None and wall_now > self.last_print_wall:
            rtf = (sim_now - self.last_print_sim) / (wall_now - self.last_print_wall)
            rtf_text = f'{rtf:.2f}'
        self.last_print_wall = wall_now
        self.last_print_sim = sim_now

        elapsed = sim_now - self.sim_start
        dist = 'n/a' if self.center_distance is None else f'{self.center_distance:.2f}m'
        cov = 'n/a' if self.cov_last is None else f'{max(self.cov_last[0], self.cov_last[1]):.2f}'
        err = 'n/a' if self.err_last is None else f'{self.err_last[0]:.2f}m'
        status = core.STATUS_NAMES.get(self.last_status, 'aucun goal')
        self.get_logger().info(
            f't_sim={elapsed:5.0f}s RTF={rtf_text} | but={dist} | cov_xy={cov} | '
            f'erreur AMCL={err} | contacts={self.contacts_total.episodes} | '
            f'recoveries={self.recoveries} | Nav2={status}'
        )

    # ---------------- fin du run ----------------

    @staticmethod
    def _round(value, digits=2):
        if value is None:
            return None
        return round(value, digits)

    def finish(self, result, note=''):
        if self.finished:
            return
        self.finished = True

        sim_end = self.last_sim if self.last_sim is not None else 0.0
        start = self.sim_start if self.sim_start is not None else sim_end
        elapsed = sim_end - start
        wall_elapsed = time.time() - self.wall_start

        start_distance = None
        progress = None
        if self.first_pose is not None:
            start_distance = math.hypot(
                self.goal_x - self.first_pose[0], self.goal_y - self.first_pose[1]
            )
            if self.center_distance is not None:
                progress = start_distance - self.center_distance

        time_since_goal = None
        if self.t_first_goal is not None:
            time_since_goal = elapsed - self.t_first_goal

        average_speed = None
        if elapsed > 0 and self.first_pose is not None:
            average_speed = self.path_length / elapsed

        cov_max_xy = None
        if self.cov_samples > 0:
            cov_max_xy = max(self.cov_max[0], self.cov_max[1])

        loc_error_available = self.err_count > 0
        loc_error_mean = self.err_sum_xy / self.err_count if loc_error_available else None

        self.report = {
            'label': self.args.label,
            'result': result,
            'note': note,
            'date': datetime.now().isoformat(timespec='seconds'),
            'git_commit': get_git_commit(),
            'time_sec': self._round(elapsed, 1),
            'time_since_first_goal_sec': self._round(time_since_goal, 1),
            'time_limit_sec': self.args.time_limit,
            'stop_at_sec': self.args.stop_at,
            'wall_seconds': self._round(wall_elapsed, 1),
            'real_time_factor': self._round(elapsed / wall_elapsed) if wall_elapsed > 0 else None,
            'goal': {
                'x': self.goal_x,
                'y': self.goal_y,
                'radius': self.goal_radius,
                'radius_source': self.goal_radius_source,
            },
            'pose_source_used': self.pose_source_used,
            'start_distance_to_goal': self._round(start_distance),
            'progress_m': self._round(progress),
            'final_center_distance': self._round(self.center_distance),
            'final_edge_distance': self._round(self.edge_distance),
            'min_center_distance': self._round(self.min_center_distance),
            'path_length_m': self._round(self.path_length),
            'average_speed_mps': self._round(average_speed),
            'contacts_available': bool(self.contact_subs),
            'contacts_episodes': self.contacts_total.episodes if self.contact_subs else None,
            'contacts_messages': self.contacts_total.messages,
            'contacts_by_sensor': {
                topic: counter.episodes for topic, counter in self.contacts_by_sensor.items()
            },
            'contact_events': self.contact_events,
            'recoveries': self.recoveries,
            'nav2': {
                'goals_seen': len(self.goal_ids),
                'last_status': core.STATUS_NAMES.get(self.last_status, 'aucun'),
                'status_history': self.status_history,
            },
            't_first_goal': self._round(self.t_first_goal),
            't_first_motion': self._round(self.t_first_motion),
            'localization': {
                'ready_at': self.loc_ready_at,
                'loss_count': self.loc_loss_count,
            },
            'at_first_loss': self.at_first_loss,
            'amcl_cov': {
                'samples': self.cov_samples,
                'first_xx_yy_yaw': [round(v, 3) for v in self.cov_first] if self.cov_first else None,
                'max_xx_yy_yaw': [round(v, 3) for v in self.cov_max] if self.cov_samples else None,
                'last_xx_yy_yaw': [round(v, 3) for v in self.cov_last] if self.cov_last else None,
            },
            'amcl_cov_max_xy': self._round(cov_max_xy, 3),
            'localization_error': {
                'available': loc_error_available,
                'first_xy': self._round(self.err_first),
                'max_xy': self._round(self.err_max_xy) if loc_error_available else None,
                'mean_xy': self._round(loc_error_mean),
                'final_xy': self._round(self.err_last[0]) if self.err_last else None,
                'final_yaw': self._round(self.err_last[1]) if self.err_last else None,
            },
            'loc_error_max_xy': self._round(self.err_max_xy) if loc_error_available else None,
            'sim_freeze': {
                'max_wall_seconds': self._round(self.max_freeze_wall, 1),
            },
            # Colonnes : t simulé, cov_xx, cov_yy, cov_yaw, erreur AMCL (m)
            'amcl_series': self.cov_series,
        }

        os.makedirs(self.args.output_dir, exist_ok=True)
        safe_label = ''.join(c if c.isalnum() or c in '-_.' else '_' for c in self.args.label)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.report_path = os.path.join(self.args.output_dir, f'run_{stamp}_{safe_label}.json')
        with open(self.report_path, 'w', encoding='utf-8') as handle:
            json.dump(self.report, handle, indent=2, ensure_ascii=False)

        self.print_report()

    def print_report(self):
        r = self.report
        print()
        print('=' * 62)
        print(f" RÉSULTAT : {r['result']}   (label : {r['label']})")
        if r['note']:
            print(f" Note     : {r['note']}")
        print('=' * 62)
        print(f" Temps (simulé)        : {r['time_sec']} s   (limite {r['time_limit_sec']} s)")
        print(f" Position utilisée     : {r['pose_source_used']}")
        print(f" Distance finale       : {r['final_center_distance']} m (centre -> but)")
        print(f" Progression           : {r['progress_m']} m   (départ à {r['start_distance_to_goal']} m)")
        print(f" Distance parcourue    : {r['path_length_m']} m   (vitesse moy. {r['average_speed_mps']} m/s)")
        if r['contacts_available']:
            print(f" Contacts (épisodes)   : {r['contacts_episodes']}")
        else:
            print(' Contacts (épisodes)   : NON MESURÉS (aucun topic de collision)')
        print(f" Recoveries Nav2       : {r['recoveries']}")
        print(f" Dernier statut Nav2   : {r['nav2']['last_status']}")
        print(f" Pertes localisation   : {r['localization']['loss_count']}")
        cov = r['amcl_cov']
        print(f" Covariance AMCL       : départ {cov['first_xx_yy_yaw']}  max {cov['max_xx_yy_yaw']}")
        err = r['localization_error']
        if err['available']:
            print(f" Erreur AMCL vs réel   : départ {err['first_xy']} m, max {err['max_xy']} m, "
                  f"moyenne {err['mean_xy']} m")
        else:
            print(' Erreur AMCL vs réel   : NON MESURÉE (vraie pose ou TF indisponible)')
        if r['at_first_loss']:
            print(f" Au moment de la perte : {r['at_first_loss']}")
        print(f" Facteur temps réel    : {r['real_time_factor']}")
        print(f" Fichier               : {self.report_path}")
        print('=' * 62)


# --------------------------------------------------------------------------
# Programme principal
# --------------------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(description="Benchmark d'un run de navigation PARC 2026.")
    parser.add_argument('--label', default='run',
                        help='Nom de la configuration testée (ex: baseline_sans_camera)')
    parser.add_argument('--time-limit', type=float, default=600.0,
                        help='Durée maximale en secondes simulées (défaut 600)')
    parser.add_argument('--stop-at', type=float, default=None,
                        help='Run court : arrêter après N secondes simulées (résultat CHECKPOINT)')
    parser.add_argument('--goal-x', type=float, default=None,
                        help='But X (défaut : lu dans task_params.yaml)')
    parser.add_argument('--goal-y', type=float, default=None,
                        help='But Y (défaut : lu dans task_params.yaml)')
    parser.add_argument('--goal-radius', type=float, default=None,
                        help='Rayon du cercle vert (défaut : lu dans model.sdf)')
    parser.add_argument('--pose-source', choices=['auto', 'truth', 'tf'], default='auto',
                        help="'truth' = vraie pose Gazebo ; 'tf' = AMCL ; "
                             "'auto' = vraie pose si disponible, sinon AMCL")
    parser.add_argument('--truth-topic', default='/sitoe_robot/pose',
                        help='Topic ROS de la vraie pose Gazebo (défaut /sitoe_robot/pose)')
    parser.add_argument('--output-dir', default=os.path.join(SCRIPT_DIR, 'results'),
                        help='Dossier des résultats')
    parser.add_argument('--contact-gap', type=float, default=1.0,
                        help='Secondes sans contact pour séparer deux épisodes (défaut 1.0)')
    parser.add_argument('--ignore-keywords', default='ground,floor,plane',
                        help='Contacts ignorés si un nom de collision contient un de ces mots')
    parser.add_argument('--debug-contacts', action='store_true',
                        help='Affiche chaque paire de collisions vue (pour régler les mots ignorés)')
    parser.add_argument('--freeze-abort', type=float, default=60.0,
                        help="Arrêter si /clock n'avance plus depuis N secondes réelles")
    parser.add_argument('--no-stop-on-nav-end', dest='stop_on_nav_end', action='store_false',
                        help="Continuer jusqu'à la limite de temps même si Nav2 a fini son goal")
    parser.add_argument('--nav-end-grace', type=float, default=5.0,
                        help="Secondes simulées d'attente après la fin du goal Nav2 (défaut 5)")
    return parser.parse_args()


def main():
    args = parse_args()

    # But : arguments, sinon task_params.yaml
    goal_x, goal_y = args.goal_x, args.goal_y
    if goal_x is None or goal_y is None:
        params = load_task_params()
        goal_x = float(params['goal_x']) if goal_x is None else goal_x
        goal_y = float(params['goal_y']) if goal_y is None else goal_y

    # Rayon du cercle : argument, sinon model.sdf, sinon valeur par défaut
    if args.goal_radius is not None:
        goal_radius, radius_source = args.goal_radius, 'argument'
    else:
        goal_radius, radius_source = read_goal_radius()
        if goal_radius is None:
            goal_radius = DEFAULT_GOAL_RADIUS
            radius_source = 'DEFAUT (a verifier)'
            print(f'ATTENTION : rayon du but introuvable, valeur par défaut {goal_radius} m.')
            print('            Donnez le vrai rayon avec --goal-radius.')

    print(f'Benchmark "{args.label}" : but=({goal_x:.3f}, {goal_y:.3f}), rayon={goal_radius} m '
          f'({radius_source})')

    rclpy.init()
    node = BenchmarkNode(args, goal_x, goal_y, goal_radius, radius_source)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    # Ctrl+C : on enregistre quand même ce qu'on a mesuré
    if not node.finished:
        node.finish('INTERRUPTED', 'Run arrêté à la main (Ctrl+C).')

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
