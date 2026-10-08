#!/usr/bin/env python3
"""Point d'entrée officiel PARC 2026 — Engineers League, tâche de navigation.

Commande d'évaluation (après `ros2 launch parc_robot_bringup task.launch.py`) :

    ros2 run caytu_nav_solution task_solution.py

Ce nœud exécute TOUTE la solution, sans aucune intervention manuelle :
  0. arrête un ancien lanceur de la solution resté en vie ;
  1. lit le point de départ et le but dans task_params.yaml (repère Gazebo) ;
  2. vérifie que la carte des murs correspond bien au monde chargé ;
  3. démarre la perception, la localisation et Nav2 (launch du bringup) ;
  4. envoie le but à Nav2 ; à chaque interruption, écrit la cause dans le
     journal, nettoie ce qu'il faut et renvoie le but ; si le but lui-même est
     inaccessible, vise le point libre le plus proche ;
  5. arrête le robot sur le but, écrit le rapport du trajet et ferme proprement
     tout ce qu'il a lancé.

Essais comparatifs (voir drive_settings.py) :

    ros2 run caytu_nav_solution task_solution.py --ros-args -p drive_profile:=reference
"""

import math
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from datetime import datetime
from typing import Optional

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import BackUp, ComputePathToPose, FollowPath, NavigateToPose, Spin
from nav2_msgs.srv import (
    ClearCostmapAroundRobot, ClearCostmapExceptRegion, ClearEntireCostmap)
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from caytu_nav_solution import drive_settings
from caytu_nav_solution.nav_math import (
    Pose2D, cell_cost, footprint_gap, load_pgm_map, nearest_free_cell, quaternion_from_yaw,
    scan_map_agreement, yaw_from_quaternion)
from caytu_nav_solution.process_utils import find_stale_bringups
from caytu_nav_solution.retry_policy import (
    CAUSE_CANCELED, CAUSE_NONE, CAUSE_SILENT, CAUSE_TF, CAUSE_TOO_FAR, CLEAR_ALL, CLEAR_FAR,
    CLEAR_NEAR, EXPLANATIONS, FALLBACK, RESEND, RetryPolicy, describe_error,
    resolve_error_names)
from caytu_nav_solution.run_report import RunMetrics, append_csv_row
from caytu_nav_solution.task_params import TaskParams, load_task_params

try:
    from ros_gz_interfaces.msg import Contacts
except ImportError:
    Contacts = None

EXIT_SUCCESS, EXIT_FAILURE, EXIT_TIMEOUT = 0, 1, 2
# Position du lidar dans base_footprint (URDF officiel) : 2,1 cm derrière le centre.
LIDAR_OFFSET_X = -0.021
# Dans une OccupancyGrid publiée par Nav2, 99 = le centre du robot toucherait
# un obstacle, 100 = obstacle.
GOAL_BLOCKED_COST = 99
# Nœuds gérés par le lifecycle manager de navigation.launch.py, dans l'ordre où
# il les active : bt_navigator actif veut dire que tout Nav2 est actif.
NAV2_NODES = ('map_server', 'controller_server', 'planner_server', 'behavior_server',
              'bt_navigator')
LIFECYCLE_ACTIVE = 3                    # lifecycle_msgs/State.PRIMARY_STATE_ACTIVE


class TaskSolution(Node):

    def __init__(self, parameter_overrides=None):
        super().__init__('task_solution', parameter_overrides=parameter_overrides or [])

        # Durée officielle de la tâche et marge gardée pour s'arrêter proprement.
        self.declare_parameter('time_limit_sec', 600.0)
        self.declare_parameter('stop_margin_sec', 1.0)
        # Le robot est considéré arrivé si son centre est à moins de ce rayon du
        # centre du but, même si Nav2 ne l'a pas encore annoncé (cercle : 0,6 m).
        self.declare_parameter('success_radius', 0.20)
        # Garde-fou : si Nav2 annonce l'arrivée alors que le robot est encore à
        # plus de cette distance du centre du but, le but est renvoyé.
        self.declare_parameter('arrival_recheck_radius', 0.35)
        # Pause après un nettoyage de costmap, en temps du nœud (simulé) : les
        # capteurs doivent avoir le temps de la remplir avant le but suivant.
        self.declare_parameter('retry_pause_sec', 1.0)
        # Démarrage de Nav2. Il arrive (rarement) que son gestionnaire de cycle
        # de vie reste bloqué : une réponse de service se perd et il attend sans
        # fin. Si aucun nœud de Nav2 ne change d'état pendant ce délai (temps
        # réel), tout le bringup est arrêté puis relancé ; le délai double à
        # chaque relance. Un démarrage lent mais qui avance n'est jamais coupé.
        self.declare_parameter('nav2_startup_timeout_sec', 60.0)
        self.declare_parameter('nav2_startup_restarts', 2)
        # Sans nouvelles de Nav2 pendant ce délai (temps du nœud) alors qu'un
        # but est en cours, le but est annulé puis renvoyé. 0.0 = désactivé.
        self.declare_parameter('nav2_silence_timeout_sec', 30.0)
        # Nettoyage après un échec : on garde la costmap globale dans ce rayon
        # autour du robot (les plateaux de table ne sont vus que vers l'avant).
        self.declare_parameter('clear_keep_radius', 2.5)
        # Côté du carré effacé autour du robot quand Nav2 le dit « dans un obstacle ».
        self.declare_parameter('clear_near_size', 1.2)
        # Lancement du reste de la solution par ce nœud (exigence PARC).
        self.declare_parameter('launch_bringup', True)
        self.declare_parameter('bringup_package', 'caytu_nav_bringup')
        self.declare_parameter('bringup_launch', 'solution_bringup.launch.py')
        # Réglages de conduite pour les essais comparatifs (drive_settings.py) :
        # drive_profile = default (valeurs de nav2_params.yaml) ou reference
        # (valeurs de la version mesurée à 2 min 33 s). Chaque réglage peut aussi
        # être donné seul ; vide = non donné.
        self.declare_parameter('drive_profile', drive_settings.PROFILE_DEFAULT)
        for name in drive_settings.NAMES:
            self.declare_parameter(name, '', ParameterDescriptor(dynamic_typing=True))
        # Chemin stabilisé (path_keeper.py) : action et arbre de comportement.
        self.declare_parameter('keeper_action', 'compute_path_stable')
        self.declare_parameter('keeper_tree', 'navigate_keep_path.xml')
        self.declare_parameter('keeper_wait_sec', 8.0)              # temps réel
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
        self.declare_parameter('cmd_vel_topic', '/robot_base_controller/cmd_vel_unstamped')
        self.declare_parameter('write_report', True)
        self.declare_parameter('report_dir', '~/.ros/fosa_runs')
        self.declare_parameter('goal_fallback', True)
        self.declare_parameter('goal_circle_radius', 0.60)
        self.declare_parameter('fallback_after_stalls', 2)
        self.declare_parameter('fallback_progress_min', 0.25)
        self.declare_parameter('fallback_max_radius', 1.50)
        # 70 correspond à un centre de robot à 0,35 m au moins d'un obstacle avec l'inflation globale (0,90 m / 3,0).
        self.declare_parameter('fallback_max_cost', 70)
        self.declare_parameter('fallback_max_checks', 6)
        self.declare_parameter('fallback_hold_sec', 10.0)
        # Planificateur interrogé pour le repli : celui de l'arbre de comportement.
        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')

        self._task: TaskParams = load_task_params(
            self.get_parameter('task_params_file').value or None)
        self._global_frame = self.get_parameter('global_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._bringup: Optional[subprocess.Popen] = None
        self._start_time: Optional[Time] = None
        self._last_scan: Optional[LaserScan] = None
        self._last_feedback_log = 0.0
        self._goal_handle = None        # but Nav2 en cours, pour pouvoir l'annuler
        self._goal_uses_keeper = False  # le but en cours passe par path_keeper
        self._last_immediate = False    # dernier échec : immédiat, sans cause, robot immobile
        # Réglages demandés : une valeur invalide arrête tout de suite, avec un
        # message clair, plutôt que de fausser un essai.
        profile = str(self.get_parameter('drive_profile').value)
        overrides = {name: self.get_parameter(name).value for name in drive_settings.NAMES}
        self._drive = drive_settings.resolve(profile, overrides)
        self._use_keeper = bool(self._drive['path_keeper'])
        self._keeper_strikes = 0        # interruptions inexpliquées avec le chemin stabilisé
        self._keeper_quick = 0          # échecs immédiats de suite avec le chemin stabilisé
        self._keeper_tree = ''          # chemin du fichier XML, résolu au démarrage
        self._metrics = RunMetrics()
        self._metrics.settings = drive_settings.describe(profile, overrides)
        self._metrics.contacts_available = Contacts is not None
        self._last_report_t = 0.0
        self._wall_start = time.monotonic()
        self._temp_dir: Optional[str] = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._target = (self._task.goal_x, self._task.goal_y)   # cible envoyée à Nav2
        self._on_fallback = False
        self._policy = RetryPolicy(
            progress_min=float(self.get_parameter('fallback_progress_min').value),
            fallback_after=int(self.get_parameter('fallback_after_stalls').value),
            fallback_enabled=bool(self.get_parameter('goal_fallback').value))
        # Codes d'erreur relus dans le paquet nav2_msgs installé.
        self._errors = resolve_error_names({
            'contrôleur': FollowPath.Result, 'planificateur': ComputePathToPose.Result,
            'pivot': Spin.Result, 'recul': BackUp.Result})
        self._last_cause = CAUSE_NONE
        self._last_feedback_t = 0.0
        self._costmap: Optional[OccupancyGrid] = None
        self._costmap_sub = None                                 # créé au premier besoin
        self._plan_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self._keeper_client = ActionClient(
            self, ComputePathToPose, str(self.get_parameter('keeper_action').value))
        # État de cycle de vie des nœuds de Nav2, pour suivre leur démarrage.
        self._state_clients = {
            name: self.create_client(GetState, f'/{name}/get_state') for name in NAV2_NODES}
        self._states = {}
        self._state_requests = {}
        self._restarts_left = 0
        self._startup_patience = 0.0
        self._cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self._clear_local = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self._clear_global_all = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self._clear_global_far = self.create_client(
            ClearCostmapExceptRegion, '/global_costmap/clear_except_global_costmap')
        self._clear_global_near = self.create_client(
            ClearCostmapAroundRobot, '/global_costmap/clear_around_global_costmap')
        self._scan_sub = self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self._on_scan,
            qos_profile_sensor_data)
        if Contacts is not None:
            for topic in ('/base_collisions', '/left_wheel_collisions',
                          '/right_wheel_collisions', '/top_chassis_collisions'):
                self.create_subscription(Contacts, topic, self._on_contacts, 10)

        self.get_logger().info(
            f'Départ ({self._task.spawn.x:.3f}, {self._task.spawn.y:.3f}, '
            f'cap {self._task.spawn.yaw:.3f}) -> but ({self._task.goal_x:.3f}, '
            f'{self._task.goal_y:.3f}), lus dans {self._task.path}')

    # ------------------------------------------------------------- utilitaires
    def _on_scan(self, msg: LaserScan):
        self._last_scan = msg

    def _on_contacts(self, msg) -> None:
        for contact in msg.contacts:
            self._metrics.add_contact(
                contact.collision1.name, contact.collision2.name)

    def _report_pose(self) -> None:
        """Échantillonne la pose pour le rapport, au plus 5 Hz (horloge du nœud)."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_report_t < 0.2:
            return
        self._last_report_t = now
        pose = self._robot_pose()
        if pose is None:
            return
        self._metrics.update_pose(
            now, pose.x, pose.y, self._task.goal_x, self._task.goal_y, pose.yaw)

    def _spin(self, wall_seconds: float):
        """Traite les messages ROS pendant une durée réelle donnée."""
        end = time.monotonic() + wall_seconds
        while rclpy.ok():
            remaining = end - time.monotonic()
            if remaining <= 0.0:
                break
            rclpy.spin_once(self, timeout_sec=min(0.05, remaining))
            self._report_pose()

    def _elapsed(self) -> float:
        """Temps écoulé depuis le lancement de la solution (horloge du nœud)."""
        if self._start_time is None:
            return 0.0
        return (self.get_clock().now() - self._start_time).nanoseconds * 1e-9

    def _time_left(self) -> float:
        return (float(self.get_parameter('time_limit_sec').value)
                - float(self.get_parameter('stop_margin_sec').value) - self._elapsed())

    def _robot_pose(self) -> Optional[Pose2D]:
        try:
            tf = self._tf_buffer.lookup_transform(self._global_frame, self._base_frame, Time())
        except TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return Pose2D(t.x, t.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def _distance_to_goal(self) -> Optional[float]:
        pose = self._robot_pose()
        if pose is None:
            return None
        return pose.distance_to(self._task.goal_x, self._task.goal_y)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _spin_sim(self, seconds: float) -> None:
        """Traite les messages pendant une durée en temps du nœud (simulé).

        Garde-fou : si l'horloge simulée n'avance plus, on sort quand même au
        bout de 30 fois cette durée en temps réel (au moins 2 s).
        """
        target = self._now_sec() + seconds
        wall_end = time.monotonic() + max(2.0, 30.0 * seconds)
        while (rclpy.ok() and self._now_sec() < target and self._time_left() > 0.0
               and time.monotonic() < wall_end):
            rclpy.spin_once(self, timeout_sec=0.05)
            self._report_pose()

    def _distance_to_target(self) -> Optional[float]:
        pose = self._robot_pose()
        if pose is None:
            return None
        return pose.distance_to(self._target[0], self._target[1])

    def _fresh_costmap(self) -> Optional[OccupancyGrid]:
        """Costmap globale publiée au moins 1 s (temps du nœud) après l'appel.

        L'attente laisse aux capteurs le temps de remplir une costmap que
        l'arbre de comportement vient peut-être d'effacer. Bornée à 5 s de temps
        du nœud et à 60 s de temps réel (horloge simulée arrêtée).
        """
        if self._costmap_sub is None:
            self._costmap_sub = self.create_subscription(
                OccupancyGrid, str(self.get_parameter('costmap_topic').value),
                lambda msg: setattr(self, '_costmap', msg),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
        t0 = self._now_sec()
        wall_end = time.monotonic() + 60.0
        self._costmap = None
        while (rclpy.ok() and self._time_left() > 0.0 and self._now_sec() <= t0 + 5.0
               and time.monotonic() < wall_end):
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._costmap is not None and self._costmap.header.stamp.sec + \
                    self._costmap.header.stamp.nanosec * 1e-9 >= t0 + 1.0:
                return self._costmap
        return self._costmap

    def _goal_blocked(self) -> tuple:
        """(but officiel occupé dans la costmap globale ?, costmap lue ou None).

        Nav2 ne signale presque jamais lui-même un but occupé : avec une
        tolérance de planification non nulle, Smac répond « aucun chemin ».
        """
        if not bool(self.get_parameter('goal_fallback').value) or self._on_fallback:
            return False, None
        costmap = self._fresh_costmap()
        if costmap is None:
            return False, None
        cost = cell_cost(
            costmap.data, costmap.info.width, costmap.info.height, costmap.info.resolution,
            costmap.info.origin.position.x, costmap.info.origin.position.y,
            self._task.goal_x, self._task.goal_y)
        return cost is not None and cost >= GOAL_BLOCKED_COST, costmap

    def _plan_exists(self, x: float, y: float) -> bool:
        """Vrai si le planificateur trouve un chemin jusqu'à (x, y).

        Le robot ne bouge pas pendant cette question.
        """
        return self._plan_with(str(self.get_parameter('planner_id').value), x, y)

    def _plan_with(self, planner: str, x: float, y: float) -> bool:
        if not self._plan_client.wait_for_server(timeout_sec=1.0):
            return False
        pose = self._robot_pose() or self._task.spawn
        heading = math.atan2(y - pose.y, x - pose.x)
        goal = PoseStamped()
        goal.header.frame_id = self._global_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        qx, qy, qz, qw = quaternion_from_yaw(heading)
        goal.pose.orientation.x, goal.pose.orientation.y = qx, qy
        goal.pose.orientation.z, goal.pose.orientation.w = qz, qw
        request = ComputePathToPose.Goal()
        request.goal = goal
        request.planner_id = planner
        request.use_start = False
        send = self._plan_client.send_goal_async(request)
        end = time.monotonic() + 10.0
        while rclpy.ok() and not send.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            self._report_pose()
            if self._time_left() <= 0.0 or not self._bringup_alive() or time.monotonic() > end:
                return False
        try:
            handle = send.result() if send.done() else None
        except Exception:               # noqa: BLE001 — serveur d'action disparu
            return False
        if handle is None or not handle.accepted:
            return False
        result = handle.get_result_async()
        while rclpy.ok() and not result.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            self._report_pose()
            if self._time_left() <= 0.0 or not self._bringup_alive() or time.monotonic() > end:
                return False
        try:
            res = result.result() if result.done() else None
        except Exception:               # noqa: BLE001
            return False
        return res is not None and res.status == GoalStatus.STATUS_SUCCEEDED \
            and len(res.result.path.poses) > 0

    def _choose_fallback(self, costmap: Optional[OccupancyGrid] = None) -> bool:
        """Vise le point libre le plus proche du but. `costmap` : lecture déjà faite."""
        if costmap is None:
            costmap = self._fresh_costmap()
        if costmap is None:
            self.get_logger().warn('Repli : costmap indisponible.')
            return False
        excluded: list = []
        goal_x, goal_y = self._task.goal_x, self._task.goal_y
        max_radius = float(self.get_parameter('fallback_max_radius').value)
        max_cost = int(self.get_parameter('fallback_max_cost').value)
        max_checks = int(self.get_parameter('fallback_max_checks').value)
        for _ in range(max_checks):
            cell = nearest_free_cell(
                costmap.data, costmap.info.width, costmap.info.height,
                costmap.info.resolution, costmap.info.origin.position.x,
                costmap.info.origin.position.y, goal_x, goal_y,
                max_radius, max_cost, excluded)
            if cell is None:
                break
            if cell[2] < costmap.info.resolution and not self._on_fallback:
                return False
            if self._plan_exists(cell[0], cell[1]):
                self._target = (cell[0], cell[1])
                self._on_fallback = True
                self._metrics.fallback_used = True
                self._metrics.fallback_offset_m = cell[2]
                self._policy.reset(self._distance_to_goal())
                self.get_logger().warn(
                    f'Repli : but officiel inaccessible, nouvelle cible à {cell[2]:.2f} m '
                    'du centre du but.')
                return True
            excluded.append((cell[0], cell[1]))
        self.get_logger().warn(
            f'Repli : aucun point libre atteignable à moins de {max_radius} m du but.')
        return False

    def _reset_target(self) -> None:
        self._target = (self._task.goal_x, self._task.goal_y)
        self._on_fallback = False
        self._policy.reset(self._distance_to_goal())

    def _bringup_alive(self) -> bool:
        return self._bringup is None or self._bringup.poll() is None

    # ------------------------------------------------------------------ carte
    def _choose_map(self) -> str:
        """Retourne le YAML de carte à donner à Nav2 (murs réels ou carte vide)."""
        mode = str(self.get_parameter('static_map').value).lower()
        map_yaml = self.get_parameter('map_yaml').value
        if not map_yaml:
            from ament_index_python.packages import get_package_share_directory
            map_yaml = os.path.join(
                get_package_share_directory(self.get_parameter('bringup_package').value),
                'maps', 'cafe_map.yaml')

        if mode == 'never':
            self.get_logger().info('Carte des murs désactivée (static_map=never).')
            return self._blank_or_cafe(map_yaml)
        if mode == 'always':
            self.get_logger().info('Carte des murs imposée (static_map=always).')
            return map_yaml

        # Mode auto : la carte décrit le café officiel. Si le monde chargé est
        # différent, elle gênerait le planificateur ; on le vérifie en comparant
        # un scan lidar pris au point de départ avec les murs de la carte.
        try:
            with open(map_yaml, encoding='utf-8') as stream:
                meta = yaml.safe_load(stream)
            image = meta['image']
            if not os.path.isabs(image):
                image = os.path.join(os.path.dirname(map_yaml), image)
            with open(image, 'rb') as stream:
                grid = load_pgm_map(stream.read(), float(meta['resolution']),
                                    float(meta['origin'][0]), float(meta['origin'][1]))
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
            self.get_logger().warn(f'Carte illisible ({error}) : navigation sans carte.')
            return self._blank_or_cafe(map_yaml)

        deadline = time.monotonic() + float(self.get_parameter('map_check_timeout_sec').value)
        while rclpy.ok() and self._last_scan is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        scan = self._last_scan
        if scan is None:
            self.get_logger().warn(
                'Pas de scan reçu pour vérifier la carte : carte du café conservée.')
            return map_yaml

        lidar = self._task.spawn.compose(Pose2D(LIDAR_OFFSET_X, 0.0, 0.0))
        agreement, used = scan_map_agreement(
            grid, lidar, scan.angle_min, scan.angle_increment, scan.ranges)
        threshold = float(self.get_parameter('map_check_min_agreement').value)
        if used >= 30 and agreement < threshold:
            self.get_logger().warn(
                f'Seulement {agreement * 100:.0f} % des {used} retours lidar tombent sur un '
                'mur de la carte : le monde ne semble pas être le café officiel, '
                'navigation sans carte statique.')
            return self._blank_or_cafe(map_yaml)
        self.get_logger().info(
            f'Carte du café validée : {agreement * 100:.0f} % des {used} retours lidar '
            'coïncident avec ses murs.')
        return map_yaml

    def _write_blank_map(self) -> str:
        """Écrit une carte entièrement libre couvrant largement départ et but."""
        resolution, margin = 0.05, 15.0
        x_min = min(self._task.spawn.x, self._task.goal_x) - margin
        y_min = min(self._task.spawn.y, self._task.goal_y) - margin
        x_max = max(self._task.spawn.x, self._task.goal_x) + margin
        y_max = max(self._task.spawn.y, self._task.goal_y) + margin
        width = int(math.ceil((x_max - x_min) / resolution))
        height = int(math.ceil((y_max - y_min) / resolution))
        folder = tempfile.mkdtemp(prefix='fosa_map_')
        self._temp_dir = folder
        with open(os.path.join(folder, 'blank_map.pgm'), 'wb') as stream:
            stream.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
            stream.write(bytes([254]) * (width * height))
        yaml_path = os.path.join(folder, 'blank_map.yaml')
        with open(yaml_path, 'w', encoding='utf-8') as stream:
            stream.write(
                'image: blank_map.pgm\nmode: trinary\n'
                f'resolution: {resolution}\norigin: [{x_min:.3f}, {y_min:.3f}, 0.0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        return yaml_path

    def _blank_or_cafe(self, map_yaml: str) -> str:
        try:
            return self._write_blank_map()
        except OSError as error:
            self.get_logger().warn(
                f'Carte vide impossible à écrire ({error}) : carte du café conservée.')
            return map_yaml

    # ---------------------------------------------------------------- bringup
    def _start_bringup(self, map_yaml: str):
        if not self.get_parameter('launch_bringup').value:
            self.get_logger().info('launch_bringup=false : le bringup doit déjà tourner.')
            return
        use_sim_time = self.get_parameter('use_sim_time').value
        command = [
            'ros2', 'launch',
            self.get_parameter('bringup_package').value,
            self.get_parameter('bringup_launch').value,
            f'use_sim_time:={"true" if use_sim_time else "false"}',
            f'map:={map_yaml}',
            f'task_params_file:={self._task.path}',
        ] + drive_settings.launch_arguments(self._drive)
        self.get_logger().info('Démarrage de la solution : ' + ' '.join(command))
        # Groupe de processus dédié : permet d'arrêter tout le bringup d'un coup.
        self._bringup = subprocess.Popen(command, start_new_session=True)

    def stop_bringup(self):
        if self._bringup is None or self._bringup.poll() is not None:
            return
        self.get_logger().info('Arrêt du bringup...')
        for sig, wait in ((signal.SIGINT, 10.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 2.0)):
            try:
                os.killpg(os.getpgid(self._bringup.pid), sig)
            except ProcessLookupError:
                return
            try:
                self._bringup.wait(timeout=wait)
                return
            except subprocess.TimeoutExpired:
                continue

    def _stop_stale_bringups(self) -> None:
        pids = find_stale_bringups(
            str(self.get_parameter('bringup_package').value),
            str(self.get_parameter('bringup_launch').value), os.getpid())
        if not pids:
            return
        self.get_logger().warn(
            f'Ancien lanceur de la solution encore actif (PID {pids}) : arrêt avant de démarrer.')
        for pid in pids:
            for sig, wait in ((signal.SIGINT, 10.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 2.0)):
                try:
                    pgid = os.getpgid(pid)
                except ProcessLookupError:
                    break
                try:
                    if pgid != os.getpgrp():
                        os.killpg(pgid, sig)
                    else:
                        os.kill(pid, sig)
                except ProcessLookupError:
                    break
                except PermissionError as error:
                    self.get_logger().warn(f'PID {pid} non arrêtable : {error!r}.')
                    break
                end = time.monotonic() + wait
                while time.monotonic() < end:
                    if not os.path.exists(f'/proc/{pid}'):
                        break
                    self._spin(0.1)
                if not os.path.exists(f'/proc/{pid}'):
                    break
        self._spin(1.0)

    def stop_robot(self):
        """Envoie plusieurs commandes nulles : le robot reste où il est.

        Indispensable : le plugin DiffDrive de Gazebo garde la dernière vitesse
        reçue tant qu'on ne lui en envoie pas une autre.
        """
        if not rclpy.ok():
            return
        for _ in range(10):
            self._cmd_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

    def stop_motion(self):
        """Annule le but en cours et arrête le robot (le bringup reste en place)."""
        if rclpy.ok() and self._goal_handle is not None:
            # Nav2 doit cesser de commander le robot avant qu'on l'arrête.
            future = self._goal_handle.cancel_goal_async()
            end = time.monotonic() + 1.0
            while rclpy.ok() and not future.done() and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
            self._goal_handle = None
        self.stop_robot()

    def shutdown_sequence(self):
        """Arrêt ordonné, quel que soit le motif (arrivée, échec, Ctrl-C)."""
        self.stop_motion()
        self.stop_bringup()
        if self._temp_dir is not None:
            shutil.rmtree(self._temp_dir, ignore_errors=True)
        # Dernière commande nulle une fois que plus rien ne peut en publier.
        self.stop_robot()

    # ------------------------------------------------------------------- Nav2
    def _wait_for_clock(self):
        """Démarre le chronomètre dès que l'horloge (simulée) est disponible."""
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.get_clock().now().nanoseconds > 0:
                break
        self._start_time = self.get_clock().now()
        if self._start_time.nanoseconds == 0:
            self.get_logger().warn('Horloge /clock absente : la simulation tourne-t-elle ?')

    def _poll_lifecycle(self) -> bool:
        """Demande l'état de chaque nœud de Nav2 ; vrai si l'un d'eux a changé.

        Une seule demande en cours par nœud ; une demande sans réponse depuis
        3 s (temps réel) est abandonnée puis refaite.
        """
        changed = False
        now = time.monotonic()
        for name, client in self._state_clients.items():
            pending = self._state_requests.get(name)
            if pending is not None:
                future, sent = pending
                if future.done():
                    del self._state_requests[name]
                    try:
                        state = int(future.result().current_state.id)
                    except Exception:   # noqa: BLE001 — nœud disparu entre-temps
                        state = None
                    if state is not None and self._states.get(name) != state:
                        self._states[name] = state
                        changed = True
                elif now - sent > 3.0:
                    del self._state_requests[name]
                continue
            if client.service_is_ready():
                self._state_requests[name] = (client.call_async(GetState.Request()), now)
        return changed

    def _nav2_active(self) -> bool:
        """Tout Nav2 est actif (le lifecycle manager active bt_navigator en dernier)."""
        return self._states.get('bt_navigator') == LIFECYCLE_ACTIVE

    def _restart_bringup(self, reason: str) -> bool:
        """Arrête puis relance le bringup ; faux si plus aucune relance n'est permise."""
        if self._restarts_left <= 0:
            self.get_logger().error(f'Démarrage impossible : {reason}.')
            return False
        self._restarts_left -= 1
        self._metrics.nav2_restarts += 1
        self.get_logger().warn(
            f'{reason[0].upper()}{reason[1:]} : arrêt puis relance du bringup '
            f'({self._restarts_left} relance(s) encore possible(s)).')
        self.stop_bringup()
        self._states.clear()
        self._state_requests.clear()
        self._start_bringup(self._metrics.map_used)
        self._startup_patience *= 2.0
        return True

    def _wait_for_nav2(self) -> bool:
        """Attend que Nav2 soit actif et que le robot soit localisé.

        Si plus rien n'avance pendant nav2_startup_timeout_sec (aucun nœud de
        Nav2 ne change d'état), ou si le bringup s'arrête pendant le
        démarrage, le bringup est relancé.
        """
        if not self._startup_patience:
            self._startup_patience = float(self.get_parameter('nav2_startup_timeout_sec').value)
            owns_bringup = bool(self.get_parameter('launch_bringup').value)
            self._restarts_left = (int(self.get_parameter('nav2_startup_restarts').value)
                                   if owns_bringup else 0)
        start = progress = last_log = time.monotonic()
        while rclpy.ok():
            self._spin(0.2)
            if self._poll_lifecycle():
                progress = time.monotonic()
            server = self._nav_client.server_is_ready()
            localized = self._robot_pose() is not None
            # Repli si les services d'état n'existent pas : serveur d'action
            # découvert depuis 5 s (ancien critère, sans surveillance du blocage).
            no_states = not self._states and time.monotonic() - start > 5.0
            if localized and server and (self._nav2_active() or no_states):
                self._metrics.mark_nav2_ready(self._now_sec())
                self.get_logger().info(
                    f'Nav2 prêt et robot localisé après {time.monotonic() - start:.1f} s réelles.')
                return True
            if self._time_left() <= 0.0:
                self.get_logger().error('Nav2 n\'est pas devenu disponible à temps.')
                return False
            still = time.monotonic() - progress
            dead = not self._bringup_alive()
            if dead or still > self._startup_patience:
                stuck = [name for name in NAV2_NODES
                         if self._states.get(name) != LIFECYCLE_ACTIVE]
                reason = ('le bringup s\'est arrêté pendant le démarrage' if dead else
                          f'démarrage de Nav2 bloqué depuis {still:.0f} s réelles '
                          f'(pas encore actifs : {", ".join(stuck)})')
                if not self._restart_bringup(reason):
                    return False
                start = progress = last_log = time.monotonic()
                continue
            if time.monotonic() - last_log > 5.0:
                last_log = time.monotonic()
                active = sum(self._states.get(name) == LIFECYCLE_ACTIVE for name in NAV2_NODES)
                self.get_logger().info(
                    f'Attente de Nav2 ({active}/{len(NAV2_NODES)} nœuds actifs, '
                    f'localisation : {"oui" if localized else "non"})...')
        return False

    def _goal_message(self) -> NavigateToPose.Goal:
        pose = self._robot_pose() or self._task.spawn
        heading = math.atan2(self._target[1] - pose.y, self._target[0] - pose.x)
        goal = PoseStamped()
        goal.header.frame_id = self._global_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = self._target[0]
        goal.pose.position.y = self._target[1]
        qx, qy, qz, qw = quaternion_from_yaw(heading)
        goal.pose.orientation.x, goal.pose.orientation.y = qx, qy
        goal.pose.orientation.z, goal.pose.orientation.w = qz, qw
        message = NavigateToPose.Goal()
        message.pose = goal
        # Arbre « chemin stabilisé » seulement si son nœud répond : sinon l'arbre
        # par défaut, qui parle directement au planificateur.
        self._goal_uses_keeper = self._keeper_ready()
        if self._goal_uses_keeper:
            message.behavior_tree = self._keeper_tree
        return message

    # --------------------------------------------------------- chemin stabilisé
    def _resolve_keeper_tree(self) -> None:
        """Trouve le fichier XML de l'arbre « chemin stabilisé »."""
        if not self._use_keeper:
            return
        tree = str(self.get_parameter('keeper_tree').value)
        if not os.path.isabs(tree):
            try:
                from ament_index_python.packages import get_package_share_directory
                tree = os.path.join(
                    get_package_share_directory(self.get_parameter('bringup_package').value),
                    'behavior_trees', tree)
            except Exception as error:      # noqa: BLE001 — paquet introuvable
                self.get_logger().warn(f'Arbre du chemin stabilisé introuvable ({error!r}).')
                tree = ''
        if tree and os.path.isfile(tree):
            self._keeper_tree = tree
        else:
            self.get_logger().warn(
                'Chemin stabilisé désactivé : arbre de comportement introuvable.')
            self._use_keeper = False

    def _keeper_ready(self) -> bool:
        return (self._use_keeper and bool(self._keeper_tree)
                and self._keeper_client.server_is_ready())

    def _wait_for_keeper(self) -> None:
        """Laisse au nœud path_keeper le temps d'apparaître (temps réel borné)."""
        if not self._use_keeper:
            return
        end = time.monotonic() + float(self.get_parameter('keeper_wait_sec').value)
        while rclpy.ok() and not self._keeper_ready() and time.monotonic() < end \
                and self._bringup_alive():
            self._spin(0.1)
        if self._keeper_ready():
            self.get_logger().info('Chemin stabilisé actif.')
        else:
            self.get_logger().warn(
                'Chemin stabilisé indisponible : navigation avec l\'arbre par défaut.')

    def _note_keeper_failure(self, cause: str, immediate: bool) -> None:
        """Le chemin stabilisé est mis de côté s'il semble en cause.

        Deux échecs immédiats de suite (l'arbre n'arrive pas à démarrer avec
        lui), ou deux interruptions sans cause ou pour délai dépassé : retour à
        l'arbre par défaut pour le reste du trajet.
        """
        if not (self._use_keeper and self._goal_uses_keeper):
            return
        if immediate and cause == CAUSE_NONE:
            self._keeper_quick += 1
        elif cause in (CAUSE_NONE, 'TIMEOUT'):
            self._keeper_quick = 0
            self._keeper_strikes += 1
        else:
            self._keeper_quick = 0
            return
        if self._keeper_quick >= 2 or self._keeper_strikes >= 2:
            self._use_keeper = False
            self.get_logger().warn(
                'Chemin stabilisé désactivé après deux interruptions où il pouvait être en '
                'cause : retour à l\'arbre par défaut.')

    def _on_feedback(self, feedback_msg):
        self._last_feedback_t = self._now_sec()       # Nav2 est vivant
        now = time.monotonic()
        if now - self._last_feedback_log < 5.0:
            return
        self._last_feedback_log = now
        remaining = getattr(feedback_msg.feedback, 'distance_remaining', float('nan'))
        self.get_logger().info(
            f'En route : {remaining:.2f} m restants, {self._elapsed():.0f} s écoulées.')

    def _clear(self, action: str) -> None:
        """Nettoie les costmaps selon la décision de RetryPolicy.

        La costmap locale (fenêtre de 6 m qui suit le robot) est toujours
        effacée : lidar et caméras la remplissent en moins d'une seconde.
        La costmap globale garde sa mémoire autant que possible.
        """
        if self._clear_local.service_is_ready():
            self._clear_local.call_async(ClearEntireCostmap.Request())
        if action == CLEAR_FAR and self._clear_global_far.service_is_ready():
            request = ClearCostmapExceptRegion.Request()
            request.reset_distance = 2.0 * float(self.get_parameter('clear_keep_radius').value)
            self._clear_global_far.call_async(request)
            what = (f'costmap globale effacée à plus de '
                    f'{self.get_parameter("clear_keep_radius").value} m du robot')
        elif action == CLEAR_NEAR and self._clear_global_near.service_is_ready():
            request = ClearCostmapAroundRobot.Request()
            request.reset_distance = float(self.get_parameter('clear_near_size').value)
            self._clear_global_near.call_async(request)
            what = 'costmap globale effacée autour du robot'
        elif action in (CLEAR_ALL, CLEAR_FAR, CLEAR_NEAR) \
                and self._clear_global_all.service_is_ready():
            # Dernier recours, ou service partiel indisponible.
            self._clear_global_all.call_async(ClearEntireCostmap.Request())
            what = 'costmap globale entièrement effacée'
        else:
            what = 'costmap globale conservée (service indisponible)'
        self.get_logger().info(f'Suite : {what}, puis nouvel envoi du but.')

    def _cancel(self, goal_handle, wall_seconds: float) -> None:
        """Demande l'annulation d'un but et attend la réponse (temps réel borné)."""
        try:
            future = goal_handle.cancel_goal_async()
        except Exception as error:      # noqa: BLE001 — Nav2 peut avoir disparu
            self.get_logger().warn(f'Annulation impossible ({error!r}).')
            return
        end = time.monotonic() + wall_seconds
        while rclpy.ok() and not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            self._report_pose()

    def _overlaps_goal_circle(self, margin: float = 0.05) -> bool:
        """Une partie du robot est-elle dans le cercle du but (règle PARC) ?"""
        pose = self._robot_pose()
        if pose is None:
            return False
        circle = float(self.get_parameter('goal_circle_radius').value)
        return footprint_gap(pose, self._task.goal_x, self._task.goal_y) <= circle - margin

    def _done(self) -> bool:
        """Arrivé : centre sur le but, ou, si le but est occupé, robot dans le cercle."""
        return self._arrived() or (self._on_fallback and self._overlaps_goal_circle())

    def _arrived(self) -> bool:
        distance = self._distance_to_goal()
        return distance is not None and distance <= float(
            self.get_parameter('success_radius').value)

    def _write_report(self, code: int) -> None:
        """Écrit le rapport (JSON + CSV + journal), une seule fois en fin de trajet."""
        if not bool(self.get_parameter('write_report').value):
            return
        try:
            import json
            names = {EXIT_SUCCESS: 'success', EXIT_TIMEOUT: 'timeout'}
            result = names.get(code, 'failure')
            final = self._distance_to_goal()
            self._metrics.finish(result, self._now_sec(),
                                 time.monotonic() - self._wall_start, final)
            folder = os.path.expanduser(str(self.get_parameter('report_dir').value))
            os.makedirs(folder, exist_ok=True)
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            data = self._metrics.to_dict()
            with open(os.path.join(folder, f'run_{stamp}.json'), 'w', encoding='utf-8') as stream:
                json.dump(data, stream, indent=2)
            append_csv_row(os.path.join(folder, 'trajets.csv'), data)
            self.get_logger().info('\n' + self._metrics.to_text())
        except Exception as error:      # noqa: BLE001 — le rapport ne doit jamais gêner l'arrêt
            self.get_logger().warn(f'Rapport non écrit : {error!r}')

    def _set_cause(self, cause: str) -> None:
        self._last_cause = cause

    def _navigate_once(self) -> str:
        """Un envoi du but.

        Retourne 'success', 'retry' (échec de Nav2, cause dans _last_cause),
        'rejected' (Nav2 pas encore actif), 'timeout', 'abort' (bringup arrêté)
        ou 'hold' (cible de repli atteinte hors du cercle).
        """
        self._set_cause(CAUSE_NONE)
        self._last_immediate = False
        sent_at, sent_pose = self._now_sec(), self._robot_pose()
        send_future = self._nav_client.send_goal_async(
            self._goal_message(), feedback_callback=self._on_feedback)
        ack_deadline = time.monotonic() + 10.0
        while rclpy.ok() and not send_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            self._report_pose()
            if self._time_left() <= 0.0:
                return 'timeout'
            if not self._bringup_alive():
                return 'abort'
            if time.monotonic() > ack_deadline:
                self.get_logger().warn('Nav2 n\'a pas répondu à l\'envoi du but en 10 s.')
                return 'rejected'
        try:
            goal_handle = send_future.result() if send_future.done() else None
        except Exception as error:      # noqa: BLE001 — serveur d'action disparu
            self.get_logger().warn(f'Envoi du but impossible ({error!r}).')
            goal_handle = None
        if goal_handle is None or not goal_handle.accepted:
            return 'rejected'
        self._goal_handle = goal_handle
        self._metrics.mark_goal_accepted(self._now_sec())
        self._last_feedback_t = self._now_sec()
        silence = float(self.get_parameter('nav2_silence_timeout_sec').value)

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            self._report_pose()
            if self._time_left() <= 0.0:
                self.get_logger().error('Limite de temps atteinte : annulation du but.')
                self._cancel(goal_handle, 2.0)
                return 'timeout'
            if not self._bringup_alive():
                return 'abort'
            if silence > 0.0 and self._now_sec() - self._last_feedback_t > silence:
                # Nav2 a accepté le but puis ne dit plus rien : sans cette
                # garde, la solution attendrait jusqu'à la limite de temps.
                self.get_logger().error(
                    f'Aucune nouvelle de Nav2 depuis {silence:.0f} s : but annulé puis renvoyé.')
                self._cancel(goal_handle, 2.0)
                self._goal_handle = None
                self._set_cause(CAUSE_SILENT)
                return 'retry'

        self._goal_handle = None
        try:
            result = result_future.result() if result_future.done() else None
        except Exception as error:      # noqa: BLE001
            self.get_logger().warn(f'Résultat de Nav2 illisible ({error!r}).')
            result = None
        status = result.status if result is not None else GoalStatus.STATUS_UNKNOWN
        if status == GoalStatus.STATUS_SUCCEEDED:
            distance = self._distance_to_target()
            limit = float(self.get_parameter('arrival_recheck_radius').value)
            if distance is not None and distance > limit:
                self._set_cause(CAUSE_TOO_FAR)
                self.get_logger().warn(
                    f'Nav2 annonce l\'arrivée mais le robot est à {distance:.2f} m de la cible : '
                    'nouvel envoi.')
                return 'retry'
            if not self._on_fallback:
                return 'success'
            return 'success' if self._overlaps_goal_circle() else 'hold'
        if self._done():
            # But occupé : le robot est bloqué par l'obstacle posé sur le but,
            # mais il est déjà dans le cercle. Insister ne le rapprocherait pas.
            return 'success'
        code = getattr(getattr(result, 'result', None), 'error_code', 0)
        cause, reason = describe_error(int(code or 0), self._errors)
        if cause == CAUSE_NONE and status == GoalStatus.STATUS_CANCELED:
            cause, reason = CAUSE_CANCELED, EXPLANATIONS[CAUSE_CANCELED]
        self._set_cause(cause)
        pose = self._robot_pose()
        moved = (pose.distance_to(sent_pose.x, sent_pose.y)
                 if pose is not None and sent_pose is not None else None)
        # Échec sans cause, aussitôt après l'envoi et sans que le robot bouge.
        self._last_immediate = (self._now_sec() - sent_at < 1.0
                                and moved is not None and moved < 0.05)
        distance = self._distance_to_goal()
        self.get_logger().warn(
            f'Navigation interrompue par Nav2 (statut {status}) : {reason}'
            + (f', à {distance:.2f} m du but.' if distance is not None else '.'))
        return 'retry'

    # ------------------------------------------------------------------- tâche
    def run(self) -> int:
        self._wait_for_clock()
        # Instants absolus de l'horloge du nœud : les poses du rapport sont datées ainsi.
        self._metrics.start(self._now_sec(), time.monotonic() - self._wall_start)
        self.get_logger().info(f'Réglages de conduite : {self._metrics.settings}.')
        map_yaml = self._choose_map()
        self._metrics.map_used = map_yaml
        self.destroy_subscription(self._scan_sub)      # le scan ne sert plus ici
        if bool(self.get_parameter('launch_bringup').value):
            self._stop_stale_bringups()
        self._resolve_keeper_tree()
        self._start_bringup(map_yaml)
        if not self._wait_for_nav2():
            return EXIT_FAILURE
        self._wait_for_keeper()

        self._policy.reset(self._distance_to_goal())
        attempt = 0
        rejections = 0
        rejected_since = time.monotonic()
        while rclpy.ok():
            if self._done():
                outcome = 'success'
            elif self._time_left() <= 0.0:
                outcome = 'timeout'
            else:
                if rejections == 0:
                    attempt += 1
                    self._metrics.add_attempt()
                    distance = self._distance_to_goal()
                    self.get_logger().info(
                        f'Envoi du but, tentative {attempt}'
                        + (f' ({distance:.2f} m à parcourir).' if distance is not None else '.'))
                outcome = self._navigate_once()

            if outcome == 'rejected':
                # Nav2 existe mais n'est pas encore actif : on insiste, sans
                # toucher aux costmaps ni compter une tentative. Des refus qui
                # durent trop sont traités comme un démarrage bloqué.
                if rejections == 0:
                    rejected_since = time.monotonic()
                if rejections % 10 == 0:
                    self.get_logger().info('Nav2 n\'accepte pas encore de but, nouvel essai...')
                rejections += 1
                self._metrics.add_rejection()
                if time.monotonic() - rejected_since > self._startup_patience:
                    if not self._restart_bringup(
                            f'Nav2 refuse le but depuis {self._startup_patience:.0f} s réelles') \
                            or not self._wait_for_nav2():
                        return EXIT_FAILURE
                    rejections = 0
                    continue
                self._spin(0.2)
                continue
            rejections = 0

            if outcome == 'success':
                self.stop_robot()
                distance = self._distance_to_goal()
                if self._on_fallback:
                    self.get_logger().info(
                        f'BUT ATTEINT par repli en {self._elapsed():.1f} s, {attempt} tentative(s)'
                        + (f' : centre du robot à {distance:.2f} m du centre du but, robot '
                           'dans le cercle de 0,6 m.' if distance is not None else '.'))
                else:
                    self.get_logger().info(
                        f'BUT ATTEINT en {self._elapsed():.1f} s, {attempt} tentative(s)'
                        + (f', centre du robot à {distance:.2f} m du centre du but.'
                           if distance is not None else '.'))
                return EXIT_SUCCESS
            if outcome == 'timeout':
                self.stop_robot()
                distance = self._distance_to_goal()
                self.get_logger().error(
                    'Limite de temps atteinte'
                    + (f' à {distance:.2f} m du but.' if distance is not None else '.'))
                return EXIT_TIMEOUT
            if outcome == 'abort':
                self.get_logger().error('Le bringup s\'est arrêté : navigation impossible.')
                return EXIT_FAILURE

            if outcome == 'hold':
                # Cible de repli atteinte hors du cercle : le robot ne bouge plus et
                # réexamine la situation sans rouler.
                self.stop_robot()
                self.get_logger().warn('Repli atteint hors du cercle : attente et réexamen.')
                saved_target = self._target
                saved_offset = self._metrics.fallback_offset_m
                while rclpy.ok() and self._time_left() > 0.0:
                    self._spin_sim(float(self.get_parameter('fallback_hold_sec').value))
                    if self._time_left() <= 0.0:
                        break
                    if self._plan_exists(self._task.goal_x, self._task.goal_y):
                        self.get_logger().info('Le but officiel est de nouveau atteignable.')
                        self._reset_target()
                        break
                    previous = self._metrics.fallback_offset_m
                    if self._choose_fallback() and previous is not None \
                            and self._metrics.fallback_offset_m <= previous - 0.10:
                        break                       # un point nettement plus proche existe
                    self._target = saved_target
                    self._metrics.fallback_offset_m = saved_offset
                continue

            # Échec récupérable : la suite dépend de la cause et du progrès réel
            # du robot vers le but officiel (voir RetryPolicy).
            self._metrics.add_failure(self._last_cause)
            self._note_keeper_failure(self._last_cause, self._last_immediate)
            blocked, costmap = False, None
            if self._last_cause not in (CAUSE_SILENT, CAUSE_TOO_FAR, CAUSE_CANCELED, CAUSE_TF) \
                    and not self._last_immediate:
                blocked, costmap = self._goal_blocked()
                if blocked:
                    self.get_logger().warn('Le but est occupé dans la costmap globale.')
            action = self._policy.on_failure(
                self._distance_to_goal(), self._last_cause, goal_blocked=blocked,
                immediate=self._last_immediate)
            if action == RESEND:
                self.get_logger().info('Suite : rien à nettoyer, nouvel envoi du but.')
                self._spin_sim(0.3)
                continue
            if action == FALLBACK:
                # La costmap doit rester intacte pendant la recherche du repli.
                if self._choose_fallback(costmap):
                    continue
                action = CLEAR_ALL
            self._clear(action)
            self._spin_sim(float(self.get_parameter('retry_pause_sec').value))
        return EXIT_FAILURE


def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    # On garde la main sur Ctrl-C et SIGTERM : rclpy fermerait sinon son contexte
    # avant qu'on ait pu envoyer une vitesse nulle au robot.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise_keyboard_interrupt)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    signal.signal(signal.SIGHUP, _raise_keyboard_interrupt)
    # La tâche tourne dans Gazebo : temps simulé par défaut, sans option à passer.
    # Une valeur donnée en ligne de commande reste prioritaire.
    overrides = []
    if not any('use_sim_time' in argument for argument in sys.argv):
        overrides.append(Parameter('use_sim_time', Parameter.Type.BOOL, True))
    node = None
    code = EXIT_FAILURE
    try:
        node = TaskSolution(parameter_overrides=overrides)
        code = node.run()
    except KeyboardInterrupt:
        pass
    except Exception as error:      # noqa: BLE001 — on veut toujours arrêter le robot
        if node is not None:
            node.get_logger().error(f'Erreur inattendue : {error!r}')
        else:
            print(f'task_solution: erreur au démarrage : {error!r}', file=sys.stderr)
    finally:
        if node is not None:
            # Un second Ctrl-C pendant le nettoyage ne doit pas l'interrompre.
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            signal.signal(signal.SIGHUP, signal.SIG_IGN)
            # Le robot s'arrête d'abord ; le rapport lit ensuite sa position
            # finale, tant que la localisation tourne encore.
            try:
                node.stop_motion()
            except Exception:           # noqa: BLE001
                pass
            try:
                node._write_report(code)
            except Exception:           # noqa: BLE001
                pass
            try:
                node.shutdown_sequence()
            except Exception as error:      # noqa: BLE001
                print(f'task_solution: erreur pendant l\'arrêt : {error!r}', file=sys.stderr)
                node.stop_bringup()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
