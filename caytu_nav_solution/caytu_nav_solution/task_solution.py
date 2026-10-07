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
from typing import Optional

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

from caytu_nav_solution.nav_math import (
    Pose2D, load_pgm_map, quaternion_from_yaw, scan_map_agreement, yaw_from_quaternion)
from caytu_nav_solution.task_params import TaskParams, load_task_params

EXIT_SUCCESS, EXIT_FAILURE, EXIT_TIMEOUT = 0, 1, 2
# Position du lidar dans base_footprint (URDF officiel) : 2,1 cm derrière le centre.
LIDAR_OFFSET_X = -0.021


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
        self.declare_parameter('retry_pause_sec', 1.0)
        self.declare_parameter('nav2_startup_timeout_sec', 180.0)   # temps réel
        # Lancement du reste de la solution par ce nœud (exigence PARC).
        self.declare_parameter('launch_bringup', True)
        self.declare_parameter('bringup_package', 'caytu_nav_bringup')
        self.declare_parameter('bringup_launch', 'solution_bringup.launch.py')
        # Replanification : if_invalid (défaut) ou periodic (variante de secours).
        self.declare_parameter('behavior_tree', 'if_invalid')
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

        self._task: TaskParams = load_task_params(
            self.get_parameter('task_params_file').value or None)
        self._global_frame = self.get_parameter('global_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._bringup: Optional[subprocess.Popen] = None
        self._start_time: Optional[Time] = None
        self._last_scan: Optional[LaserScan] = None
        self._last_feedback_log = 0.0
        self._goal_handle = None        # but Nav2 en cours, pour pouvoir l'annuler

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self._clear_clients = [
            self.create_client(
                ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap'),
            self.create_client(
                ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap'),
        ]
        self._scan_sub = self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self._on_scan,
            qos_profile_sensor_data)

        self.get_logger().info(
            f'Départ ({self._task.spawn.x:.3f}, {self._task.spawn.y:.3f}, '
            f'cap {self._task.spawn.yaw:.3f}) -> but ({self._task.goal_x:.3f}, '
            f'{self._task.goal_y:.3f}), lus dans {self._task.path}')

    # ------------------------------------------------------------- utilitaires
    def _on_scan(self, msg: LaserScan):
        self._last_scan = msg

    def _spin(self, wall_seconds: float):
        """Traite les messages ROS pendant une durée réelle donnée."""
        end = time.monotonic() + wall_seconds
        while rclpy.ok():
            remaining = end - time.monotonic()
            if remaining <= 0.0:
                break
            rclpy.spin_once(self, timeout_sec=min(0.05, remaining))

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
            return self._write_blank_map()
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
            return self._write_blank_map()

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
            return self._write_blank_map()
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
            f'behavior_tree:={self.get_parameter("behavior_tree").value}',
            f'task_params_file:={self._task.path}',
        ]
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

    def shutdown_sequence(self):
        """Arrêt ordonné, quel que soit le motif (arrivée, échec, Ctrl-C)."""
        if rclpy.ok() and self._goal_handle is not None:
            # Nav2 doit cesser de commander le robot avant qu'on l'arrête.
            future = self._goal_handle.cancel_goal_async()
            end = time.monotonic() + 1.0
            while rclpy.ok() and not future.done() and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
            self._goal_handle = None
        self.stop_robot()
        self.stop_bringup()
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

    def _wait_for_nav2(self) -> bool:
        """Attend que Nav2 accepte des buts et que le robot soit localisé."""
        timeout = float(self.get_parameter('nav2_startup_timeout_sec').value)
        start = last_log = time.monotonic()
        while rclpy.ok():
            self._spin(0.2)
            server = self._nav_client.server_is_ready()
            localized = self._robot_pose() is not None
            if server and localized:
                self.get_logger().info(
                    f'Nav2 prêt et robot localisé après {time.monotonic() - start:.1f} s réelles.')
                return True
            if not self._bringup_alive():
                self.get_logger().error('Le bringup s\'est arrêté pendant le démarrage.')
                return False
            if self._time_left() <= 0.0 or time.monotonic() - start > timeout:
                self.get_logger().error('Nav2 n\'est pas devenu disponible à temps.')
                return False
            if time.monotonic() - last_log > 5.0:
                last_log = time.monotonic()
                self.get_logger().info(
                    f'Attente de Nav2 (serveur d\'action : {"oui" if server else "non"}, '
                    f'localisation : {"oui" if localized else "non"})...')
        return False

    def _goal_message(self) -> NavigateToPose.Goal:
        pose = self._robot_pose() or self._task.spawn
        heading = math.atan2(self._task.goal_y - pose.y, self._task.goal_x - pose.x)
        goal = PoseStamped()
        goal.header.frame_id = self._global_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = self._task.goal_x
        goal.pose.position.y = self._task.goal_y
        qx, qy, qz, qw = quaternion_from_yaw(heading)
        goal.pose.orientation.x, goal.pose.orientation.y = qx, qy
        goal.pose.orientation.z, goal.pose.orientation.w = qz, qw
        message = NavigateToPose.Goal()
        message.pose = goal
        return message

    def _on_feedback(self, feedback_msg):
        now = time.monotonic()
        if now - self._last_feedback_log < 5.0:
            return
        self._last_feedback_log = now
        remaining = getattr(feedback_msg.feedback, 'distance_remaining', float('nan'))
        self.get_logger().info(
            f'En route : {remaining:.2f} m restants, {self._elapsed():.0f} s écoulées.')

    def _clear_costmaps(self):
        for client in self._clear_clients:
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())

    def _arrived(self) -> bool:
        distance = self._distance_to_goal()
        return distance is not None and distance <= float(
            self.get_parameter('success_radius').value)

    def _navigate_once(self) -> str:
        """Un envoi du but.

        Retourne 'success', 'retry' (échec de Nav2), 'rejected' (Nav2 pas encore
        actif), 'timeout' ou 'abort' (bringup arrêté).
        """
        send_future = self._nav_client.send_goal_async(
            self._goal_message(), feedback_callback=self._on_feedback)
        while rclpy.ok() and not send_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._time_left() <= 0.0:
                return 'timeout'
            if not self._bringup_alive():
                return 'abort'
        goal_handle = send_future.result() if send_future.done() else None
        if goal_handle is None or not goal_handle.accepted:
            return 'rejected'
        self._goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._time_left() <= 0.0:
                self.get_logger().error('Limite de temps atteinte : annulation du but.')
                cancel_future = goal_handle.cancel_goal_async()
                end = time.monotonic() + 2.0
                while rclpy.ok() and not cancel_future.done() and time.monotonic() < end:
                    rclpy.spin_once(self, timeout_sec=0.05)
                return 'timeout'
            if not self._bringup_alive():
                return 'abort'

        self._goal_handle = None
        result = result_future.result() if result_future.done() else None
        status = result.status if result is not None else GoalStatus.STATUS_UNKNOWN
        if status == GoalStatus.STATUS_SUCCEEDED:
            distance = self._distance_to_goal()
            limit = float(self.get_parameter('arrival_recheck_radius').value)
            if distance is not None and distance > limit:
                self.get_logger().warn(
                    f'Nav2 annonce l\'arrivée mais le robot est à {distance:.2f} m du but : '
                    'nouvel envoi.')
                return 'retry'
            return 'success'
        if self._arrived():
            return 'success'
        self.get_logger().warn(f'Nav2 a interrompu la navigation (statut {status}).')
        return 'retry'

    # ------------------------------------------------------------------- tâche
    def run(self) -> int:
        self._wait_for_clock()
        map_yaml = self._choose_map()
        self.destroy_subscription(self._scan_sub)      # le scan ne sert plus ici
        self._start_bringup(map_yaml)
        if not self._wait_for_nav2():
            return EXIT_FAILURE

        attempt = 0
        rejections = 0
        while rclpy.ok():
            if self._arrived():
                outcome = 'success'
            elif self._time_left() <= 0.0:
                outcome = 'timeout'
            else:
                if rejections == 0:
                    attempt += 1
                    distance = self._distance_to_goal()
                    self.get_logger().info(
                        f'Envoi du but, tentative {attempt}'
                        + (f' ({distance:.2f} m à parcourir).' if distance is not None else '.'))
                outcome = self._navigate_once()

            if outcome == 'rejected':
                # Nav2 existe mais n'est pas encore actif : on insiste, sans
                # toucher aux costmaps ni compter une tentative.
                if rejections % 10 == 0:
                    self.get_logger().info('Nav2 n\'accepte pas encore de but, nouvel essai...')
                rejections += 1
                self._spin(0.5)
                continue
            rejections = 0

            if outcome == 'success':
                self.stop_robot()
                distance = self._distance_to_goal()
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

            # Échec récupérable : on repart avec des costmaps propres.
            self._clear_costmaps()
            self._spin(float(self.get_parameter('retry_pause_sec').value))
        return EXIT_FAILURE


def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    # On garde la main sur Ctrl-C et SIGTERM : rclpy fermerait sinon son contexte
    # avant qu'on ait pu envoyer une vitesse nulle au robot.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise_keyboard_interrupt)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
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
