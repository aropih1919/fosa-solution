#!/usr/bin/env python3
"""Session de cartographie manuelle : note le départ et sauvegarde la carte.

Lancé par caytu_nav_bringup/launch/mapping.launch.py, à côté de slam_toolbox.
  - Au début, il note la pose du robot dans `map`. Pour se localiser plus tard
    sur la carte, le robot devra repartir de ce même endroit.
  - Dans Gazebo, il note aussi le lien entre `map` et le repère du monde
    (spawn de task_params.yaml), pour pouvoir viser le cercle vert.
  - Toutes les `autosave_period_sec` secondes, il sauvegarde la carte dans
    <dossier>/auto. Un créneau qui s'arrête net ne fait donc rien perdre.
  - Le service /fosa/save_map (std_srvs/Trigger) enregistre la carte dans le
    dossier principal ; c'est ce que fait « ros2 run caytu_nav_solution save_map ».

Une sauvegarde écrit la carte au format map_server (map.pgm, map.yaml), le
graphe de slam_toolbox (map.posegraph, map.data) et fosa_map.yaml.
"""

import datetime
import os
import threading
import time
from typing import Optional, Tuple

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from slam_toolbox.srv import SerializePoseGraph
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from caytu_nav_solution.map_io import (
    AUTOSAVE_DIR, NAV_MAP, POSEGRAPH, RAW_MAP, MapInfo, default_maps_dir,
    occupancy_to_pixels, pixel_stats, prepare_map_folder, save_map_info,
    world_from_map_at_start, write_map_files)
from caytu_nav_solution.nav_math import Pose2D, lacet_depuis_quaternion
from caytu_nav_solution.task_params import load_task_params


class MappingSession(Node):

    def __init__(self):
        super().__init__('mapping_session')

        self.declare_parameter('map_name', 'carte')
        self.declare_parameter('maps_dir', '')                  # vide = ~/fosa_maps
        self.declare_parameter('autosave_period_sec', 30.0)     # 0 = jamais
        # Gazebo : noter le lien map <-> monde à partir du spawn de task_params.yaml.
        self.declare_parameter('record_world_frame', False)
        self.declare_parameter('task_params_file', '')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('serialize_service', '/slam_toolbox/serialize_map')
        self.declare_parameter('progress_period_sec', 10.0)

        maps_dir = self.get_parameter('maps_dir').value or default_maps_dir()
        self._folder, moved = prepare_map_folder(maps_dir, self.get_parameter('map_name').value)
        self._name = os.path.basename(self._folder)
        if moved:
            self.get_logger().warn(f'Une carte « {self._name} » existait : déplacée dans {moved}.')
        self.get_logger().info(f'Cartes enregistrées dans {self._folder}')

        self._map_frame = self.get_parameter('map_frame').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._created = datetime.datetime.now().astimezone().isoformat(timespec='seconds')

        self._spawn: Optional[Pose2D] = None
        if self.get_parameter('record_world_frame').value:
            task = load_task_params(self.get_parameter('task_params_file').value or None)
            self._spawn = task.spawn
            self.get_logger().info(
                f'Gazebo : spawn ({task.spawn.x:.3f}, {task.spawn.y:.3f}, '
                f'cap {task.spawn.yaw:.3f}) lu dans {task.path}.')

        self._lock = threading.Lock()
        self._map: Optional[OccupancyGrid] = None
        self._start_pose: Optional[Pose2D] = None
        self._world_from_map: Optional[Pose2D] = None
        self._last_progress = 0.0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        # Les sauvegardes ne se chevauchent jamais ; l'appel au service de
        # slam_toolbox est servi par un autre fil pendant qu'on l'attend.
        self._save_group = MutuallyExclusiveCallbackGroup()
        self._client_group = ReentrantCallbackGroup()
        self._serialize = self.create_client(
            SerializePoseGraph, self.get_parameter('serialize_service').value,
            callback_group=self._client_group)

        # Volatile : compatible avec un éditeur « transient local » ou non.
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.VOLATILE)
        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value, self._on_map, qos,
            callback_group=self._client_group)
        self._start_timer = self.create_timer(0.2, self._capture_start)
        period = float(self.get_parameter('autosave_period_sec').value)
        if period > 0.0:
            self.create_timer(period, self._autosave, callback_group=self._save_group)
        self.create_service(Trigger, '/fosa/save_map', self._on_save_request,
                            callback_group=self._save_group)

        self.get_logger().info(
            'Pilotez le robot lentement (environ 0,2 m/s), revenez au point de départ, '
            'puis : ros2 run caytu_nav_solution save_map')

    # ------------------------------------------------------------- entrées
    def _lookup(self, target: str, source: str) -> Optional[Pose2D]:
        try:
            tf = self._tf_buffer.lookup_transform(target, source, Time())
        except TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return Pose2D(t.x, t.y, lacet_depuis_quaternion(q.x, q.y, q.z, q.w))

    def _capture_start(self):
        """Note la pose de départ dès que slam_toolbox publie map -> odom."""
        start = self._lookup(self._map_frame, self._base_frame)
        map_from_odom = self._lookup(self._map_frame, self._odom_frame)
        if start is None or map_from_odom is None:
            return
        with self._lock:
            self._start_pose = start
            if self._spawn is not None:
                self._world_from_map = world_from_map_at_start(self._spawn, map_from_odom)
        self._start_timer.cancel()
        self.get_logger().info(
            f'Départ noté : ({start.x:.2f}, {start.y:.2f}, cap {start.yaw:.2f}) dans '
            f'`{self._map_frame}`. Pour naviguer sur cette carte, le robot devra '
            'repartir de cet endroit.')

    def _on_map(self, msg: OccupancyGrid):
        with self._lock:
            self._map = msg
        now = time.monotonic()
        if now - self._last_progress < float(self.get_parameter('progress_period_sec').value):
            return
        self._last_progress = now
        info = msg.info
        pixels = occupancy_to_pixels(info.width, info.height, msg.data)
        stats = pixel_stats(info.width, info.height, pixels, info.resolution)
        self.get_logger().info(
            f'Carte en cours : zone explorée {stats.known_width_m:.1f} m x '
            f'{stats.known_height_m:.1f} m, {stats.known_cells * info.resolution ** 2:.1f} m² connus.')

    # --------------------------------------------------------- sauvegarde
    def _serialize_posegraph(self, prefix: str) -> Tuple[bool, str]:
        if not self._serialize.wait_for_service(timeout_sec=2.0):
            return False, 'service de slam_toolbox indisponible'
        request = SerializePoseGraph.Request()
        request.filename = prefix
        future = self._serialize.call_async(request)
        deadline = time.monotonic() + 20.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            return False, 'slam_toolbox n\'a pas répondu à temps'
        result = future.result()
        if result is None or result.result != SerializePoseGraph.Response.RESULT_SUCCESS:
            return False, 'slam_toolbox n\'a pas pu écrire le graphe'
        if not (os.path.isfile(prefix + '.posegraph') and os.path.isfile(prefix + '.data')):
            return False, 'fichiers du graphe absents après sauvegarde'
        return True, ''

    def save(self, folder: str, saved_by: str) -> Tuple[bool, str]:
        with self._lock:
            msg = self._map
            start_pose = self._start_pose
            world_from_map = self._world_from_map
        if msg is None:
            return False, 'Aucune carte reçue sur /map : slam_toolbox tourne-t-il ?'
        info = msg.info
        pixels = occupancy_to_pixels(info.width, info.height, msg.data)
        stats = pixel_stats(info.width, info.height, pixels, info.resolution)
        origin = (info.origin.position.x, info.origin.position.y,
                  lacet_depuis_quaternion(info.origin.orientation.x, info.origin.orientation.y,
                                      info.origin.orientation.z, info.origin.orientation.w))
        write_map_files(folder, RAW_MAP, info.width, info.height, pixels,
                        info.resolution, origin)
        graph_ok, graph_error = self._serialize_posegraph(os.path.join(folder, POSEGRAPH))

        # Une carte retouchée avec edit_map reste celle donnée à Nav2.
        nav_map = (NAV_MAP + '.yaml' if os.path.isfile(os.path.join(folder, NAV_MAP + '.yaml'))
                   else RAW_MAP + '.yaml')
        save_map_info(MapInfo(
            folder=folder, name=self._name, created=self._created,
            saved_by=f'{saved_by} {datetime.datetime.now().astimezone().isoformat(timespec="seconds")}',
            nav_map=nav_map, posegraph=POSEGRAPH, resolution=float(info.resolution),
            explored_m=(stats.known_width_m, stats.known_height_m),
            start_pose=start_pose, world_from_map=world_from_map))

        message = (f'Carte enregistrée dans {folder} : zone explorée '
                   f'{stats.known_width_m:.1f} m x {stats.known_height_m:.1f} m.')
        if not graph_ok:
            message += (f' ATTENTION : graphe slam_toolbox non enregistré ({graph_error}) ; '
                        'la carte servira à Nav2 mais pas à la localisation.')
        if start_pose is None:
            message += ' ATTENTION : pose de départ inconnue (TF map -> base_footprint absente).'
        return graph_ok and start_pose is not None, message

    def _autosave(self):
        with self._lock:
            ready = self._map is not None and self._start_pose is not None
        if not ready:
            # Rien d'utile à sauver avant la première carte et la pose de
            # départ (au lancement, l'horloge simulée fait aussi sauter le
            # minuteur une première fois).
            return
        ok, message = self.save(os.path.join(self._folder, AUTOSAVE_DIR), 'auto')
        self._log(ok, '[sauvegarde auto] ' + message)

    def _on_save_request(self, request, response):
        ok, message = self.save(self._folder, 'manuel')
        self._log(ok, message)
        response.success = ok
        response.message = message
        return response

    def _log(self, ok: bool, message: str):
        # rclpy refuse qu'une même ligne de code journalise à deux niveaux
        # différents : un appel par niveau.
        if ok:
            self.get_logger().info(message)
        else:
            self.get_logger().warn(message)


def main(args=None):
    rclpy.init(args=args)
    node = MappingSession()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:       # noqa: BLE001
        # Ctrl-C dans le terminal du launch : le contexte ROS est déjà fermé
        # quand l'exécuteur se réveille. Toute autre erreur est remontée.
        if rclpy.ok():
            raise
    finally:
        try:
            executor.shutdown()
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
