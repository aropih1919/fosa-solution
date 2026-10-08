#!/usr/bin/env python3
"""Stabilise le chemin : le robot garde la route choisie pour contourner un obstacle.

Ce nœud se place entre l'arbre de comportement et le planificateur de Nav2 :

    arbre  --compute_path_stable-->  path_keeper  --compute_path_to_pose-->  planificateur

À chaque demande (2 Hz), il fait calculer un chemin au planificateur. Si ce
chemin suit la même route que le chemin en cours, il est transmis tel quel : le
chemin s'ajuste en continu, comme sans ce nœud. S'il passe par une autre route
(de l'autre côté d'une table, par exemple), il n'est adopté que s'il est
nettement meilleur ou si la route en cours est coupée (voir path_choice.py).
L'arbre et le contrôleur ne voient aucune différence : ils reçoivent un chemin.

En cas de doute (costmap pas encore reçue, planificateur muet), le nœud rend
simplement ce que répond le planificateur. task_solution.py n'utilise l'arbre
qui passe par ce nœud que si le nœud est joignable.
"""

import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from caytu_nav_solution.path_choice import (
    BETTER, BLOCKED, OFF_PATH, TOO_CLOSE, CostGrid, RouteKeeper, as_points)


class PathKeeper(Node):

    def __init__(self):
        super().__init__('path_keeper')
        self.declare_parameter('action_name', 'compute_path_stable')
        self.declare_parameter('planner_action', 'compute_path_to_pose')
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        # Gain minimal (mètres équivalents) pour changer de route.
        self.declare_parameter('min_gain', 1.5)
        # Deux chemins qui ne s'écartent pas de plus que cela suivent la même route.
        self.declare_parameter('same_route_dist', 1.0)
        # Au-delà de cet écart (m), le robot a quitté le chemin gardé.
        self.declare_parameter('max_offset', 0.50)
        # Délai minimal (temps du nœud) entre deux changements de route pour un
        # simple gain ; une route coupée est abandonnée sans attendre.
        self.declare_parameter('min_dwell_sec', 5.0)
        # Prix d'un pivot (mètres équivalents par radian) : environ la distance
        # que le robot aurait parcourue pendant le temps du pivot.
        self.declare_parameter('turn_cost', 1.0)
        # Doit rester égal à cost_travel_multiplier du planificateur.
        self.declare_parameter('cost_travel_multiplier', 3.0)
        # Sans demande pendant ce délai (temps du nœud), on repart de zéro : une
        # récupération (pivot, recul) a eu lieu entre-temps.
        self.declare_parameter('reset_after_sec', 2.0)
        # Une costmap plus vieille que cela (temps du nœud) n'est pas utilisée.
        self.declare_parameter('costmap_max_age_sec', 2.0)
        # Délai maximal (temps réel) accordé au planificateur.
        self.declare_parameter('planner_timeout_sec', 5.0)

        self._min_gain = float(self.get_parameter('min_gain').value)
        self._costmap_max_age = float(self.get_parameter('costmap_max_age_sec').value)
        self._timeout = float(self.get_parameter('planner_timeout_sec').value)
        self._keeper = RouteKeeper(
            min_gain=self._min_gain,
            same_route_dist=float(self.get_parameter('same_route_dist').value),
            max_offset=float(self.get_parameter('max_offset').value),
            min_dwell=float(self.get_parameter('min_dwell_sec').value),
            reset_after=float(self.get_parameter('reset_after_sec').value),
            multiplier=float(self.get_parameter('cost_travel_multiplier').value),
            turn_cost=float(self.get_parameter('turn_cost').value))

        self._lock = threading.Lock()
        self._grid = None               # (CostGrid, instant du message)
        # Numéro de la demande la plus récente. Comme le serveur du planificateur
        # de Nav2, on ne sert qu'une demande à la fois : une demande plus récente
        # fait abandonner l'ancienne, que l'arbre n'attend plus.
        self._latest = 0

        group = ReentrantCallbackGroup()
        self._client = ActionClient(
            self, ComputePathToPose, self.get_parameter('planner_action').value,
            callback_group=group)
        self.create_subscription(
            OccupancyGrid, self.get_parameter('costmap_topic').value, self._on_costmap,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=group)
        self._server = ActionServer(
            self, ComputePathToPose, self.get_parameter('action_name').value,
            execute_callback=self._execute,
            goal_callback=lambda request: GoalResponse.ACCEPT,
            cancel_callback=lambda handle: CancelResponse.ACCEPT,
            callback_group=group)
        self.get_logger().info(
            f'Chemin stabilisé : changement de route seulement pour un gain de plus de '
            f'{self._min_gain:.1f} m ou si la route suivie est coupée.')

    # --------------------------------------------------------------- entrées
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_costmap(self, msg: OccupancyGrid):
        info = msg.info
        grid = CostGrid.from_flat(
            msg.data, info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            self._grid = (grid, stamp)

    # ----------------------------------------------------------- planificateur
    def _superseded(self, number: int) -> bool:
        with self._lock:
            return number != self._latest

    def _wait(self, event: threading.Event, goal_handle, deadline: float, number: int) -> bool:
        """Attend `event` ; faux si le délai est dépassé, la demande annulée ou remplacée."""
        while not event.wait(0.005):
            if time.monotonic() > deadline or goal_handle.is_cancel_requested \
                    or self._superseded(number) or not rclpy.ok():
                return False
        return True

    def _ask_planner(self, goal_handle, number: int):
        """Transmet la demande au planificateur. Retourne son résultat ou None."""
        if not self._client.server_is_ready():
            return None
        deadline = time.monotonic() + self._timeout
        accepted = threading.Event()
        send = self._client.send_goal_async(goal_handle.request)
        send.add_done_callback(lambda future: accepted.set())
        if not self._wait(accepted, goal_handle, deadline, number):
            return None
        handle = send.result()
        if handle is None or not handle.accepted:
            return None
        finished = threading.Event()
        pending = handle.get_result_async()
        pending.add_done_callback(lambda future: finished.set())
        if not self._wait(finished, goal_handle, deadline, number):
            handle.cancel_goal_async()          # l'arbre n'attend plus ce chemin
            return None
        return pending.result()

    # ---------------------------------------------------------------- décision
    def _execute(self, goal_handle):
        request = goal_handle.request
        result = ComputePathToPose.Result()
        with self._lock:
            self._latest += 1
            number = self._latest
        try:
            wrapped = self._ask_planner(goal_handle, number)
        except Exception as error:      # noqa: BLE001 — le planificateur a pu disparaître
            self.get_logger().warn(f'Planificateur injoignable ({error!r}).')
            wrapped = None

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return result
        if self._superseded(number):
            # Une demande plus récente est en cours : celle-ci n'attend plus rien.
            result.error_code = ComputePathToPose.Result.UNKNOWN
            goal_handle.abort()
            return result
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED \
                or not wrapped.result.path.poses:
            # Échec transmis tel quel : l'arbre décide de la récupération d'après
            # le code d'erreur, exactement comme sans ce nœud.
            with self._lock:
                self._keeper.forget()
            if wrapped is not None:
                result.error_code = wrapped.result.error_code or ComputePathToPose.Result.UNKNOWN
                if hasattr(result, 'error_msg'):
                    result.error_msg = getattr(wrapped.result, 'error_msg', '')
            else:
                result.error_code = ComputePathToPose.Result.TIMEOUT
            goal_handle.abort()
            return result

        new_path = wrapped.result.path
        new_points = as_points(
            [(pose.pose.position.x, pose.pose.position.y) for pose in new_path.poses])
        goal_xy = (request.goal.pose.position.x, request.goal.pose.position.y)
        now = self._now()
        with self._lock:
            grid = None
            if self._grid is not None and abs(now - self._grid[1]) <= self._costmap_max_age:
                grid = self._grid[0]
            choice, poses = self._keeper.update(now, new_points, new_path.poses, grid, goal_xy)
            holds = self._keeper.holds
        result.path.header = new_path.header
        result.path.poses = poses

        if choice.use_new and choice.reason in (BLOCKED, TOO_CLOSE, BETTER, OFF_PATH):
            gain = (f', gain {choice.gain:.1f} m, pivot {math.degrees(choice.turn):.0f}°'
                    if choice.gain is not None else '')
            self.get_logger().info(f'Changement de route : {choice.reason}{gain}.')
        elif not choice.use_new and holds % 10 == 1:
            gain = f'gain {choice.gain:.1f} m' if choice.gain is not None else 'gain inconnu'
            self.get_logger().info(
                f'Autre route proposée ({gain}) : route suivie gardée ({holds} fois).')
        result.planning_time = wrapped.result.planning_time
        result.error_code = ComputePathToPose.Result.NONE
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PathKeeper()
    # Une demande servie à la fois ; les autres fils traitent les réponses du
    # planificateur et la costmap pendant qu'elle attend.
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Arrêt silencieux : le launch envoie SIGINT à la fermeture.
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
