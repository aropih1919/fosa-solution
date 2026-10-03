#!/usr/bin/env python3
"""
nav_monitor.py — Moniteur Nav2 pour terminal

Affiche en temps réel l'état de la navigation Nav2 :
  - réception d'un goal (via /goal_pose ou action NavigateToPose)
  - recherche de chemin (planner_server /plan)
  - chemin trouvé / échec
  - avancement (distance_remaining, cmd_vel)
  - recovery (Spin / Backup / ClearCostmap détecté via cmd_vel angulaire pur)
  - arrivée / échec final

Utilisation :
  ros2 run caytu_nav_solution nav_monitor
  ros2 run caytu_nav_solution nav_monitor --ros-args -p verbose:=true

Topics écoutés :
  - /goal_pose (geometry_msgs/PoseStamped)
  - /plan (nav_msgs/Path) — publié par planner_server
  - /navigate_to_pose/_action/status (action_msgs/GoalStatusArray)
  - /navigate_to_pose/_action/feedback (nav2_msgs/action/NavigateToPose_FeedbackMessage)
  - /robot_base_controller/cmd_vel_unstamped (geometry_msgs/Twist)
  - /scan_filtered (sensor_msgs/LaserScan) — juste pour heartbeat coût map
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from action_msgs.msg import GoalStatusArray
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool

# QoS map / transient local pour /plan parfois
_qos_transient = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=10,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)


class NavMonitor(Node):
    def __init__(self):
        super().__init__('nav_monitor')

        self.declare_parameter('verbose', False)
        self.declare_parameter('cmd_vel_topic', '/robot_base_controller/cmd_vel_unstamped')
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('goal_topic', '/goal_pose')

        self.verbose = self.get_parameter('verbose').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        plan_topic = self.get_parameter('plan_topic').value
        goal_topic = self.get_parameter('goal_topic').value

        # État interne
        self._last_goal = None
        self._last_plan_len = 0.0
        self._last_plan_points = 0
        self._last_plan_time = 0.0
        self._last_status = None
        self._last_distance = None
        self._last_cmd = None
        self._spinning_since = None
        self._blocked_since = None
        self._goal_active = False

        # Throttle
        self._last_distance_log = 0.0
        self._last_cmd_log = 0.0

        # ANSI couleurs (désactivées si non tty)
        self._C = {
            'reset': '\033[0m',
            'cyan': '\033[36m',
            'green': '\033[32m',
            'yellow': '\033[33m',
            'red': '\033[31m',
            'mag': '\033[35m',
            'dim': '\033[2m',
            'bold': '\033[1m',
        }

        # Subscriptions
        self.create_subscription(PoseStamped, goal_topic, self._on_goal, 10)
        # /plan est parfois transient_local sur map
        try:
            self.create_subscription(Path, plan_topic, self._on_plan, 10)
        except Exception:
            self.create_subscription(Path, plan_topic, self._on_plan, _qos_transient)

        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self._on_status, 10)
        # Feedback : topic caché de l'action
        try:
            from nav2_msgs.action import NavigateToPose
            # Le type feedback message est publié sur /navigate_to_pose/_action/feedback
            # On s'abonne en générique via rclpy (on importe le msg)
            from nav2_msgs.action import NavigateToPose_FeedbackMessage  # type: ignore
            self.create_subscription(NavigateToPose_FeedbackMessage, '/navigate_to_pose/_action/feedback', self._on_feedback, 10)
        except Exception as e:
            self.get_logger().warn(f'Feedback NavigateToPose non abonné: {e}')

        self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, 10)
        # Fallback si topic sans namespace
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel_fallback, 10)

        # Optionnel : surveiller localization_ready
        self.create_subscription(Bool, '/localization_ready', self._on_localization, 10)

        self.get_logger().info(
            self._fmt('cyan', '[NAV-MONITOR] démarré — en écoute sur:')
            + f"\n  • goal:   {goal_topic}"
            + f"\n  • plan:   {plan_topic}"
            + f"\n  • action: /navigate_to_pose"
            + f"\n  • cmd_vel:{cmd_vel_topic}"
            + (f"\n  • verbose: true" if self.verbose else "")
        )
        self._log('dim', 'En attente d\'un goal (RViz 2D Goal Pose, task_solution, ou ros2 action)...')

    # ------------------------------------------------------------------ utils

    def _fmt(self, color, msg):
        c = self._C.get(color, '')
        r = self._C['reset']
        return f"{c}{msg}{r}"

    def _log(self, color, msg, throttle_sec=0.0):
        # log ROS + print terminal avec timestamp
        stamp = time.strftime('%H:%M:%S', time.localtime())
        prefix = {
            'cyan': '[NAV]',
            'green': '[NAV]',
            'yellow': '[NAV]',
            'red': '[NAV]',
            'mag': '[NAV]',
            'dim': '[NAV]',
        }.get(color, '[NAV]')
        line = f"{stamp} {prefix} {msg}"
        if color == 'green':
            self.get_logger().info(line)
        elif color == 'yellow':
            self.get_logger().warn(line)
        elif color == 'red':
            self.get_logger().error(line)
        else:
            self.get_logger().info(line)

    def _path_length(self, path: Path) -> float:
        if not path.poses or len(path.poses) < 2:
            return 0.0
        s = 0.0
        for i in range(1, len(path.poses)):
            a = path.poses[i-1].pose.position
            b = path.poses[i].pose.position
            s += math.hypot(b.x - a.x, b.y - a.y)
        return s

    # ---------------------------------------------------------------- callbacks

    def _on_goal(self, msg: PoseStamped):
        self._last_goal = msg
        self._goal_active = True
        x = msg.pose.position.x
        y = msg.pose.position.y
        # yaw depuis quaternion
        q = msg.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self._log('cyan', '')
        self._log('cyan', '┌─────────────────────────────────────────────')
        self._log('cyan', f'│ 🎯 NOUVEAU GOAL REÇU  x={x:.2f}  y={y:.2f}  yaw={math.degrees(yaw):.0f}°  frame={msg.header.frame_id}')
        self._log('cyan', '└─────────────────────────────────────────────')
        self._log('yellow', '  🔍 À la recherche de chemin... (ComputePathToPose → planner_server)')
        self._last_plan_time = time.time()

    def _on_plan(self, msg: Path):
        now = time.time()
        n = len(msg.poses)
        length = self._path_length(msg)
        # Throttle : éviter spam de replanification 1 Hz -> log seulement si changement significatif
        if n == 0:
            self._log('red', '  ❌ Planner: aucun chemin trouvé (path vide) — costmap bloquée ou goal inatteignable')
            self._log('dim', '     → Recovery possible: ClearCostmap + Spin (cf. BT navigate_bounded_recovery.xml)')
            return
        # Si même taille que précédent et <2s depuis dernier, throttle
        if n == self._last_plan_points and abs(length - self._last_plan_len) < 0.05 and (now - self._last_plan_time) < 1.5:
            return
        self._last_plan_points = n
        self._last_plan_len = length
        self._last_plan_time = now
        elapsed = now - (self._last_goal.header.stamp.sec if self._last_goal else now) if self._last_goal else 0
        self._log('green', f'  ✅ Chemin trouvé: {n} points, longueur {length:.2f} m  (replan {elapsed:.1f}s après goal)')
        self._log('dim', '     → Controller: FollowPath (DWB) va suivre le chemin — surveille /cmd_vel')
        if length > 20:
            self._log('yellow', '     ⚠️  Chemin long (>20m) — la vitesse max 0.40 m/s implique ~50s minimum, vérifier vitesse')

    def _on_status(self, msg: GoalStatusArray):
        if not msg.status_list:
            return
        # Le dernier status est le plus récent
        s = msg.status_list[-1]
        status = s.status  # action_msgs/GoalStatus constants
        # 1=EXECUTING, 2=CANCELING, 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
        names = {1: 'EXECUTING', 2: 'CANCELING', 4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
        txt = names.get(status, str(status))
        if txt == self._last_status:
            return
        self._last_status = txt
        if status == 1:
            if not self._goal_active:
                self._goal_active = True
                self._log('cyan', '┌─────────────────────────────────────────────')
                self._log('cyan', '│ 🚀 NAVIGATION ACTIVE (bt_navigator EXECUTING)')
                self._log('cyan', '└─────────────────────────────────────────────')
                self._log('yellow', '  🔍 ComputePathToPose en cours...')
            else:
                self._log('dim', '  ▶ bt_navigator: EXECUTING (Pipe: ComputePath → FollowPath, replan 1 Hz)')
        elif status == 4:
            self._goal_active = False
            self._log('green', '┌─────────────────────────────────────────────')
            self._log('green', '│ 🏁 GOAL ATTEINT — SUCCEEDED')
            self._log('green', '└─────────────────────────────────────────────')
            self._spinning_since = None
            self._blocked_since = None
        elif status == 6:
            self._goal_active = False
            self._log('red', '  ❌ Navigation ABORTED — échec planner ou controller après recoveries')
            self._log('dim', '     → Causes fréquentes: inflation_radius trop grand (0.45), scan bloqué, goal dans obstacle')
        elif status == 5:
            self._goal_active = False
            self._log('yellow', '  ⏹ Navigation CANCELED (timeout 600s ou annulation utilisateur)')

    def _on_feedback(self, msg):
        # msg est NavigateToPose_FeedbackMessage
        try:
            fb = msg.feedback
            dist = getattr(fb, 'distance_remaining', None)
            if dist is None:
                return
            now = time.time()
            # log distance toutes les 3s seulement
            if now - self._last_distance_log < 3.0 and abs((dist or 0) - (self._last_distance or 0)) < 0.10:
                return
            self._last_distance = dist
            self._last_distance_log = now
            if dist is not None and dist > 0.05:
                self._log('dim', f'  📏 Distance restante: {dist:.2f} m')
                if self.verbose:
                    eta = dist / 0.35 if dist else 0
                    self.get_logger().info(f'  [verbose] ETA ~{eta:.0f}s à 0.35 m/s')
        except Exception:
            pass

    def _on_cmd_vel(self, msg: Twist):
        now = time.time()
        lin = msg.linear.x
        ang = msg.angular.z
        # Détecter rotation sur place (lin ~0, ang !=0)
        is_spinning = abs(lin) < 0.02 and abs(ang) > 0.15
        is_moving = abs(lin) > 0.05 or abs(ang) > 0.05
        is_blocked = abs(lin) < 0.01 and abs(ang) < 0.01 and self._goal_active

        # Log spin
        if is_spinning:
            if self._spinning_since is None:
                self._spinning_since = now
                self._log('yellow', f'  🔄 ROTATION SUR PLACE détectée  ang={ang:.2f} rad/s  (recovery Spin 1.57 rad ou RotateToGoal)')
                self._log('dim', '     → Si répétée: progress_checker bloqué ou local_costmap saturée → ClearCostmap + Wait 2s prévu par BT')
            elif now - self._spinning_since > 6.0 and now - self._last_cmd_log > 4.0:
                self._log('yellow', f'  🔄 Rotation toujours en cours ({now - self._spinning_since:.0f}s) — vérifier si robot tourne en boucle')
                self._last_cmd_log = now
        else:
            if self._spinning_since is not None and now - self._spinning_since > 0.8:
                dur = now - self._spinning_since
                self._log('dim', f'  ↩ Fin rotation ({dur:.1f}s)')
            self._spinning_since = None

        # Log blocage (pas de cmd_vel alors que goal actif) → progress_checker
        if is_blocked:
            if self._blocked_since is None:
                self._blocked_since = now
            elif now - self._blocked_since > 4.0 and now - self._last_cmd_log > 3.0:
                self._log('red', '  ⏸️  Robot à l\'arrêt depuis 4s alors que goal actif — progress_checker va déclarer échec')
                self._log('dim', '     → Causes: DWB bloqué par inflation 0.45, obstacle fantôme, ou vitesse trop faible')
                self._last_cmd_log = now
        else:
            self._blocked_since = None

        # Log avancement normal (throttled 2s)
        if is_moving and not is_spinning and now - self._last_cmd_log > 2.0:
            if abs(lin) > 0.02:
                self._log('dim', f'  🚀 Avance  lin={lin:.2f} m/s  ang={ang:.2f} rad/s')
            self._last_cmd_log = now

        self._last_cmd = msg

    def _on_cmd_vel_fallback(self, msg: Twist):
        # Ignorer si le topic principal a déjà reçu un msg récemment
        if self._last_cmd is not None and time.time() - self._last_cmd_log < 1.0:
            return
        self._on_cmd_vel(msg)

    def _on_localization(self, msg: Bool):
        # Juste heartbeat si verbose
        if self.verbose:
            state = 'OK' if msg.data else 'INCERTAINE'
            self._log('dim', f'  [verbose] localization_ready={state}')


def main(args=None):
    rclpy.init(args=args)
    node = NavMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
