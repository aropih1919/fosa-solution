#!/usr/bin/env python3
"""Moniteur de navigation : journal d'evenements lisible dans un terminal.

Outil de mise au point, jamais lance par la solution. Chaque callback
transmet le temps du noeud au detecteur pur (nav_events) puis affiche
les evenements sous la forme `[ETIQUETTE] texte`, avec couleurs ANSI
seulement sur terminal interactif.
"""

import math
import sys

import numpy as np
import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Pose, PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from caytu_nav_solution.nav_events import NavEventDetector

try:
    from ros_gz_interfaces.msg import Contacts
except ImportError:
    Contacts = None

try:
    from nav2_msgs.msg import CollisionMonitorState
except ImportError:
    CollisionMonitorState = None

CONTACT_TOPICS = (
    "/base_collisions",
    "/left_wheel_collisions",
    "/right_wheel_collisions",
    "/top_chassis_collisions",
)

# Couleurs desactivees hors terminal pour garder les journaux lisibles.
_COLORS = {
    "info": "\033[36m",
    "warn": "\033[33m",
    "error": "\033[31m",
}
_RESET = "\033[0m"


def _count_valid(msg: LaserScan) -> int:
    """Nombre de rayons exploitables, en une passe numpy (O(M))."""
    ranges = np.asarray(msg.ranges, dtype=np.float64)
    ok = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
    return int(np.count_nonzero(ok))


def _path_length(msg: Path) -> float:
    """Longueur du chemin, calculee une fois par chemin recu (O(P))."""
    if len(msg.poses) < 2:
        return 0.0
    xs = np.fromiter((p.pose.position.x for p in msg.poses), dtype=np.float64)
    ys = np.fromiter((p.pose.position.y for p in msg.poses), dtype=np.float64)
    return float(np.hypot(np.diff(xs), np.diff(ys)).sum())


def _yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class NavMonitor(Node):
    """Abonnements seuls : ne publie rien et ne commande jamais le robot."""

    def __init__(self) -> None:
        super().__init__("nav_monitor")
        self.declare_parameter("cmd_vel_topic", "/robot_base_controller/cmd_vel_unstamped")
        self.declare_parameter("log_file", "")
        self._cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._log_path = str(self.get_parameter("log_file").value)
        self._log_stream = None
        if self._log_path:
            self._log_stream = open(self._log_path, "a", encoding="utf-8")
        self._use_color = sys.stdout.isatty()
        self._detector = NavEventDetector()
        self._pose = None
        self._truth = None
        self._cmd = (0.0, 0.0)
        self._tilt = None
        self._counts = {"lidar": None, "top": None, "bottom": None, "filtered": None}
        self._contacts = 0
        self._contact_names = set()
        self._goal = self._load_goal()
        self._cm_state = None

        self.create_subscription(Path, "/plan", self._on_plan, 10)
        self.create_subscription(
            GoalStatusArray, "/navigate_to_pose/_action/status", self._on_status, 10)
        self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            "/navigate_to_pose/_action/feedback", self._on_feedback, 10)
        self.create_subscription(Twist, self._cmd_topic, self._on_cmd, 10)
        self.create_subscription(PoseStamped, "/localization_pose", self._on_pose, 10)
        self.create_subscription(Vector3Stamped, "/robot_tilt", self._on_tilt, 10)
        self.create_subscription(
            Vector3Stamped, "/localization_correction", self._on_correction, 10)
        self.create_subscription(
            LaserScan, "/scan_filtered", self._on_filtered, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, "/scan_clean", self._on_clean, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, "/top_camera_scan", self._on_top, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, "/bottom_camera_scan", self._on_bottom, qos_profile_sensor_data)
        if Contacts is not None:
            for topic in CONTACT_TOPICS:
                self.create_subscription(Contacts, topic, self._on_contacts, 10)
        self.create_subscription(Pose, "/sitoe_robot/pose", self._on_truth, 10)
        if CollisionMonitorState is not None:
            self.create_subscription(
                CollisionMonitorState, "/collision_monitor_state", self._on_cm, 10)
        self.create_timer(2.0, self._on_timer)

    def _load_goal(self):
        """But officiel pour la ligne d'etat, sans coordonnee en dur."""
        try:
            from caytu_nav_solution.task_params import load_task_params
            task = load_task_params(None)
            return (task.goal_x, task.goal_y)
        except Exception:
            return None

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _emit(self, events) -> None:
        for level, label, text in events:
            line = f"[{label}] {text}"
            if self._use_color and level in _COLORS:
                line = f"{_COLORS[level]}{line}{_RESET}"
            if level == "warn":
                self.get_logger().warn(line)
            elif level == "error":
                self.get_logger().error(line)
            else:
                self.get_logger().info(line)
            if self._log_stream is not None:
                self._log_stream.write(f"[{label}] {text}\n")
                self._log_stream.flush()

    def _on_status(self, msg: GoalStatusArray) -> None:
        if not msg.status_list:
            return
        self._emit(self._detector.on_status(msg.status_list[-1].status, self._now()))

    def _on_plan(self, msg: Path) -> None:
        length = _path_length(msg)
        self._emit(self._detector.on_plan(len(msg.poses), length, self._now()))

    def _on_feedback(self, msg) -> None:
        dist = getattr(msg.feedback, "distance_remaining", None)
        if dist is None:
            return
        self._emit(self._detector.on_feedback(float(dist), self._now()))

    def _on_cmd(self, msg: Twist) -> None:
        self._cmd = (msg.linear.x, msg.angular.z)
        self._emit(self._detector.on_cmd(msg.linear.x, msg.angular.z, self._now()))

    def _on_pose(self, msg: PoseStamped) -> None:
        p = msg.pose
        self._pose = (p.position.x, p.position.y, _yaw_of(p.orientation))

    def _on_truth(self, msg: Pose) -> None:
        self._truth = (msg.position.x, msg.position.y)

    def _on_tilt(self, msg: Vector3Stamped) -> None:
        self._tilt = (msg.vector.y, msg.header.frame_id)

    def _on_correction(self, msg: Vector3Stamped) -> None:
        self._emit([("info", "CORR",
                     f"recalage cumul x={msg.vector.x:+.3f} y={msg.vector.y:+.3f} "
                     f"q={msg.vector.z:.2f}")])

    def _on_filtered(self, msg: LaserScan) -> None:
        self._counts["filtered"] = _count_valid(msg)

    def _on_clean(self, msg: LaserScan) -> None:
        self._counts["lidar"] = _count_valid(msg)

    def _on_top(self, msg: LaserScan) -> None:
        self._counts["top"] = _count_valid(msg)

    def _on_bottom(self, msg: LaserScan) -> None:
        self._counts["bottom"] = _count_valid(msg)

    def _on_contacts(self, msg) -> None:
        for contact in msg.contacts:
            names = (contact.collision1.name, contact.collision2.name)
            if any("floor" in n or "ground" in n for n in names):
                continue
            self._contacts += 1
            self._contact_names.update(names)

    def _on_cm(self, msg) -> None:
        self._cm_state = str(getattr(msg, "action_type", ""))

    def _status_line(self) -> str:
        parts = []
        if self._pose is None:
            parts.append("pose --")
        else:
            x, y, yaw = self._pose
            parts.append(f"pose ({x:+.2f}, {y:+.2f}, {math.degrees(yaw):+.0f}deg)")
            if self._goal is not None:
                parts.append(f"but a {math.hypot(self._goal[0] - x, self._goal[1] - y):.2f} m")
            if self._truth is not None:
                parts.append(f"ecart {math.hypot(self._truth[0] - x, self._truth[1] - y):.2f} m")
            else:
                parts.append('ecart n/d')
        v, w = self._cmd
        parts.append(f"v={v:+.2f} w={w:+.2f}")
        parts.append("lidar {} top {} bottom {}".format(
            self._counts["lidar"], self._counts["top"], self._counts["bottom"]))
        if self._counts["filtered"] is not None and self._counts["lidar"] is not None:
            parts.append(f"sol_lidar:{self._counts['filtered'] - self._counts['lidar']}")
        if self._tilt is not None:
            pitch, frame = self._tilt
            src = "mesure" if frame == "imu" else "valeur par defaut"
            parts.append(f"tangage {math.degrees(pitch):.2f}deg ({src})")
        parts.append(f"replans:{self._detector.replans}")
        parts.append(f"contacts:{self._contacts}")
        return " | ".join(parts)

    def _on_timer(self) -> None:
        line = self._status_line()
        self.get_logger().info(line)
        if self._log_stream is not None:
            self._log_stream.write(line + "\n")
            self._log_stream.flush()


def main(args=None):
    rclpy.init(args=args)
    node = NavMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node._log_stream is not None:
                node._log_stream.close()
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
