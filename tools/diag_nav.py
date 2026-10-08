#!/usr/bin/env python3
"""Diagnostic de navigation Fosa : une ligne toutes les 2 s (temps simulé).

Outil de mise au point, NON utilisé par la solution. Chaque ligne donne :
  - la pose estimée par odom_imu_localizer et la distance restante au but ;
  - l'écart avec la pose réelle du robot dans Gazebo (/sitoe_robot/pose), pour
    mesurer la dérive de la localisation ;
  - la commande envoyée au robot ;
  - le nombre de directions où chaque scan voit un obstacle ;
  - le nombre de contacts avec autre chose que le sol depuis la ligne précédente.

Lancer dans un terminal à part, pendant que la solution tourne :
    python3 tools/diag_nav.py --ros-args -p use_sim_time:=true | tee diag.log

Lecture rapide :
  ecart      écart entre la position estimée et la position réelle. Doit rester
             sous ~0.05 m, y compris après les virages. S'il grandit à chaque
             changement de cap (jusqu'à 0,1-0,2 m), vérifier axle_offset
             (docs/NOTES_TECHNIQUES.md, section Localisation).
  top        sur sol dégagé : 0 à 2. Des dizaines = faux obstacles de sol.
  sol_lidar  rayons du lidar retirés car ils touchent le sol.
  contacts   doit rester à 0 (critère n° 1 du barème PARC).
"""
import math
import os

import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseStamped, Twist, Vector3Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

try:                                    # présent avec ros_gz, absent sinon
    from ros_gz_interfaces.msg import Contacts
except ImportError:                     # pragma: no cover
    Contacts = None

CMD_TOPIC = '/robot_base_controller/cmd_vel_unstamped'
SCANS = {
    'lidar': '/scan_clean',
    'top': '/top_camera_scan',
    'bottom': '/bottom_camera_scan',
}
CONTACT_TOPICS = (
    '/base_collisions', '/top_chassis_collisions',
    '/left_wheel_collisions', '/right_wheel_collisions')


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def load_goal():
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(
            get_package_share_directory('parc_robot_bringup'), 'config', 'task_params.yaml')
        with open(path, encoding='utf-8') as stream:
            params = yaml.safe_load(stream)['/**']['ros__parameters']
        return float(params['goal_x']), float(params['goal_y'])
    except Exception:                   # noqa: BLE001 — outil de diagnostic
        return None


class Diag(Node):
    def __init__(self):
        super().__init__('diag_nav')
        self.goal = load_goal()
        self.estimate = None            # (x, y, yaw) dans map
        self.truth = None               # (x, y, yaw) dans Gazebo
        self.cmd = (0.0, 0.0)
        self.tilt = None
        self.obstacles = {k: None for k in SCANS}
        self.msgs = {k: 0 for k in SCANS}
        self.raw_valid = None
        self.contacts = 0
        self.contact_names = set()
        self.max_gap = 0.0

        self.create_subscription(PoseStamped, '/localization_pose', self._on_estimate, 10)
        self.create_subscription(Pose, '/sitoe_robot/pose', self._on_truth, 10)
        self.create_subscription(Twist, CMD_TOPIC, self._on_cmd, 10)
        self.create_subscription(Vector3Stamped, '/robot_tilt', self._on_tilt, 10)
        for name, topic in SCANS.items():
            self.create_subscription(
                LaserScan, topic, lambda msg, n=name: self._on_scan(n, msg),
                qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, '/scan_filtered', self._on_raw_scan, qos_profile_sensor_data)
        if Contacts is not None:
            for topic in CONTACT_TOPICS:
                self.create_subscription(Contacts, topic, self._on_contacts, 10)
        self.create_timer(2.0, self._print)

    def _on_estimate(self, msg):
        p = msg.pose
        self.estimate = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _on_truth(self, msg):
        self.truth = (msg.position.x, msg.position.y, yaw_of(msg.orientation))

    def _on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)

    def _on_tilt(self, msg):
        self.tilt = (msg.vector.x, msg.vector.y, msg.vector.z)

    @staticmethod
    def _count(msg):
        return sum(1 for r in msg.ranges
                   if math.isfinite(r) and msg.range_min <= r <= msg.range_max)

    def _on_scan(self, name, msg):
        self.msgs[name] += 1
        self.obstacles[name] = self._count(msg)

    def _on_raw_scan(self, msg):
        self.raw_valid = self._count(msg)

    def _on_contacts(self, msg):
        for contact in msg.contacts:
            names = (contact.collision1.name, contact.collision2.name)
            # Les roues et la roulette touchent le sol en permanence.
            if any('floor' in n or 'ground' in n for n in names):
                continue
            self.contacts += 1
            self.contact_names.update(names)

    def _print(self):
        parts = []
        if self.estimate is None:
            parts.append('pose estimée: --')
        else:
            x, y, yaw = self.estimate
            text = f'pose ({x:+.2f}, {y:+.2f}, {math.degrees(yaw):+.0f}°)'
            if self.goal is not None:
                text += f' but à {math.hypot(self.goal[0] - x, self.goal[1] - y):.2f} m'
            parts.append(text)
            if self.truth is not None:
                gap = math.hypot(self.truth[0] - x, self.truth[1] - y)
                dyaw = math.degrees(math.atan2(
                    math.sin(self.truth[2] - yaw), math.cos(self.truth[2] - yaw)))
                self.max_gap = max(self.max_gap, gap)
                parts.append(f'ecart {gap:.2f} m / {dyaw:+.1f}° (max {self.max_gap:.2f})')
        v, w = self.cmd
        parts.append(f'v={v:+.2f} w={w:+.2f}')
        scans = ' '.join(
            f'{k}:{"--" if self.obstacles[k] is None else self.obstacles[k]}({self.msgs[k]})'
            for k in SCANS)
        parts.append(scans)
        if self.raw_valid is not None and self.obstacles['lidar'] is not None:
            parts.append(f'sol_lidar:{self.raw_valid - self.obstacles["lidar"]}')
        if self.tilt is not None:
            parts.append(f'tangage {math.degrees(self.tilt[1]):.2f}°')
        if Contacts is None:
            parts.append('contacts: n/d')
        else:
            parts.append(f'contacts:{self.contacts}'
                         + (f' {sorted(self.contact_names)}' if self.contact_names else ''))
        self.get_logger().info(' | '.join(parts))
        self.msgs = {k: 0 for k in SCANS}
        self.contacts = 0
        self.contact_names = set()


def main():
    rclpy.init()
    node = Diag()
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
