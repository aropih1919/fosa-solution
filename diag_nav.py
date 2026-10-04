#!/usr/bin/env python3
"""Diagnostic de navigation : une ligne toutes les 2 s (temps simulé).

Affiche ensemble la covariance AMCL, l'état de /localization_ready, la commande
envoyée au robot et le nombre d'obstacles vus par chaque scan (lidar + 2 caméras).
Permet de voir CE QUE FAISAIT LE ROBOT quand la localisation a été perdue, et si
les caméras voient quelque chose.

Lancer (nouveau terminal, après `source install/setup.bash`) :
    python3 diag_nav.py --ros-args -p use_sim_time:=true | tee diag.log
"""
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

CMD_TOPIC = '/robot_base_controller/cmd_vel_unstamped'
SCANS = {
    'lidar': '/scan_filtered',
    'top': '/top_camera_scan',
    'bottom': '/bottom_camera_scan',
}
# Memes seuils que localization_watchdog.py
XY_LOSS, YAW_LOSS = 0.5, 0.25


class Diag(Node):
    def __init__(self):
        super().__init__('diag_nav')
        self.cov = (math.nan, math.nan, math.nan)
        self.ready = None
        self.cmd = (0.0, 0.0)
        self.obstacles = {k: None for k in SCANS}   # points finis du dernier scan
        self.msgs = {k: 0 for k in SCANS}            # messages depuis la derniere ligne

        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_pose, 10)
        self.create_subscription(Bool, '/localization_ready', self._on_ready, 10)
        self.create_subscription(Twist, CMD_TOPIC, self._on_cmd, 10)
        for name, topic in SCANS.items():
            self.create_subscription(
                LaserScan, topic,
                lambda msg, n=name: self._on_scan(n, msg), qos_profile_sensor_data)
        self.create_timer(2.0, self._print)

    def _on_pose(self, msg):
        c = msg.pose.covariance
        self.cov = (c[0], c[7], c[35])

    def _on_ready(self, msg):
        self.ready = msg.data

    def _on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)

    def _on_scan(self, name, msg):
        self.msgs[name] += 1
        self.obstacles[name] = sum(
            1 for r in msg.ranges
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max)

    def _print(self):
        xx, yy, yaw = self.cov
        lost = xx > XY_LOSS or yy > XY_LOSS or yaw > YAW_LOSS
        v, w = self.cmd
        action = ''
        if abs(w) > 0.05 and abs(v) < 0.02:
            action = ' ROTATION-SUR-PLACE'
        elif abs(v) < 0.02 and abs(w) <= 0.05:
            action = ' ARRET'
        scans = ' | '.join(
            f'{k}: {"--" if self.obstacles[k] is None else self.obstacles[k]} pts'
            f' ({self.msgs[k]} msg)' for k in SCANS)
        self.get_logger().info(
            f'cov xx={xx:.3f} yy={yy:.3f} yaw={yaw:.3f}'
            f'{" [SEUIL PERTE DEPASSE]" if lost else ""} '
            f'ready={self.ready} | v={v:+.2f} w={w:+.2f}{action} | {scans}')
        self.msgs = {k: 0 for k in SCANS}


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
