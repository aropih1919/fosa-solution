#!/usr/bin/env python3
"""Signal « prêt » de la perception caméra (même contrat que localization_watchdog).

Publie std_msgs/Bool sur /camera_perception_ready à 2 Hz.
Prêt = le scan d'obstacles arrive à cadence suffisante, voit au moins la moitié de
ses secteurs (caméra non aveugle) et la TF vers base_footprint existe, le tout de
façon stable. Hystérésis : perdu seulement après plusieurs évaluations mauvaises.
"""
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


class CameraPerceptionWatchdog(Node):

    def __init__(self):
        super().__init__('camera_perception_watchdog')
        self.declare_parameter('scan_topic', '/top_camera/obstacles_scan')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('min_rate_hz', 8.0)          # le nuage arrive à 15 Hz
        self.declare_parameter('window_sec', 2.0)
        self.declare_parameter('min_known_ratio', 0.5)      # part de secteurs non NaN
        self.declare_parameter('ready_after', 4)            # évaluations saines (2 Hz)
        self.declare_parameter('lost_after', 3)             # évaluations mauvaises

        g = self.get_parameter
        self.base_frame = g('base_frame').value
        self.min_rate = float(g('min_rate_hz').value)
        self.window = float(g('window_sec').value)
        self.min_known = float(g('min_known_ratio').value)
        self.ready_after = int(g('ready_after').value)
        self.lost_after = int(g('lost_after').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self._stamps = deque()      # (heure du nœud, ratio de secteurs connus)
        self._scan_frame = None
        self.is_ready = False
        self._good = 0
        self._bad = 0

        self.create_subscription(LaserScan, g('scan_topic').value, self._on_scan,
                                 qos_profile_sensor_data)
        self.pub = self.create_publisher(Bool, '/camera_perception_ready', 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info('Camera perception watchdog démarré, en attente du scan caméra...')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_scan(self, msg: LaserScan):
        self._scan_frame = msg.header.frame_id
        r = np.asarray(msg.ranges, dtype=np.float32)
        self._stamps.append((self._now(), float(np.mean(~np.isnan(r))) if r.size else 0.0))

    def _healthy(self) -> bool:
        now = self._now()
        while self._stamps and now - self._stamps[0][0] > self.window:
            self._stamps.popleft()
        if not self._stamps or self._scan_frame is None:
            return False
        rate = len(self._stamps) / self.window
        known = float(np.mean([k for _, k in self._stamps]))
        tf_ok = self.tf_buffer.can_transform(self.base_frame, self._scan_frame,
                                             rclpy.time.Time())
        return rate >= self.min_rate and known >= self.min_known and tf_ok

    def _tick(self):
        if self._healthy():
            self._good, self._bad = self._good + 1, 0
            if not self.is_ready and self._good >= self.ready_after:
                self.is_ready = True
                self.get_logger().info('Perception caméra PRÊTE (cadence, champ de vision et TF stables).')
        else:
            self._bad, self._good = self._bad + 1, 0
            if self.is_ready and self._bad >= self.lost_after:
                self.is_ready = False
                self.get_logger().warn('Perception caméra PERDUE (scan absent, aveugle ou TF manquante).')
        self.pub.publish(Bool(data=self.is_ready))


def main():
    rclpy.init()
    node = CameraPerceptionWatchdog()
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
