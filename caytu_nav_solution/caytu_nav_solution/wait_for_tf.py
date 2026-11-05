#!/usr/bin/env python3
"""Attend qu'une transformation TF existe, puis s'arrête (code 0, sinon 1).

solution_bringup.launch.py s'en sert avec localization:=slam : Nav2 ne démarre
qu'une fois slam_toolbox localisé sur la carte (map -> base_footprint). Sinon
la costmap globale peut abandonner son activation avant la première position,
surtout en temps simulé, et tout Nav2 reste arrêté.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class WaitForTf(Node):

    def __init__(self):
        super().__init__('wait_for_tf')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('source_frame', 'base_footprint')
        self.declare_parameter('timeout_sec', 180.0)            # temps réel
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self, spin_thread=False)


def main(args=None):
    rclpy.init(args=args)
    node = WaitForTf()
    target = node.get_parameter('target_frame').value
    source = node.get_parameter('source_frame').value
    timeout = float(node.get_parameter('timeout_sec').value)
    start = last_log = time.monotonic()
    code = 1
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.buffer.can_transform(target, source, Time()):
                node.get_logger().info(
                    f'TF {target} -> {source} disponible après '
                    f'{time.monotonic() - start:.1f} s : démarrage de Nav2.')
                code = 0
                break
            if time.monotonic() - start > timeout:
                node.get_logger().error(
                    f'TF {target} -> {source} absente après {timeout:.0f} s : Nav2 '
                    'ne sera pas démarré (slam_toolbox s\'est-il localisé ?).')
                break
            if time.monotonic() - last_log > 5.0:
                last_log = time.monotonic()
                node.get_logger().info(f'Attente de la TF {target} -> {source}...')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
