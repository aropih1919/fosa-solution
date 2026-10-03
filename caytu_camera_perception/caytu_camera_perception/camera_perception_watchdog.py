"""Signale a Nav2 que la perception camera est stable (meme contrat que localization_watchdog)."""

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


class CameraPerceptionWatchdog(Node):

    def __init__(self):
        super().__init__('camera_perception_watchdog')
        self.declare_parameter('obstacles_topic', '/top_camera/obstacles_points')
        self.declare_parameter('ready_topic', '/camera_perception_ready')
        self.declare_parameter('min_rate_hz', 2.0)
        self.declare_parameter('window_sec', 3.0)

        self._window = self.get_parameter('window_sec').value
        self._min_rate = self.get_parameter('min_rate_hz').value
        self._stamps = []
        self.create_subscription(
            PointCloud2, self.get_parameter('obstacles_topic').value,
            self._on_msg, 10)
        self._pub = self.create_publisher(
            Bool, self.get_parameter('ready_topic').value, 10)
        self.create_timer(1.0, self._on_tick)
        self.get_logger().info('camera_watchdog: surveillance du flux obstacles camera.')

    def _on_msg(self, _msg):
        now = time.monotonic()
        self._stamps.append(now)
        self._stamps = [t for t in self._stamps if now - t <= self._window]

    def _on_tick(self):
        ready = len(self._stamps) / self._window >= self._min_rate
        self._pub.publish(Bool(data=ready))


def main(args=None):
    rclpy.init(args=args)
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
