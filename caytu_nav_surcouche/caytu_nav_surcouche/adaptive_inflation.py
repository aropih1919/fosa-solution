"""Ajuste inflation_radius des costmaps en live selon vitesse + encombrement (params dynamiques Nav2)."""

import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rcl_interfaces.srv import SetParameters

from caytu_nav_surcouche.inflation_policy import target_inflation


class AdaptiveInflation(Node):

    def __init__(self):
        super().__init__('adaptive_inflation')
        self.declare_parameter('open_r', 0.20)
        self.declare_parameter('mid_r', 0.30)
        self.declare_parameter('closed_r', 0.35)
        self.declare_parameter('period_sec', 2.0)
        self.declare_parameter('apply_step', 0.05)

        self._speed = 0.0
        self._min_range = math.inf
        self._applied = None
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(LaserScan, '/scan_filtered', self._on_scan, 10)
        self._local = self.create_client(SetParameters, '/local_costmap/set_parameters')
        self._global = self.create_client(SetParameters, '/global_costmap/set_parameters')
        self.create_timer(self.get_parameter('period_sec').value, self._on_tick)
        self.get_logger().info('adaptive_inflation: ajuste inflation_radius en live.')

    def _on_odom(self, msg):
        self._speed = abs(msg.twist.twist.linear.x)

    def _on_scan(self, msg):
        valid = [r for r in msg.ranges if math.isfinite(r)]
        self._min_range = min(valid) if valid else math.inf

    def _on_tick(self):
        p = self.get_parameter
        target = target_inflation(self._speed, self._min_range,
                                  p('open_r').value, p('mid_r').value, p('closed_r').value)
        if self._applied is not None and abs(target - self._applied) < p('apply_step').value:
            return
        self._applied = target
        for client, node, name in (
                (self._local, 'local', 'local_costmap.inflation_layer.inflation_radius'),
                (self._global, 'global', 'global_costmap.inflation_layer.inflation_radius')):
            if client.wait_for_service(timeout_sec=0.5):
                req = SetParameters.Request()
                req.parameters = [Parameter(name, value=target).to_parameter_msg()]
                client.call_async(req)
                self.get_logger().info(f'inflation {node} -> {target:.2f} '
                                       f'(v={self._speed:.2f}, d={self._min_range:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveInflation()
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
