#!/usr/bin/env python3
"""Rejeu d'un enregistrement : republie /tf sans l'ancienne correction map -> odom.

Un enregistrement fait pendant une cartographie contient la TF map -> odom
publiée par slam_toolbox ce jour-là. Pour recartographier depuis cet
enregistrement avec d'autres réglages, cette ancienne TF doit disparaître,
sinon deux nœuds publient map -> odom.

    ros2 bag play <enregistrement> --clock --remap /tf:=/tf_bag
    (mapping.launch.py replay:=true lance ce relais)
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TfBagRelay(Node):

    def __init__(self):
        super().__init__('tf_bag_relay')
        self.declare_parameter('input_topic', '/tf_bag')
        self.declare_parameter('output_topic', '/tf')
        self.declare_parameter('drop_parent', 'map')
        self.declare_parameter('drop_child', 'odom')
        self._parent = self.get_parameter('drop_parent').value.lstrip('/')
        self._child = self.get_parameter('drop_child').value.lstrip('/')
        qos = QoSProfile(depth=100, reliability=QoSReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(TFMessage, self.get_parameter('output_topic').value, qos)
        self.create_subscription(
            TFMessage, self.get_parameter('input_topic').value, self._on_tf, qos)
        self._dropped = 0

    def _on_tf(self, msg: TFMessage):
        kept = [t for t in msg.transforms
                if not (t.header.frame_id.lstrip('/') == self._parent
                        and t.child_frame_id.lstrip('/') == self._child)]
        if len(kept) != len(msg.transforms) and self._dropped == 0:
            self.get_logger().info(
                f'TF {self._parent} -> {self._child} de l\'enregistrement ignorée '
                '(slam_toolbox la recalcule).')
        self._dropped += len(msg.transforms) - len(kept)
        if kept:
            self._pub.publish(TFMessage(transforms=kept))


def main(args=None):
    rclpy.init(args=args)
    node = TfBagRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
