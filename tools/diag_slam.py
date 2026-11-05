#!/usr/bin/env python3
"""Gazebo : suit en direct l'écart entre slam_toolbox et la vraie pose du robot.

    python3 src/fosa-solution/tools/diag_slam.py --ros-args -p use_sim_time:=true \\
        -p csv:=diag_slam.csv | tee diag_slam.log

À lancer pendant mapping.launch.py gazebo:=true, AVANT de bouger le robot : la
première mesure sert de référence. Chaque seconde :
  ecart     position et cap de slam_toolbox par rapport à Gazebo
            (/sitoe_robot/pose) ;
  odom      dérive du cap de l'odométrie des roues depuis le départ ;
  tour      rotation réelle cumulée depuis le départ (en degrés) ;
  v, w      commande en cours.
Une ligne « SAUT » signale une hausse brusque de l'écart. On voit ainsi si la
carte décroche dans un virage, au retour dans une zone déjà vue (fermeture de
boucle) ou dès le départ. Le fichier CSV est à nous envoyer avec la carte.
"""

import csv
import math

import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from caytu_nav_solution.nav_math import Pose2D, normaliser_angle, lacet_depuis_quaternion


class DiagSlam(Node):

    def __init__(self):
        super().__init__('diag_slam')
        self.declare_parameter('truth_topic', '/sitoe_robot/pose')
        self.declare_parameter('cmd_vel_topic', '/robot_base_controller/cmd_vel_unstamped')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('jump_m', 0.10)
        self.declare_parameter('jump_deg', 2.0)
        self.declare_parameter('csv', '')
        self.truth = None
        self.cmd = (0.0, 0.0)
        self.world_from_slam = None
        self.world_from_odom = None
        self.prev = None
        self.turned = 0.0
        self.last_truth_yaw = None
        self.max_err = (0.0, 0.0)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(Pose, self.get_parameter('truth_topic').value, self._on_truth, 10)
        self.create_subscription(Twist, self.get_parameter('cmd_vel_topic').value, self._on_cmd, 10)
        path = self.get_parameter('csv').value
        self.csv = None
        if path:
            self.csv_file = open(path, 'w', newline='', encoding='utf-8')
            self.csv = csv.writer(self.csv_file)
            self.csv.writerow(['t', 'vrai_x', 'vrai_y', 'vrai_cap_deg', 'slam_x', 'slam_y',
                               'slam_cap_deg', 'ecart_m', 'ecart_cap_deg', 'odom_derive_deg',
                               'tour_deg', 'v', 'w'])
        self.create_timer(1.0, self._tick)

    def _on_truth(self, msg: Pose):
        q = msg.orientation
        yaw = lacet_depuis_quaternion(q.x, q.y, q.z, q.w)
        if self.last_truth_yaw is not None:
            self.turned += abs(normaliser_angle(yaw - self.last_truth_yaw))
        self.last_truth_yaw = yaw
        self.truth = Pose2D(msg.position.x, msg.position.y, yaw)

    def _on_cmd(self, msg: Twist):
        self.cmd = (msg.linear.x, msg.angular.z)

    def _lookup(self, target):
        try:
            tf = self.buffer.lookup_transform(
                target, self.get_parameter('base_frame').value, Time())
        except TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return Pose2D(t.x, t.y, lacet_depuis_quaternion(q.x, q.y, q.z, q.w))

    def _tick(self):
        slam = self._lookup(self.get_parameter('map_frame').value)
        odom = self._lookup(self.get_parameter('odom_frame').value)
        if self.truth is None or slam is None or odom is None:
            self.get_logger().info('En attente de la pose Gazebo, de map et de odom...')
            return
        if self.world_from_slam is None:
            # Référence : au départ, slam_toolbox et l'odométrie sont justes.
            self.world_from_slam = self.truth.composer(slam.inverse())
            self.world_from_odom = self.truth.composer(odom.inverse())
            self.turned = 0.0
            self.get_logger().info('Référence prise : vous pouvez piloter.')
            return
        est = self.world_from_slam.composer(slam)
        odom_w = self.world_from_odom.composer(odom)
        err = est.distance_vers(self.truth.x, self.truth.y)
        err_deg = math.degrees(normaliser_angle(est.yaw - self.truth.yaw))
        odom_deg = math.degrees(normaliser_angle(odom_w.yaw - self.truth.yaw))
        turned_deg = math.degrees(self.turned)
        v, w = self.cmd
        line = (f'ecart {err:5.2f} m {err_deg:+6.1f}° | odom {odom_deg:+6.1f}° | '
                f'tour {turned_deg:6.0f}° | v={v:+.2f} w={w:+.2f}')
        jump = self.prev is not None and (
            err - self.prev[0] > self.get_parameter('jump_m').value
            or abs(err_deg - self.prev[1]) > self.get_parameter('jump_deg').value)
        if jump:
            self.get_logger().warn('SAUT  ' + line)
        else:
            self.get_logger().info(line)
        self.prev = (err, err_deg)
        self.max_err = (max(self.max_err[0], err), max(self.max_err[1], abs(err_deg)))
        if self.csv is not None:
            now = self.get_clock().now().nanoseconds * 1e-9
            self.csv.writerow([f'{now:.2f}', f'{self.truth.x:.3f}', f'{self.truth.y:.3f}',
                               f'{math.degrees(self.truth.yaw):.2f}', f'{est.x:.3f}',
                               f'{est.y:.3f}', f'{math.degrees(est.yaw):.2f}', f'{err:.3f}',
                               f'{err_deg:.2f}', f'{odom_deg:.2f}', f'{turned_deg:.0f}',
                               f'{v:.2f}', f'{w:.2f}'])
            self.csv_file.flush()


def main():
    rclpy.init()
    node = DiagSlam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Écart maximal : {node.max_err[0]:.2f} m, {node.max_err[1]:.1f}°')
        if node.csv is not None:
            node.csv_file.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
