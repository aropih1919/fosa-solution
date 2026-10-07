#!/usr/bin/env python3
"""Corrige les effets de l'inclinaison du robot sur les capteurs.

Le Sito-É repose sur deux roues et une roulette arrière qui descend 1 cm plus
bas que les roues : il penche donc d'environ 2,5° vers l'avant. La TF officielle
(odom -> base_footprint) est plane et l'ignore. Deux conséquences, mesurées dans
nos journaux de diagnostic :
  1. des rayons du lidar touchent le sol et créent de faux obstacles ;
  2. pour les caméras, le sol « monte » de 4 cm par mètre dans base_footprint.

Ce nœud mesure l'inclinaison réelle avec l'accéléromètre de l'IMU (robot à
l'arrêt, au démarrage), puis :
  - publie /scan_clean  : scan sans les retours du sol (pour MARQUER) ;
  - publie /scan_clear  : scan où le sol devient « libre jusque-là » (pour
    EFFACER), afin que la costmap continue à se nettoyer devant le robot ;
  - publie la TF statique base_footprint -> base_footprint_level, un repère
    réellement horizontal dans lequel les nuages des caméras sont filtrés.

Rien n'est modifié dans le URDF officiel.
"""

import math
import statistics

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped, Vector3Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import StaticTransformBroadcaster

from caytu_nav_solution.nav_math import (
    GRAVITY, floor_ranges, lidar_height, quaternion_from_rpy, split_floor_returns,
    tilt_from_accel)


class LidarFloorFilter(Node):

    def __init__(self):
        super().__init__('lidar_floor_filter')

        self.declare_parameter('scan_in', '/scan_filtered')
        self.declare_parameter('scan_mark', '/scan_clean')
        self.declare_parameter('scan_clear', '/scan_clear')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('level_frame', 'base_footprint_level')
        # Géométrie lue dans le URDF officiel (sitoe_robot.urdf) :
        #   capteur lidar à z = 0.0381 + 0.01 m, incliné de -2,5° (nez relevé) ;
        #   lidar 0.116 m derrière l'axe des roues ;
        #   roulette 9,8 mm plus basse que les roues, 0.228 m derrière l'axe.
        self.declare_parameter('lidar_mount_pitch', -0.04363323129985824)
        self.declare_parameter('lidar_mount_z', 0.0481)
        self.declare_parameter('lidar_lever_x', 0.116)
        self.declare_parameter('default_robot_pitch', 0.0429)
        # Nombre de mesures IMU immobiles avant de figer l'inclinaison.
        self.declare_parameter('tilt_samples', 150)
        # Un retour à plus de floor_ratio fois la distance du sol est du sol.
        self.declare_parameter('floor_ratio', 0.80)
        self.declare_parameter('clear_ratio', 0.70)
        # Forçage manuel : distance (m) de la ligne de sol droit devant le
        # lidar, mesurée dans /scan. 0.0 = calcul automatique par l'IMU.
        self.declare_parameter('floor_line_distance', 0.0)

        self._base_frame = self.get_parameter('base_frame').value
        self._level_frame = self.get_parameter('level_frame').value
        self._mount_pitch = float(self.get_parameter('lidar_mount_pitch').value)
        self._mount_z = float(self.get_parameter('lidar_mount_z').value)
        self._lever_x = float(self.get_parameter('lidar_lever_x').value)
        self._floor_ratio = float(self.get_parameter('floor_ratio').value)
        self._clear_ratio = float(self.get_parameter('clear_ratio').value)
        self._forced_line = float(self.get_parameter('floor_line_distance').value)
        self._needed = int(self.get_parameter('tilt_samples').value)

        self._roll = 0.0
        self._pitch = float(self.get_parameter('default_robot_pitch').value)
        self._tilt_frozen = False
        self._tilt_measured = False
        self._samples = []
        self._floor_cache = None        # (clé du scan, distances du sol)
        self._floor_count_logged = -1

        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._tilt_pub = self.create_publisher(Vector3Stamped, '/robot_tilt', 10)
        # Publication fiable : compatible avec les abonnés « best effort » de
        # Nav2 comme avec RViz.
        self._mark_pub = self.create_publisher(
            LaserScan, self.get_parameter('scan_mark').value, 10)
        self._clear_pub = self.create_publisher(
            LaserScan, self.get_parameter('scan_clear').value, 10)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_in').value, self._on_scan,
            qos_profile_sensor_data)
        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self._on_imu,
            qos_profile_sensor_data)

        self._publish_level_frame()
        self.get_logger().info(
            f'Inclinaison par défaut {math.degrees(self._pitch):.2f}° '
            '(géométrie du URDF) en attendant la mesure IMU.')

    # ------------------------------------------------------------------ IMU
    def _on_imu(self, msg: Imu):
        if self._tilt_frozen:
            return
        a = msg.linear_acceleration
        w = msg.angular_velocity
        norm = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
        still = math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z) < 0.02
        if not still or abs(norm - GRAVITY) > 0.5:
            self._samples.clear()       # le robot bouge : on recommence
            return
        self._samples.append((a.x, a.y, a.z))
        if len(self._samples) < self._needed:
            return

        ax = statistics.median(s[0] for s in self._samples)
        ay = statistics.median(s[1] for s in self._samples)
        az = statistics.median(s[2] for s in self._samples)
        roll, pitch = tilt_from_accel(ax, ay, az)
        self._tilt_frozen = True
        self._samples.clear()
        if abs(pitch) > 0.15 or abs(roll) > 0.10:
            self.get_logger().warn(
                f'Inclinaison IMU invraisemblable (tangage {math.degrees(pitch):.1f}°, '
                f'roulis {math.degrees(roll):.1f}°) : valeur par défaut conservée.')
            return
        self._roll, self._pitch = roll, pitch
        self._tilt_measured = True
        self._floor_cache = None
        self._publish_level_frame()

        plane_pitch = self._plane_pitch()
        line = (self._lidar_height() / math.sin(plane_pitch)
                if plane_pitch > 1e-4 else math.inf)
        self.get_logger().info(
            f'Inclinaison mesurée : tangage {math.degrees(pitch):.2f}°, '
            f'roulis {math.degrees(roll):.2f}°. Plan du lidar : '
            f'{math.degrees(plane_pitch):+.2f}° -> '
            + (f'ligne de sol à {line:.2f} m devant le lidar.' if math.isfinite(line)
               else 'aucun rayon ne touche le sol.'))

    # ----------------------------------------------------------- géométrie
    def _lidar_height(self) -> float:
        return lidar_height(self._pitch, self._mount_z, self._lever_x)

    def _plane_pitch(self) -> float:
        if self._forced_line > 0.0:
            # Dans /scan, la distance lue droit devant vaut hauteur / sin(tangage).
            return math.asin(min(1.0, self._lidar_height() / self._forced_line))
        return self._pitch + self._mount_pitch

    def _publish_level_frame(self):
        """Publie base_footprint -> base_footprint_level (repère horizontal)."""
        # Inverse de l'inclinaison du robot : conjugué du quaternion (r, p, 0).
        qx, qy, qz, qw = quaternion_from_rpy(self._roll, self._pitch, 0.0)
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        msg.child_frame_id = self._level_frame
        msg.transform.rotation.x = -qx
        msg.transform.rotation.y = -qy
        msg.transform.rotation.z = -qz
        msg.transform.rotation.w = qw
        self._static_broadcaster.sendTransform(msg)

    def _publish_tilt(self, header):
        """Diagnostic : x = roulis, y = tangage du robot, z = tangage du plan lidar.

        frame_id indique l'origine de la valeur : « imu » une fois l'inclinaison
        mesurée, « default » tant que c'est la valeur géométrique du URDF.
        """
        tilt = Vector3Stamped()
        tilt.header.stamp = header.stamp
        tilt.header.frame_id = 'imu' if self._tilt_measured else 'default'
        tilt.vector.x, tilt.vector.y = self._roll, self._pitch
        tilt.vector.z = self._plane_pitch()
        self._tilt_pub.publish(tilt)

    # ----------------------------------------------------------------- scan
    def _on_scan(self, msg: LaserScan):
        key = (len(msg.ranges), msg.angle_min, msg.angle_increment)
        if self._floor_cache is None or self._floor_cache[0] != key:
            angles = [msg.angle_min + i * msg.angle_increment for i in range(len(msg.ranges))]
            self._floor_cache = (
                key, floor_ranges(angles, self._plane_pitch(), self._roll,
                                  self._lidar_height()))
        mark, clear, n_floor = split_floor_returns(
            msg.ranges, self._floor_cache[1], msg.range_max,
            self._floor_ratio, self._clear_ratio)

        out_mark = LaserScan()
        out_mark.header = msg.header
        out_mark.angle_min, out_mark.angle_max = msg.angle_min, msg.angle_max
        out_mark.angle_increment = msg.angle_increment
        out_mark.time_increment, out_mark.scan_time = msg.time_increment, msg.scan_time
        out_mark.range_min, out_mark.range_max = msg.range_min, msg.range_max
        out_mark.ranges = mark
        out_mark.intensities = msg.intensities
        self._mark_pub.publish(out_mark)

        out_clear = LaserScan()
        out_clear.header = msg.header
        out_clear.angle_min, out_clear.angle_max = msg.angle_min, msg.angle_max
        out_clear.angle_increment = msg.angle_increment
        out_clear.time_increment, out_clear.scan_time = msg.time_increment, msg.scan_time
        out_clear.range_min, out_clear.range_max = msg.range_min, msg.range_max
        out_clear.ranges = clear
        out_clear.intensities = msg.intensities
        self._clear_pub.publish(out_clear)
        self._publish_tilt(msg.header)

        # Un seul message quand le nombre de rayons au sol change nettement.
        if self._tilt_frozen and abs(n_floor - self._floor_count_logged) > 10:
            self._floor_count_logged = n_floor
            self.get_logger().info(f'{n_floor} rayons du lidar attribués au sol et retirés.')


def main(args=None):
    rclpy.init(args=args)
    node = LidarFloorFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Arrêt silencieux : le launch envoie SIGINT à la fermeture.
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
