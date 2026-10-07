#!/usr/bin/env python3
"""Localisation sans carte : publie map -> odom à partir des roues et de l'IMU.

Remplace AMCL. Le repère `map` est rendu identique au repère du monde Gazebo en
partant du spawn de task_params.yaml ; le but de task_params.yaml s'envoie donc
à Nav2 tel quel. Le cap vient de l'IMU, car l'odométrie de Gazebo surestime les
rotations (entraxe déclaré plus petit que l'entraxe réel).

Entrées : /odom (nav_msgs/Odometry), /imu (sensor_msgs/Imu)
Sorties : /tf (map -> odom), /localization_pose (geometry_msgs/PoseStamped)
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from caytu_nav_solution.nav_math import (
    OdomImuFusion, Pose2D, normalize_angle, quaternion_from_yaw, yaw_from_quaternion)
from caytu_nav_solution.task_params import load_task_params


class OdomImuLocalizer(Node):

    def __init__(self):
        super().__init__('odom_imu_localizer')

        self.declare_parameter('task_params_file', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/imu')
        # Mettre à False pour revenir au cap de l'odométrie (diagnostic).
        self.declare_parameter('use_imu_yaw', True)
        # Au-delà de ce délai sans IMU, on retombe sur le cap de l'odométrie.
        self.declare_parameter('imu_timeout', 0.5)
        # La TF est post-datée comme le fait AMCL, pour rester utilisable entre
        # deux messages d'odométrie.
        self.declare_parameter('transform_tolerance', 0.2)

        task = load_task_params(self.get_parameter('task_params_file').value or None)
        self._fusion = OdomImuFusion(task.spawn)
        self._map_frame = self.get_parameter('map_frame').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._use_imu = bool(self.get_parameter('use_imu_yaw').value)
        self._imu_timeout = Duration(seconds=float(self.get_parameter('imu_timeout').value))
        self._tolerance = Duration(
            seconds=float(self.get_parameter('transform_tolerance').value))

        self._imu_yaw = None            # lacet absolu courant donné par l'IMU
        self._imu_stamp = None
        self._gyro_yaw = 0.0            # secours : intégration du gyroscope
        self._last_gyro_stamp = None
        self._got_odom = False
        self._imu_mode_logged = None

        self._broadcaster = TransformBroadcaster(self)
        self._pose_pub = self.create_publisher(PoseStamped, '/localization_pose', 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._on_odom,
            qos_profile_sensor_data)
        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self._on_imu,
            qos_profile_sensor_data)
        # Tant que l'odométrie n'est pas arrivée, Nav2 a quand même besoin de
        # map -> odom pour démarrer ses costmaps.
        self._startup_timer = self.create_timer(0.1, self._publish_startup_transform)

        self.get_logger().info(
            f'Localisation roues + IMU : spawn lu dans {task.path} '
            f'(x={task.spawn.x:.3f}, y={task.spawn.y:.3f}, yaw={task.spawn.yaw:.3f})')

    # ------------------------------------------------------------------ IMU
    def _on_imu(self, msg: Imu):
        stamp = Time.from_msg(msg.header.stamp)
        q = msg.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        orientation_valid = abs(norm - 1.0) < 0.05 and msg.orientation_covariance[0] >= 0.0

        # Secours : intégration de la vitesse de lacet si l'orientation manque.
        if self._last_gyro_stamp is not None:
            dt = (stamp - self._last_gyro_stamp).nanoseconds * 1e-9
            if 0.0 < dt < 0.5:
                self._gyro_yaw = normalize_angle(self._gyro_yaw + msg.angular_velocity.z * dt)
        self._last_gyro_stamp = stamp

        if orientation_valid:
            self._imu_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
            mode = 'orientation'
        else:
            self._imu_yaw = self._gyro_yaw
            mode = 'gyroscope intégré'
        self._imu_stamp = stamp
        if mode != self._imu_mode_logged:
            self._imu_mode_logged = mode
            self.get_logger().info(f'Cap fourni par l\'IMU ({mode}).')

    # ----------------------------------------------------------------- odom
    def _on_odom(self, msg: Odometry):
        stamp = Time.from_msg(msg.header.stamp)
        p = msg.pose.pose
        odom = Pose2D(
            p.position.x, p.position.y,
            yaw_from_quaternion(p.orientation.x, p.orientation.y,
                                p.orientation.z, p.orientation.w))

        imu_yaw = None
        if self._use_imu and self._imu_yaw is not None and self._imu_stamp is not None:
            age = stamp - self._imu_stamp
            if abs(age.nanoseconds) <= self._imu_timeout.nanoseconds:
                imu_yaw = self._imu_yaw
        had_imu = self._fusion.uses_imu
        map_to_odom = self._fusion.update(odom, imu_yaw)
        if self._got_odom and had_imu and not self._fusion.uses_imu:
            self.get_logger().warn(
                'IMU absente : cap repris sur l\'odométrie (moins précis en virage).')

        if not self._got_odom:
            self._got_odom = True
            self._startup_timer.cancel()
            self.get_logger().info('Première odométrie reçue : map -> odom publié en continu.')

        self._publish_transform(map_to_odom, stamp + self._tolerance)

        pose = self._fusion.pose
        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._map_frame
        out.pose.position.x = pose.x
        out.pose.position.y = pose.y
        qx, qy, qz, qw = quaternion_from_yaw(pose.yaw)
        out.pose.orientation.x, out.pose.orientation.y = qx, qy
        out.pose.orientation.z, out.pose.orientation.w = qz, qw
        self._pose_pub.publish(out)

    # ------------------------------------------------------------------- TF
    def _publish_startup_transform(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return                      # /clock pas encore reçu (temps simulé)
        self._publish_transform(self._fusion.initial_transform(), now + self._tolerance)

    def _publish_transform(self, transform: Pose2D, stamp: Time):
        msg = TransformStamped()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self._map_frame
        msg.child_frame_id = self._odom_frame
        msg.transform.translation.x = transform.x
        msg.transform.translation.y = transform.y
        qx, qy, qz, qw = quaternion_from_yaw(transform.yaw)
        msg.transform.rotation.x, msg.transform.rotation.y = qx, qy
        msg.transform.rotation.z, msg.transform.rotation.w = qz, qw
        self._broadcaster.sendTransform(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomImuLocalizer()
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
