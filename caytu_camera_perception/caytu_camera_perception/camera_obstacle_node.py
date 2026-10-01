#!/usr/bin/env python3
"""top_camera (nuage de points) -> LaserScan d'obstacles en hauteur pour Nav2.

Un seul node, pas de PointCloud2 intermédiaire : décimation, filtrage par bande
de hauteur et projection se font en numpy sur le buffer brut du message.
Les transformations caméra -> base_footprint / scan sont statiques (liens fixes
dans le URDF) : elles sont lues une fois puis mises en cache.
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener

from caytu_camera_perception.scan_projection import ProjectionParams, project_to_scan


def _quat_to_rot(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _cloud_xyz(msg: PointCloud2, row_stride: int, col_stride: int):
    """Extrait (N,3) float32 du buffer brut, décimé, sans passer par read_points()."""
    offs = {f.name: f.offset for f in msg.fields}
    dt = np.dtype({
        'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
        'offsets': [offs['x'], offs['y'], offs['z']], 'itemsize': msg.point_step,
    })
    n = msg.width * msg.height
    if len(msg.data) != n * msg.point_step:
        return None
    arr = np.frombuffer(msg.data, dtype=dt, count=n)
    if msg.height > 1:      # nuage organisé (cas de Gazebo : 640x480)
        arr = arr.reshape(msg.height, msg.width)[::row_stride, ::col_stride].reshape(-1)
    else:
        arr = arr[::row_stride * col_stride]
    return np.column_stack((arr['x'], arr['y'], arr['z'])).astype(np.float32)


class CameraObstacleNode(Node):

    def __init__(self):
        super().__init__('camera_obstacle_node')
        d = ProjectionParams()
        self.declare_parameter('cloud_topic', '/top_camera_depth/points')
        self.declare_parameter('scan_topic', '/top_camera/obstacles_scan')
        self.declare_parameter('scan_frame', 'top_camera_link')   # horizontal, origine = caméra
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('row_stride', 2)
        self.declare_parameter('col_stride', 4)
        for name in ('z_min', 'z_max', 'r_min', 'r_max', 'angle_min', 'angle_max',
                     'angle_inc', 'quantile'):
            self.declare_parameter(name, getattr(d, name))
        self.declare_parameter('min_points', d.min_points)
        self.declare_parameter('min_valid', d.min_valid)

        self.p = ProjectionParams(**{k: self.get_parameter(k).value for k in (
            'z_min', 'z_max', 'r_min', 'r_max', 'angle_min', 'angle_max', 'angle_inc',
            'quantile', 'min_points', 'min_valid')})
        self.row_stride = int(self.get_parameter('row_stride').value)
        self.col_stride = int(self.get_parameter('col_stride').value)
        self.scan_frame = self.get_parameter('scan_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # spin_thread=True : sans lui, lookup_transform() bloquerait le callback du nuage
        # alors que le même executor doit recevoir /tf_static.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self._cache = None      # (R_scan, t_scan, z_row_base, z_off_base) une fois résolu

        self.pub = self.create_publisher(LaserScan, self.get_parameter('scan_topic').value,
                                         qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.get_parameter('cloud_topic').value,
                                 self._on_cloud, qos_profile_sensor_data)

        self._frames = 0
        self._ms_total = 0.0
        self._last_stats = time.monotonic()
        self.get_logger().info(
            f'Bande de hauteur [{self.p.z_min:.2f}, {self.p.z_max:.2f}] m, portée '
            f'[{self.p.r_min:.2f}, {self.p.r_max:.2f}] m, {self.p.n_bins} secteurs -> '
            f'{self.get_parameter("scan_topic").value} (frame {self.scan_frame})')

    def _resolve_tf(self, cloud_frame: str) -> bool:
        try:
            t_scan = self.tf_buffer.lookup_transform(self.scan_frame, cloud_frame, Time())
            t_base = self.tf_buffer.lookup_transform(self.base_frame, cloud_frame, Time())
        except TransformException as ex:
            self.get_logger().warn(f'TF {cloud_frame} -> {self.scan_frame}/{self.base_frame} '
                                   f'indisponible : {ex}', throttle_duration_sec=2.0)
            return False
        ts, tb = t_scan.transform, t_base.transform
        r_scan = _quat_to_rot(ts.rotation)
        r_base = _quat_to_rot(tb.rotation)
        self._cache = (
            cloud_frame, r_scan,
            np.array([ts.translation.x, ts.translation.y, ts.translation.z]),
            r_base[2, :], tb.translation.z,     # seule la hauteur dans base_footprint compte
        )
        self.get_logger().info(f'TF caméra résolue (cloud frame = "{cloud_frame}")')
        return True

    def _on_cloud(self, msg: PointCloud2):
        t0 = time.perf_counter()
        frame = msg.header.frame_id
        if self._cache is None or self._cache[0] != frame:
            if not self._resolve_tf(frame):
                return
        _, r_scan, t_scan, z_row, z_off = self._cache

        pts = _cloud_xyz(msg, self.row_stride, self.col_stride)
        if pts is None:
            self.get_logger().error('PointCloud2 malformé (taille buffer)', throttle_duration_sec=5.0)
            return
        pts_scan = pts @ r_scan.T + t_scan
        z_base = pts @ z_row + z_off

        ranges = project_to_scan(pts_scan, z_base, self.p)

        out = LaserScan()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.scan_frame
        # LaserScan : le rayon i est à angle_min + i*inc ; nos secteurs sont centrés à +0.5*inc
        out.angle_min = float(self.p.angle_min + 0.5 * self.p.angle_inc)
        out.angle_increment = float(self.p.angle_inc)
        out.angle_max = float(out.angle_min + (self.p.n_bins - 1) * self.p.angle_inc)
        out.range_min = float(self.p.r_min)
        out.range_max = float(self.p.r_max)
        out.scan_time = 1.0 / 15.0
        out.ranges = ranges.tolist()
        self.pub.publish(out)

        self._frames += 1
        self._ms_total += (time.perf_counter() - t0) * 1e3
        now = time.monotonic()
        if now - self._last_stats > 5.0:
            n_obs = int(np.isfinite(ranges).sum())
            n_free = int(np.isposinf(ranges).sum())
            n_unk = int(np.isnan(ranges).sum())
            zp = np.nanpercentile(z_base, [5, 50, 95]) if np.isfinite(z_base).any() else [np.nan] * 3
            self.get_logger().info(
                f'{self._frames / (now - self._last_stats):.1f} Hz, '
                f'{self._ms_total / max(self._frames, 1):.2f} ms/img | secteurs : '
                f'{n_obs} obstacle, {n_free} libres, {n_unk} inconnus | '
                f'z(base) p5/p50/p95 = {zp[0]:.2f}/{zp[1]:.2f}/{zp[2]:.2f} m '
                f'(p5 doit être ~0 : le sol ; sinon repère du nuage mal interprété)')
            self._frames, self._ms_total, self._last_stats = 0, 0.0, now


def main(args=None):
    rclpy.init(args=args)
    node = CameraObstacleNode()
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
