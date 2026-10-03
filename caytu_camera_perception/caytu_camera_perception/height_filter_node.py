"""Isole les obstacles invisibles au LiDAR : nuage top_camera -> base_footprint -> bande de hauteur."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener

from caytu_camera_perception.point_filter import filter_obstacle_points


def transform_to_matrix(t):
    """Matrice 4x4 numpy depuis geometry_msgs/Transform (pas de do_transform_cloud :
    casse sur les champs rgb des nuages D435)."""
    q, p = t.transform.rotation, t.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    m = np.eye(4)
    m[:3, :3] = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    m[:3, 3] = [p.x, p.y, p.z]
    return m


class HeightFilterNode(Node):

    def __init__(self):
        super().__init__('height_filter_node')
        self.declare_parameter('input_topic', '/top_camera_depth/points')
        self.declare_parameter('output_topic', '/top_camera/obstacles_points')
        self.declare_parameter('target_frame', 'base_footprint')
        self.declare_parameter('z_min', 0.15)
        self.declare_parameter('z_max', 1.2)
        self.declare_parameter('max_range', 3.5)
        self.declare_parameter('tf_timeout_sec', 0.1)

        input_topic = self.get_parameter('input_topic').value
        self._out = self.get_parameter('output_topic').value
        self._frame = self.get_parameter('target_frame').value
        self._zmin = self.get_parameter('z_min').value
        self._zmax = self.get_parameter('z_max').value
        self._range = self.get_parameter('max_range').value
        self._tf_timeout = self.get_parameter('tf_timeout_sec').value

        self._tf = Buffer()
        TransformListener(self._tf, self)
        qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, input_topic, self._on_cloud, qos)
        self._pub = self.create_publisher(PointCloud2, self._out, 10)
        self.get_logger().info(f'height_filter: {input_topic} -> {self._out} '
                               f'(bande z [{self._zmin}, {self._zmax}] m en {self._frame})')

    def _xyz_from_msg(self, msg):
        offs = {f.name: f.offset // 4 for f in msg.fields if f.name in ('x', 'y', 'z')}
        if len(offs) != 3:
            return None
        w = msg.point_step // 4
        n = msg.width * msg.height
        flat = np.frombuffer(msg.data, dtype=np.float32).reshape(n, w)
        return np.stack([flat[:, offs['x']], flat[:, offs['y']], flat[:, offs['z']]], axis=1)

    def _on_cloud(self, msg):
        try:
            t = self._tf.lookup_transform(
                self._frame, msg.header.frame_id, msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=self._tf_timeout))
            pts = self._xyz_from_msg(msg)
            if pts is None:
                return
            m = transform_to_matrix(t)
            moved = (m[:3, :3] @ pts.T).T + m[:3, 3]
            kept, stats = filter_obstacle_points(moved, self._zmin, self._zmax, self._range)
            header = msg.header
            header.frame_id = self._frame
            self._pub.publish(point_cloud2.create_cloud_xyz32(header, kept.tolist()))
            self.get_logger().debug(f'obstacles: {stats["kept"]}/{stats["total"]}',
                                    throttle_duration_sec=5.0)
        except Exception as e:
            self.get_logger().warn(f'filtrage ignore (1 nuage) : {e}',
                                   throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = HeightFilterNode()
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
