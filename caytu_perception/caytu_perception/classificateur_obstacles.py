"""Classification géométrique de nuages de profondeur, complétée par YOLO."""

import math
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener

from caytu_config import load_config
from caytu_config.loader import config_directory
from caytu_interfaces.msg import DetectedEntity, DetectedEntityArray


def _tourner_quaternion(quaternion, vecteur):
    x, y, z, w = quaternion
    vx, vy, vz = vecteur
    uv = (y * vz - z * vy, z * vx - x * vz, x * vy - y * vx)
    uuv = (y * uv[2] - z * uv[1], z * uv[0] - x * uv[2],
           x * uv[1] - y * uv[0])
    return tuple(vecteur[i] + 2.0 * (w * uv[i] + uuv[i]) for i in range(3))


def _composantes(points, tolerance):
    """Regroupe les points sur une grille XY sans dépendance à scikit-learn."""
    cells = {}
    for point in points:
        # skip_nans ne retire pas les infinis des caméras réelles : une fois
        # tournés, ils deviennent NaN (inf × 0) et math.floor plantait.
        if not (math.isfinite(point[0]) and math.isfinite(point[1])):
            continue
        key = (math.floor(point[0] / tolerance),
               math.floor(point[1] / tolerance))
        cells.setdefault(key, []).append(point)
    remaining = set(cells)
    while remaining:
        pending = [remaining.pop()]
        component = []
        while pending:
            cell = pending.pop()
            component.extend(cells[cell])
            cx, cy = cell
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    neighbour = (cx + dx, cy + dy)
                    if neighbour in remaining:
                        remaining.remove(neighbour)
                        pending.append(neighbour)
        yield component


class ClassificateurObstacles(Node):
    def __init__(self, configuration=None):
        super().__init__('obstacle_classifier')
        self._configuration = configuration or load_config()
        self._bridge = CvBridge()
        self._camera_matrix = None
        self._yolo = None
        self._yolo_bearings = []
        self._last_yolo_time = 0.0
        self._last_entities = []
        self._previous_tracks = []
        self._next_track_id = 1
        self._last_cloud_time = time.monotonic()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False)
        perception = self._configuration.perception
        self.declare_parameter('cloud_topic', perception.top_depth_topic)
        self.declare_parameter('image_topic', perception.top_color_topic)
        self.declare_parameter('camera_info_topic', perception.top_info_topic)
        self.declare_parameter('output_topic', perception.entities_topic)
        self._publisher = self.create_publisher(
            DetectedEntityArray, self.get_parameter('output_topic').value, 10)
        self._camera_info_sub = self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self._recevoir_infos_camera, 10)
        self._cloud_sub = self.create_subscription(
            PointCloud2, self.get_parameter('cloud_topic').value,
            self._recevoir_nuage, 5)
        self._image_sub = self.create_subscription(
            Image, self.get_parameter('image_topic').value,
            self._recevoir_image, 5)
        self._charger_yolo()

    def _charger_yolo(self):
        yolo = self._configuration.perception.yolo
        if not yolo.enabled:
            return
        path = os.path.expanduser(yolo.model_path)
        if not os.path.isabs(path):
            path = str(config_directory() / path)
        try:
            self._yolo = cv2.dnn.readNetFromONNX(path)
            if yolo.device.lower() == 'cuda':
                self._yolo.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
                self._yolo.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
            self.get_logger().info(f'Modèle YOLO chargé : {path}')
        except (cv2.error, OSError) as error:
            self.get_logger().warning(
                f'YOLO indisponible ({error}); la géométrie reste active.')

    def _recevoir_infos_camera(self, message):
        self._camera_matrix = np.asarray(message.k, dtype=np.float32).reshape(3, 3)

    def _recevoir_image(self, message):
        if self._yolo is None or self._camera_matrix is None:
            return
        config = self._configuration.perception.yolo
        maintenant = time.monotonic()
        if maintenant - self._last_yolo_time < 1.0 / config.max_fps:
            return
        self._last_yolo_time = maintenant
        try:
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            self._yolo_bearings = self._detecter_directions_personnes(image)
            self._fusionner_et_publier(message.header)
        except (cv2.error, ValueError) as error:
            self.get_logger().warning(f'Image YOLO ignorée : {error}')

    def _detecter_directions_personnes(self, image):
        height, width = image.shape[:2]
        blob = cv2.dnn.blobFromImage(
            image, scalefactor=1.0 / 255.0,
            size=(self._configuration.perception.yolo.input_size,
                  self._configuration.perception.yolo.input_size), swapRB=True)
        self._yolo.setInput(blob)
        output = np.asarray(self._yolo.forward())
        output = np.squeeze(output)
        if output.ndim != 2:
            return []
        if output.shape[0] < output.shape[1]:
            output = output.T
        fx, cx = self._camera_matrix[0, 0], self._camera_matrix[0, 2]
        yolo = self._configuration.perception.yolo
        results = []
        for row in output:
            class_index = 4 + yolo.person_class_index
            if len(row) <= class_index:
                continue
            confidence = float(row[class_index])
            if confidence < yolo.confidence:
                continue
            center_x = float(row[0]) * width / yolo.input_size
            bearing = math.atan2(cx - center_x, float(fx))
            results.append((bearing, confidence))
        return results

    def _recevoir_nuage(self, message):
        try:
            transform = self._tf_buffer.lookup_transform(
                self._configuration.system.base_frame,
                message.header.frame_id,
                Time.from_msg(message.header.stamp))
        except TransformException:
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)
        points = []
        try:
            for point in point_cloud2.read_points(
                    message, field_names=('x', 'y', 'z'), skip_nans=True):
                point_tourne = _tourner_quaternion(quaternion, (float(point[0]),
                                                               float(point[1]),
                                                               float(point[2])))
                points.append((point_tourne[0] + translation.x,
                               point_tourne[1] + translation.y,
                               point_tourne[2] + translation.z))
        except (ValueError, TypeError) as error:
            self.get_logger().warning(f'Nuage de points ignoré : {error}')
            return
        config = self._configuration.perception.person
        candidates = []
        for cluster in _composantes(points, config.cluster_tolerance_m):
            if len(cluster) < config.min_cluster_points:
                continue
            xs, ys, zs = zip(*cluster)
            width = max(max(xs) - min(xs), max(ys) - min(ys))
            height = max(zs) - min(zs)
            x = sum(xs) / len(xs)
            y = sum(ys) / len(ys)
            distance = math.hypot(x, y)
            if distance == 0.0:
                continue
            is_person = height >= config.min_height_m and width <= config.max_width_m
            entity = DetectedEntity()
            entity.class_name = 'person' if is_person else 'obstacle'
            entity.confidence = min(1.0, len(cluster) / config.min_cluster_points)
            entity.position = Point(x=x, y=y, z=sum(zs) / len(zs))
            entity.distance_m = distance
            entity.bearing_rad = math.atan2(y, x)
            entity.is_person = is_person
            entity.source = 'geometry'
            candidates.append(entity)
        self._last_entities = self._attribuer_suivis(candidates)
        self._fusionner_et_publier(message.header)

    def _attribuer_suivis(self, candidates):
        now = time.monotonic()
        elapsed = max(now - self._last_cloud_time, 1e-3)
        self._last_cloud_time = now
        available = set(range(len(self._previous_tracks)))
        tracks = []
        for entity in candidates:
            best = None
            best_distance = self._configuration.perception.person.tracker_match_m
            for index in available:
                previous = self._previous_tracks[index]
                distance = math.hypot(
                    entity.position.x - previous[1],
                    entity.position.y - previous[2])
                if distance < best_distance:
                    best, best_distance = index, distance
            if best is None:
                track_id = self._next_track_id
                self._next_track_id += 1
                speed = 0.0
                velocity_x = 0.0
                velocity_y = 0.0
            else:
                previous = self._previous_tracks[best]
                available.remove(best)
                track_id = previous[0]
                speed = best_distance / elapsed
                velocity_x = (entity.position.x - previous[1]) / elapsed
                velocity_y = (entity.position.y - previous[2]) / elapsed
            entity.track_id = track_id
            entity.velocity.x = velocity_x
            entity.velocity.y = velocity_y
            entity.is_dynamic = (
                speed >= self._configuration.perception.person.dynamic_speed_mps)
            tracks.append((track_id, entity.position.x, entity.position.y))
        self._previous_tracks = tracks
        return candidates

    def _fusionner_et_publier(self, header):
        config = self._configuration.perception.person
        entities = list(self._last_entities)
        for bearing, confidence in self._yolo_bearings:
            closest = None
            closest_angle = config.bearing_tolerance_rad
            for entity in entities:
                angle = abs(entity.bearing_rad - bearing)
                if angle < closest_angle:
                    closest, closest_angle = entity, angle
            if closest is not None:
                closest.is_person = True
                closest.class_name = 'person'
                closest.confidence = max(closest.confidence, confidence)
                closest.source = 'geometry+yolo'
        message = DetectedEntityArray()
        message.header = header
        message.header.frame_id = self._configuration.system.base_frame
        message.entities = entities
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ClassificateurObstacles()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                       # Ctrl-C ou arrêt du lancement : sortie propre
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
