"""Détection AprilTag et publication de poses transformées dans le repère map."""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from caytu_config import load_config
from caytu_interfaces.msg import TagDetection, TagDetectionArray


def _quaternion_from_matrix(matrix):
    trace = matrix[0, 0] + matrix[1, 1] + matrix[2, 2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return (float((matrix[2, 1] - matrix[1, 2]) / scale),
                float((matrix[0, 2] - matrix[2, 0]) / scale),
                float((matrix[1, 0] - matrix[0, 1]) / scale),
                float(0.25 * scale))
    index = int(np.argmax(np.diag(matrix)))
    next_index = (index + 1) % 3
    last_index = (index + 2) % 3
    scale = math.sqrt(
        1.0 + matrix[index, index] - matrix[next_index, next_index]
        - matrix[last_index, last_index]) * 2.0
    result = [0.0, 0.0, 0.0, 0.0]
    result[index] = 0.25 * scale
    result[3] = (matrix[last_index, next_index]
                 - matrix[next_index, last_index]) / scale
    result[next_index] = (matrix[next_index, index]
                          + matrix[index, next_index]) / scale
    result[last_index] = (matrix[last_index, index]
                          + matrix[index, last_index]) / scale
    return tuple(float(value) for value in result)


def _multiply_quaternions(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _rotate_vector(quaternion, vector):
    x, y, z, w = quaternion
    vx, vy, vz = vector
    qv = (x, y, z)
    cross1 = (qv[1] * vz - qv[2] * vy,
              qv[2] * vx - qv[0] * vz,
              qv[0] * vy - qv[1] * vx)
    cross2 = (qv[1] * cross1[2] - qv[2] * cross1[1],
              qv[2] * cross1[0] - qv[0] * cross1[2],
              qv[0] * cross1[1] - qv[1] * cross1[0])
    return tuple(vector[i] + 2.0 * (w * cross1[i] + cross2[i])
                 for i in range(3))


class DetecteurTags(Node):
    def __init__(self, configuration=None):
        super().__init__('tag_detector')
        self._configuration = configuration or load_config()
        self._bridge = CvBridge()
        self._camera_matrix = None
        self._distortion = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False)
        self.declare_parameter(
            'image_topic', self._configuration.perception.top_color_topic)
        self.declare_parameter(
            'camera_info_topic', self._configuration.perception.top_info_topic)
        self.declare_parameter(
            'output_topic', self._configuration.perception.tags_topic)
        self._publisher = self.create_publisher(
            TagDetectionArray, self.get_parameter('output_topic').value, 10)
        self._camera_sub = self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self._receive_camera_info, 10)
        self._image_sub = self.create_subscription(
            Image, self.get_parameter('image_topic').value,
            self._receive_image, 10)
        try:
            family = self._configuration.perception.apriltag.family.upper()
            dictionary_name = f'DICT_APRILTAG_{family}'
            dictionary_id = getattr(cv2.aruco, dictionary_name)
            self._dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        except (AttributeError, cv2.error) as error:
            raise RuntimeError(
                'OpenCV contrib avec cv2.aruco est requis pour les AprilTags.') from error

    def _receive_camera_info(self, message):
        self._camera_matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        self._distortion = np.asarray(message.d, dtype=np.float64)

    def _detect(self, image):
        if hasattr(cv2.aruco, 'DetectorParameters'):
            parameters = cv2.aruco.DetectorParameters()
        else:
            parameters = cv2.aruco.DetectorParameters_create()
        if hasattr(cv2.aruco, 'ArucoDetector'):
            corners, ids, _ = cv2.aruco.ArucoDetector(
                self._dictionary, parameters).detectMarkers(image)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                image, self._dictionary, parameters=parameters)
        return corners, ids

    def _receive_image(self, message):
        if self._camera_matrix is None:
            return
        try:
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding='mono8')
            corners, ids = self._detect(image)
        except (cv2.error, ValueError) as error:
            self.get_logger().warning(f'Image AprilTag ignorée : {error}')
            return
        output = TagDetectionArray()
        output.header = message.header
        if ids is not None:
            marker_size = self._configuration.perception.apriltag.size_m
            half = marker_size / 2.0
            object_points = np.asarray([
                [-half, half, 0.0], [half, half, 0.0],
                [half, -half, 0.0], [-half, -half, 0.0],
            ], dtype=np.float64)
            for corner, tag_id in zip(corners, ids.flatten()):
                image_points = corner.reshape(4, 2).astype(np.float64)
                found, rotation_vector, translation = cv2.solvePnP(
                    object_points, image_points,
                    self._camera_matrix, self._distortion)
                if not found:
                    continue
                distance = float(np.linalg.norm(translation))
                if distance > self._configuration.perception.apriltag.max_distance_m:
                    continue
                try:
                    detection = self._to_map_detection(
                        message, int(tag_id), rotation_vector,
                        translation.reshape(3), distance)
                except TransformException:
                    continue
                output.detections.append(detection)
        self._publisher.publish(output)

    def _to_map_detection(self, image, tag_id, rotation_vector,
                          translation, distance):
        transform = self._tf_buffer.lookup_transform(
            self._configuration.semantic_map.frame,
            image.header.frame_id,
            Time.from_msg(image.header.stamp))
        rotation, _ = cv2.Rodrigues(rotation_vector)
        q_camera_tag = _quaternion_from_matrix(rotation)
        q_map_camera_msg = transform.transform.rotation
        q_map_camera = (q_map_camera_msg.x, q_map_camera_msg.y,
                        q_map_camera_msg.z, q_map_camera_msg.w)
        q_map_tag = _multiply_quaternions(q_map_camera, q_camera_tag)
        rotated = _rotate_vector(q_map_camera, tuple(translation))
        detection = TagDetection()
        detection.header = image.header
        detection.tag_id = tag_id
        detection.pose_map.header.stamp = image.header.stamp
        detection.pose_map.header.frame_id = self._configuration.semantic_map.frame
        detection.pose_map.pose.position.x = transform.transform.translation.x + rotated[0]
        detection.pose_map.pose.position.y = transform.transform.translation.y + rotated[1]
        detection.pose_map.pose.position.z = transform.transform.translation.z + rotated[2]
        (detection.pose_map.pose.orientation.x,
         detection.pose_map.pose.orientation.y,
         detection.pose_map.pose.orientation.z,
         detection.pose_map.pose.orientation.w) = q_map_tag
        detection.distance_m = distance
        detection.confidence = 1.0
        return detection


def main(args=None):
    rclpy.init(args=args)
    node = DetecteurTags()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
