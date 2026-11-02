"""Détection visuelle prototype des mouvements dans le couloir du robot.

Le nœud compense le mouvement global de l'image avec un affine RANSAC, puis
cherche les mouvements résiduels dans une région d'intérêt. Il publie un Bool
maintenu à True quelques images pour éviter les basculements image par image.

Cette première version ne classe pas les personnes et n'estime pas leur distance.
La région d'intérêt est donc une approximation en pixels du couloir de passage.
"""

from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class DynamicObstacleDetector(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_detector')
        self.declare_parameter('image_topic', '/top_camera_color/image_raw')
        self.declare_parameter('blocked_topic', '/dynamic_obstacle/blocked')
        self.declare_parameter('roi_x_min', 0.30)
        self.declare_parameter('roi_x_max', 0.70)
        self.declare_parameter('roi_y_min', 0.15)
        self.declare_parameter('roi_y_max', 0.95)
        self.declare_parameter('motion_threshold', 24)
        self.declare_parameter('min_motion_pixels', 450)
        self.declare_parameter('frames_to_block', 3)
        self.declare_parameter('frames_to_clear', 8)

        self._bridge = CvBridge()
        self._previous_gray: Optional[np.ndarray] = None
        self._blocked = False
        self._motion_count = 0
        self._clear_count = 0
        self._publisher = self.create_publisher(
            Bool, self.get_parameter('blocked_topic').value, 10)
        self._subscription = self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Écoute {self.get_parameter('image_topic').value}; publie "
            f"{self.get_parameter('blocked_topic').value}.")

    def _roi_mask(self, height: int, width: int) -> np.ndarray:
        mask = np.zeros((height, width), dtype=np.uint8)
        x0 = int(float(self.get_parameter('roi_x_min').value) * width)
        x1 = int(float(self.get_parameter('roi_x_max').value) * width)
        y0 = int(float(self.get_parameter('roi_y_min').value) * height)
        y1 = int(float(self.get_parameter('roi_y_max').value) * height)
        mask[max(0, y0):min(height, y1), max(0, x0):min(width, x1)] = 255
        return mask

    @staticmethod
    def _compensate_global_motion(previous: np.ndarray,
                                  current: np.ndarray) -> np.ndarray:
        """Aligne l'image précédente sur l'actuelle pour réduire le flux caméra."""
        points = cv2.goodFeaturesToTrack(
            previous, maxCorners=400, qualityLevel=0.01, minDistance=8,
            blockSize=7)
        if points is None or len(points) < 12:
            return previous

        tracked, status, _ = cv2.calcOpticalFlowPyrLK(
            previous, current, points, None,
            winSize=(21, 21), maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
        if tracked is None or status is None:
            return previous
        valid = status.reshape(-1) == 1
        if int(np.count_nonzero(valid)) < 12:
            return previous

        transform, inliers = cv2.estimateAffinePartial2D(
            points.reshape(-1, 2)[valid], tracked.reshape(-1, 2)[valid],
            method=cv2.RANSAC, ransacReprojThreshold=2.5,
            maxIters=1000, confidence=0.99)
        if transform is None or inliers is None or int(np.count_nonzero(inliers)) < 10:
            return previous

        height, width = current.shape
        return cv2.warpAffine(previous, transform, (width, height))

    def _on_image(self, message: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except CvBridgeError as error:
            self.get_logger().warning(f'Image caméra illisible : {error}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self._previous_gray is None or self._previous_gray.shape != gray.shape:
            self._previous_gray = gray
            self._publish_state(False)
            return

        aligned_previous = self._compensate_global_motion(self._previous_gray, gray)
        difference = cv2.absdiff(gray, aligned_previous)
        difference = cv2.GaussianBlur(difference, (5, 5), 0)
        threshold = int(self.get_parameter('motion_threshold').value)
        _, motion = cv2.threshold(difference, threshold, 255, cv2.THRESH_BINARY)
        motion = cv2.morphologyEx(
            motion, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8))
        motion = cv2.dilate(motion, np.ones((5, 5), dtype=np.uint8), iterations=2)
        motion = cv2.bitwise_and(motion, self._roi_mask(*gray.shape))

        motion_pixels = cv2.countNonZero(motion)
        detected = motion_pixels >= int(self.get_parameter('min_motion_pixels').value)
        if detected:
            self._motion_count += 1
            self._clear_count = 0
            if (not self._blocked
                    and self._motion_count >= int(self.get_parameter('frames_to_block').value)):
                self._blocked = True
                self.get_logger().info(
                    f'Mouvement dans le couloir caméra ({motion_pixels} pixels).')
        else:
            self._clear_count += 1
            self._motion_count = 0
            if (self._blocked
                    and self._clear_count >= int(self.get_parameter('frames_to_clear').value)):
                self._blocked = False
                self.get_logger().info('Le mouvement a quitté le couloir caméra.')

        self._publish_state(self._blocked)
        self._previous_gray = gray

    def _publish_state(self, blocked: bool):
        message = Bool()
        message.data = blocked
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleDetector()
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
