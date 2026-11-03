"""Calibre les poses map des tags et écrit un YAML de référence séparé."""

import math
import os
from pathlib import Path
import statistics

import rclpy
from rclpy.node import Node
import yaml

from caytu_config import load_config
from caytu_interfaces.msg import TagDetectionArray


class CalibrateurTags(Node):
    def __init__(self, configuration=None):
        super().__init__('tag_calibrator')
        self._configuration = configuration or load_config()
        tag = self._configuration.perception.apriltag
        self._samples_required = tag.calibration_samples
        self._output_path = Path(tag.calibration_file).expanduser()
        self._samples = {}
        self._subscription = self.create_subscription(
            TagDetectionArray,
            self._configuration.perception.tags_topic,
            self._recevoir,
            10)
        self.get_logger().info(
            f'Calibration: {self._samples_required} observations/tag, '
            f'sortie {self._output_path}')

    def _recevoir(self, message):
        for detection in message.detections:
            if not detection.has_pose_map:
                continue
            samples = self._samples.setdefault(detection.tag_id, [])
            if len(samples) >= self._samples_required:
                continue
            pose = detection.pose_map.pose
            yaw = math.atan2(
                2.0 * (pose.orientation.w * pose.orientation.z
                       + pose.orientation.x * pose.orientation.y),
                1.0 - 2.0 * (pose.orientation.y ** 2
                             + pose.orientation.z ** 2))
            samples.append((pose.position.x, pose.position.y,
                            pose.position.z, yaw))
            if len(samples) == self._samples_required:
                self.get_logger().info(
                    f'Tag {detection.tag_id} calibré : '
                    f'{statistics.mean(item[0] for item in samples):.3f}, '
                    f'{statistics.mean(item[1] for item in samples):.3f}')
        if self._samples and all(
                len(values) >= self._samples_required
                for values in self._samples.values()):
            self._ecrire()

    def _ecrire(self):
        data = {
            'frame': self._configuration.semantic_map.frame,
            'tags': {
                str(tag_id): {
                    'x': statistics.mean(item[0] for item in samples),
                    'y': statistics.mean(item[1] for item in samples),
                    'z': statistics.mean(item[2] for item in samples),
                    'yaw': math.atan2(
                        statistics.mean(math.sin(item[3]) for item in samples),
                        statistics.mean(math.cos(item[3]) for item in samples)),
                }
                for tag_id, samples in self._samples.items()
                if len(samples) >= self._samples_required
            },
        }
        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self._output_path.with_suffix(self._output_path.suffix + '.tmp')
        temporary.write_text(
            yaml.safe_dump(data, allow_unicode=True, sort_keys=True),
            encoding='utf-8')
        os.replace(temporary, self._output_path)
        self.get_logger().info(f'Calibration écrite dans {self._output_path}')


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateurTags()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
