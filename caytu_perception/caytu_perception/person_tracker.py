"""Déclenche un événement personne seulement quand les critères sont stables."""

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from caytu_config import load_config
from caytu_interfaces.msg import DetectedEntityArray, DialogueState


class SuiviPersonnes(Node):
    def __init__(self, configuration=None):
        super().__init__('person_tracker')
        self._configuration = configuration or load_config()
        self._person = self._configuration.perception.person
        self._counts = {}
        self._last_event = 0.0
        self._last_velocity = None
        self._dialogue_state = 'IDLE'
        self.declare_parameter(
            'entities_topic', self._configuration.perception.entities_topic)
        self.declare_parameter(
            'events_topic', self._configuration.perception.person_events_topic)
        self._publisher = self.create_publisher(
            String, self.get_parameter('events_topic').value, 10)
        self._entities_sub = self.create_subscription(
            DetectedEntityArray, self.get_parameter('entities_topic').value,
            self._receive_entities, 10)
        self._velocity_sub = self.create_subscription(
            Twist, self._configuration.navigation.cmd_vel_topic,
            self._receive_velocity, 10)
        self._state_sub = self.create_subscription(
            DialogueState, self._configuration.dialogue.state_topic,
            self._receive_state, 10)

    def _receive_velocity(self, message):
        self._last_velocity = (abs(message.linear.x), abs(message.angular.z))

    def _receive_state(self, message):
        self._dialogue_state = message.state

    def _receive_entities(self, message):
        velocity = self._last_velocity
        stopped = (velocity is not None
                   and velocity[0] <= self._person.robot_stop_threshold_mps
                   and velocity[1] <= self._person.robot_stop_threshold_mps)
        eligible = {}
        for entity in message.entities:
            if not entity.is_person or entity.confidence < self._person.min_confidence:
                continue
            if not (self._person.min_distance_m <= entity.distance_m
                    <= self._person.max_distance_m):
                continue
            if abs(entity.bearing_rad) > self._person.front_fov_rad / 2.0:
                continue
            eligible[entity.track_id] = entity
        self._counts = {
            track_id: self._counts.get(track_id, 0) + 1
            for track_id in eligible
        }
        if (not self._configuration.mission.greeting_enabled or not stopped
                or self._dialogue_state not in {'IDLE', 'LISTENING'}
                or time.monotonic() - self._last_event
                < self._person.cooldown_sec):
            return
        for track_id, count in self._counts.items():
            if count < self._person.consecutive_frames:
                continue
            entity = eligible[track_id]
            event = {
                'track_id': track_id,
                'distance_m': entity.distance_m,
                'bearing_rad': entity.bearing_rad,
                'confidence': entity.confidence,
            }
            self._publisher.publish(String(data=json.dumps(event)))
            self._last_event = time.monotonic()
            self._counts = {}
            break


def main(args=None):
    rclpy.init(args=args)
    node = SuiviPersonnes()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
