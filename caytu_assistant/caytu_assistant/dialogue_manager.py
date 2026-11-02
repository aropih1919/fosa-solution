"""Nœud de dialogue texte/transcript et pont déterministe vers Nav2."""

import json
import threading
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from caytu_assistant.dispatcher import DispatcherAssistant
from caytu_assistant.llm_client import ClientLLM
from caytu_config import load_config
from caytu_interfaces.msg import (
    DialogueState,
    SemanticLocationArray,
)
from caytu_nav_solution.navigation_runtime import (
    BringupManager,
    Nav2Navigator,
    TaskTimer,
)
from caytu_nav_solution.nav_math import Pose2D, lacet_depuis_quaternion
from caytu_semantic_map.registry import RegistreLieux, normaliser


class GestionnaireDialogue(Node):
    def __init__(self, navigation_node, configuration=None):
        super().__init__('dialogue_manager')
        self._configuration = configuration or load_config()
        self.declare_parameter('input_mode', 'topic')
        self.declare_parameter(
            'transcript_topic', self._configuration.dialogue.transcript_topic)
        self.declare_parameter(
            'response_topic', self._configuration.dialogue.response_topic)
        self.declare_parameter(
            'state_topic', self._configuration.dialogue.state_topic)
        self._locations = {}
        self._dialogue_state = 'IDLE'
        self._registry = RegistreLieux(self._configuration.semantic_map)
        self._timer = TaskTimer(
            navigation_node.get_clock().now,
            self._configuration.navigation.time_limit_sec,
            self._configuration.navigation.stop_margin_sec)
        self._timer.start()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, navigation_node, spin_thread=False)
        self._amcl_pose = None
        self._amcl_subscription = navigation_node.create_subscription(
            PoseWithCovarianceStamped,
            self._configuration.navigation.amcl_pose_topic,
            self._receive_amcl_pose, 10)
        bringup = BringupManager(
            self._configuration.navigation, navigation_node.get_logger())
        self._navigator = Nav2Navigator(
            navigation_node, self._configuration.navigation, self._timer,
            bringup, self._robot_pose,
            spin_once=lambda duration: time.sleep(duration),
            is_ok=rclpy.ok)
        self._dispatcher = DispatcherAssistant(
            self._configuration,
            self._registry,
            ClientLLM(self._configuration),
            self._navigate,
            self._cancel,
            self._resolve_location_message,
        )
        self._response_publisher = self.create_publisher(
            String, self.get_parameter('response_topic').value, 10)
        self._state_publisher = self.create_publisher(
            DialogueState, self.get_parameter('state_topic').value, 10)
        self._location_subscription = self.create_subscription(
            SemanticLocationArray,
            self._configuration.semantic_map.locations_topic,
            self._receive_locations,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._transcript_subscription = self.create_subscription(
            String, self.get_parameter('transcript_topic').value,
            self._receive_transcript, 10)
        self._person_event_subscription = self.create_subscription(
            String, self._configuration.perception.person_events_topic,
            self._receive_person_event, 10)
        if self.get_parameter('input_mode').value == 'text':
            threading.Thread(
                target=self._read_stdin, daemon=True,
                name='assistant-text-input').start()
        self.get_logger().info(
            f"Dialogue prêt (mode {self.get_parameter('input_mode').value}).")

    def _receive_amcl_pose(self, message):
        self._amcl_pose = message

    def _receive_locations(self, message):
        for location in message.locations:
            self._locations[normaliser(location.name)] = location
            for alias in location.aliases:
                self._locations[normaliser(alias)] = location

    def _resolve_location_message(self, name):
        return self._locations.get(normaliser(name))

    def _robot_pose(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                self._configuration.system.global_frame,
                self._configuration.system.base_frame,
                Time())
        except TransformException:
            return None
        position = transform.transform.translation
        rotation = transform.transform.rotation
        return Pose2D(
            position.x, position.y,
            lacet_depuis_quaternion(
                rotation.x, rotation.y, rotation.z, rotation.w))

    def _navigate(self, target_pose):
        self._timer.start()
        ready = self._navigator.attendre_pret(
            lambda: self._amcl_pose is not None,
            self._configuration.navigation.localization,
            self._configuration.navigation.nav2_startup_timeout_sec)
        if not ready:
            return False, 'Nav2 indisponible ou robot non localisé.'
        spawn = self._robot_pose()
        if spawn is None:
            return False, 'Pose du robot indisponible.'
        while rclpy.ok() and not self._timer.expired():
            result = self._navigator.naviguer_une_fois(
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.header.frame_id
                or self._configuration.semantic_map.frame,
                spawn)
            if result == 'succes':
                return True, ''
            if result in {'temps_ecoule', 'arret'}:
                return False, result
            if result == 'refuse':
                time.sleep(self._configuration.navigation.retry_pause_sec)
            else:
                self._navigator.vider_costmaps()
                time.sleep(self._configuration.navigation.retry_pause_sec)
        return False, 'limite de temps atteinte'

    def _cancel(self):
        self._navigator.annuler_but()

    def _publish_state(self, state, detail=''):
        message = DialogueState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.state = state
        message.detail = detail
        self._state_publisher.publish(message)

    def _receive_transcript(self, message):
        text = message.data.strip()
        if not text:
            return
        self._publish_state('THINKING', text)
        self._dialogue_state = 'THINKING'
        result = self._dispatcher.traiter(text)
        self._response_publisher.publish(String(data=result.response))
        self._dialogue_state = result.state
        self._publish_state(result.state, result.detail)
        self.get_logger().info(f'Visiteur : {text}')
        self.get_logger().info(f'Assistant : {result.response}')

    def _receive_person_event(self, message):
        if (not self._configuration.mission.greeting_enabled
                or self._dialogue_state not in {'IDLE', 'LISTENING'}):
            return
        try:
            event = json.loads(message.data)
        except json.JSONDecodeError:
            self.get_logger().warning('Événement personne invalide, ignoré.')
            return
        response = self._configuration.dialogue.greeting_reply
        self._response_publisher.publish(String(data=response))
        self._dialogue_state = 'LISTENING'
        self._publish_state('LISTENING', str(event.get('track_id', '')))

    def _read_stdin(self):
        self.get_logger().info('Saisissez un message, ou Ctrl-D pour terminer.')
        while rclpy.ok():
            try:
                text = input('Vous> ')
            except EOFError:
                return
            self._receive_transcript(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    configuration = load_config()
    navigation_node = rclpy.create_node(
        'assistant_navigation',
        parameter_overrides=[Parameter(
            'use_sim_time', Parameter.Type.BOOL,
            configuration.system.use_sim_time)],
    )
    navigation_executor = MultiThreadedExecutor(num_threads=2)
    navigation_executor.add_node(navigation_node)
    thread = threading.Thread(
        target=navigation_executor.spin, daemon=True,
        name='assistant-navigation-executor')
    thread.start()

    dialogue_node = GestionnaireDialogue(navigation_node, configuration)
    try:
        rclpy.spin(dialogue_node)
    except KeyboardInterrupt:
        pass
    finally:
        dialogue_node._cancel()
        dialogue_node.destroy_node()
        navigation_executor.shutdown()
        navigation_node.destroy_node()
        rclpy.shutdown()
