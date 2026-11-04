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

from caytu_assistant.dispatcher import RepartiteurAssistant
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
from caytu_semantic_map.registre import RegistreLieux, normaliser


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
        self._lieux = {}
        self._etat_dialogue = 'IDLE'
        self._registre = RegistreLieux(self._configuration.semantic_map)
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
            self._recevoir_pose_amcl, 10)
        bringup = BringupManager(
            self._configuration.navigation, navigation_node.get_logger())
        self._navigator = Nav2Navigator(
            navigation_node, self._configuration.navigation, self._timer,
            bringup, self._pose_robot,
            spin_once=lambda duration: time.sleep(duration),
            is_ok=rclpy.ok)
        self._dispatcher = RepartiteurAssistant(
            self._configuration,
            self._registre,
            ClientLLM(self._configuration),
            self._naviguer,
            self._annuler,
            self._resoudre_message_lieu,
        )
        self._response_publisher = self.create_publisher(
            String, self.get_parameter('response_topic').value, 10)
        self._state_publisher = self.create_publisher(
            DialogueState, self.get_parameter('state_topic').value, 10)
        self._location_subscription = self.create_subscription(
            SemanticLocationArray,
            self._configuration.semantic_map.locations_topic,
            self._recevoir_lieux,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._transcript_subscription = self.create_subscription(
            String, self.get_parameter('transcript_topic').value,
            self._recevoir_transcript, 10)
        self._person_event_subscription = self.create_subscription(
            String, self._configuration.perception.person_events_topic,
            self._recevoir_evenement_personne, 10)
        if self.get_parameter('input_mode').value == 'text':
            threading.Thread(
                target=self._lire_entree_standard, daemon=True,
                name='assistant-text-input').start()
        self.get_logger().info(
            f"Dialogue prêt (mode {self.get_parameter('input_mode').value}).")

    def _recevoir_pose_amcl(self, message):
        """Mémorise la dernière estimation de pose fournie par AMCL."""
        self._amcl_pose = message

    def _recevoir_lieux(self, message):
        """Met à jour le cache local des lieux sémantiques (noms et synonymes)."""
        for lieu in message.locations:
            self._lieux[normaliser(lieu.name)] = lieu
            for alias in lieu.aliases:
                self._lieux[normaliser(alias)] = lieu

    def _resoudre_message_lieu(self, nom):
        """Retrouve le message SemanticLocation correspondant à un nom ou alias."""
        return self._lieux.get(normaliser(nom))

    def _pose_robot(self):
        """Interroge TF2 pour obtenir la position (x, y) et l'orientation (lacet) actuelles du robot."""
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

    def _naviguer(self, target_pose):
        """Ordonne à Nav2 de déplacer le robot vers la pose cible avec gestion des échecs."""
        self._timer.start()
        pret = self._navigator.attendre_pret(
            lambda: self._amcl_pose is not None,
            self._configuration.navigation.localization,
            self._configuration.navigation.nav2_startup_timeout_sec)
        if not pret:
            return False, 'Nav2 indisponible ou robot non localisé.'
        point_depart = self._pose_robot()
        if point_depart is None:
            return False, 'Pose du robot indisponible.'
        while rclpy.ok() and not self._timer.expired():
            resultat = self._navigator.naviguer_une_fois(
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.header.frame_id
                or self._configuration.semantic_map.frame,
                point_depart)
            if resultat == 'succes':
                return True, ''
            if resultat in {'temps_ecoule', 'arret'}:
                return False, resultat
            if resultat == 'refuse':
                time.sleep(self._configuration.navigation.retry_pause_sec)
            else:
                # Ne pas vider les observations : une personne ou un obstacle
                # mobile doit rester marqué jusqu'à ce que les capteurs
                # confirment que le passage est libre.
                time.sleep(self._configuration.navigation.retry_pause_sec)
        return False, 'limite de temps atteinte'

    def _annuler(self):
        """Annule immédiatement tout déplacement en cours dans Nav2."""
        self._navigator.annuler_but()

    def _publier_etat(self, etat, detail=''):
        """Notifie l'état actuel de la machine de dialogue sur le topic `/assistant/state`."""
        message = DialogueState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.state = etat
        message.detail = detail
        self._state_publisher.publish(message)

    def _recevoir_transcript(self, message):
        """Point d'entrée du dialogue : traite une phrase reçue (parole ou texte)."""
        texte = message.data.strip()
        if not texte:
            return
        self._publier_etat('THINKING', texte)
        self._etat_dialogue = 'THINKING'
        resultat = self._dispatcher.traiter(texte)
        self._response_publisher.publish(String(data=resultat.reponse))
        self._etat_dialogue = resultat.etat
        self._publier_etat(resultat.etat, resultat.detail)
        self.get_logger().info(f'Visiteur : {texte}')
        self.get_logger().info(f'Assistant : {resultat.reponse}')

    def _recevoir_evenement_personne(self, message):
        """Salue automatiquement un visiteur qui s'arrête devant le robot."""
        if (not self._configuration.mission.greeting_enabled
                or self._etat_dialogue not in {'IDLE', 'LISTENING'}):
            return
        try:
            evenement = json.loads(message.data)
        except json.JSONDecodeError:
            self.get_logger().warning('Événement personne invalide, ignoré.')
            return
        reponse = self._configuration.dialogue.greeting_reply
        self._response_publisher.publish(String(data=reponse))
        self._etat_dialogue = 'LISTENING'
        self._publier_etat('LISTENING', str(evenement.get('track_id', '')))

    def _lire_entree_standard(self):
        """Boucle de saisie texte dans le terminal pour tester sans microphone."""
        self.get_logger().info('Saisissez un message, ou Ctrl-D pour terminer.')
        while rclpy.ok():
            try:
                texte = input('Vous> ')
            except EOFError:
                return
            self._recevoir_transcript(String(data=texte))

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
        dialogue_node._annuler()
        dialogue_node.destroy_node()
        navigation_executor.shutdown()
        navigation_node.destroy_node()
        rclpy.shutdown()
