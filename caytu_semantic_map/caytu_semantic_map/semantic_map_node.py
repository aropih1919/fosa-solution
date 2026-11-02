"""Services ROS de résolution des lieux et publication latched du registre."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from caytu_config import load_config
from caytu_interfaces.msg import (
    SemanticLocation as SemanticLocationMessage,
    SemanticLocationArray,
    TagDetectionArray,
)
from caytu_interfaces.srv import GetLocationByTag, ResolveLocation
from caytu_semantic_map.registry import (
    RegistreLieux,
    pose_approche_depuis_tag,
)


class NoeudCarteSemantique(Node):
    """Nœud ROS 2 gérant la carte sémantique du site.

    Rôle pour les débutants/intermédiaires :
      - Charge les lieux connus depuis `locations.yaml` (accueil, toilettes, etc.) ;
      - Écoute les détections de balises AprilTag sur le topic configuré ;
      - Calcule la pose d'approche devant chaque stand/lieu ;
      - Publie la liste des lieux sur `/semantic_map/locations` ;
      - Fournit les services ROS pour résoudre un nom de lieu ou un identifiant de tag.
    """

    def __init__(self, configuration=None):
        super().__init__('semantic_map')
        self._configuration = configuration or load_config()
        self._registry = RegistreLieux(self._configuration.semantic_map)
        self._latest_tag_pose = {}
        self._publisher = self.create_publisher(
            SemanticLocationArray,
            self._configuration.semantic_map.locations_topic,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._resolve_service = self.create_service(
            ResolveLocation, self._configuration.semantic_map.resolve_service,
            self._resoudre_lieu)
        self._tag_service = self.create_service(
            GetLocationByTag, self._configuration.semantic_map.by_tag_service,
            self._obtenir_lieu_par_tag)
        self._tag_subscription = self.create_subscription(
            TagDetectionArray, self._configuration.perception.tags_topic,
            self._recevoir_tags, 10)
        self._publier_lieux()

    def _recevoir_tags(self, message):
        """Met à jour les dernières poses connues pour chaque balise AprilTag détectée."""
        for detection in message.detections:
            self._latest_tag_pose[detection.tag_id] = detection.pose_map
        self._publier_lieux()

    def _vers_message(self, record, tag_pose=None):
        """Convertit un enregistrement de lieu interne en message ROS 2 typé `SemanticLocation`."""
        message = SemanticLocationMessage()
        message.name = record.name
        message.aliases = list(record.aliases)
        message.tag_ids = list(record.tag_ids)
        message.description = record.description
        message.category = record.category
        pose = record.approach_pose
        if pose is not None:
            message.has_approach_pose = True
            message.approach_pose.header.frame_id = self._registry.frame
            message.approach_pose.pose.position.x = pose.x
            message.approach_pose.pose.position.y = pose.y
            message.approach_pose.pose.orientation.z = math.sin(pose.yaw / 2.0)
            message.approach_pose.pose.orientation.w = math.cos(pose.yaw / 2.0)
        elif tag_pose is not None:
            message.has_approach_pose = True
            message.approach_pose.header = tag_pose.header
            x, y, yaw = pose_approche_depuis_tag(
                tag_pose.pose, record.approach_offset_m)
            message.approach_pose.pose.position.x = x
            message.approach_pose.pose.position.y = y
            message.approach_pose.pose.position.z = tag_pose.pose.position.z
            message.approach_pose.pose.orientation.z = math.sin(yaw / 2.0)
            message.approach_pose.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            message.approach_pose.header.frame_id = self._registry.frame
        return message

    def _publier_lieux(self):
        """Publie l'ensemble des lieux sémantiques avec leur statut d'approche mis à jour."""
        message = SemanticLocationArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._registry.frame
        message.locations = [self._vers_message(record) for record in self._registry.tous()]
        self._publisher.publish(message)

    def _resoudre_lieu(self, request, response):
        """Service ROS : cherche un lieu par son nom ou l'un de ses synonymes."""
        record = self._registry.resoudre(request.query)
        if record is None:
            response.success = False
            response.error = f'Lieu inconnu : {request.query}'
            return response
        pose_tag = next(
            (self._latest_tag_pose[tag_id] for tag_id in record.tag_ids
             if tag_id in self._latest_tag_pose), None)
        response.success = True
        response.location = self._vers_message(record, pose_tag)
        response.error = ''
        return response

    def _obtenir_lieu_par_tag(self, request, response):
        """Service ROS : retrouve le lieu associé au numéro de balise AprilTag."""
        record = self._registry.par_tag(request.tag_id)
        if record is None:
            response.success = False
            response.error = f'Tag inconnu : {request.tag_id}'
            return response
        response.success = True
        response.location = self._vers_message(
            record, self._latest_tag_pose.get(request.tag_id))
        response.error = ''
        return response

    # Alias de compatibilité pour le code externe ou les tests
    _receive_tags = _recevoir_tags
    _to_message = _vers_message
    _publish_locations = _publier_lieux
    _resolve_location = _resoudre_lieu
    _get_location_by_tag = _obtenir_lieu_par_tag


# Alias de classe pour compatibilité
SemanticMapNode = NoeudCarteSemantique


def main(args=None):
    rclpy.init(args=args)
    node = NoeudCarteSemantique()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
