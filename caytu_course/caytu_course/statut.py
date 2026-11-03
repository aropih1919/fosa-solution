"""Signalement de la course aux organisateurs (départ, visites, fin).

Le paquet de statut officiel n'est pas encore publié : ce module est le SEUL
endroit à adapter quand il le sera. En attendant, les événements partent sur
les topics de `course.status` (JSON lisible, identifiant du tag, image).
QoS fiable + transient local : un afficheur lancé en retard reçoit le dernier
état.
"""

import json
import time

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String


def qos_statut(profondeur=10):
    return QoSProfile(
        depth=profondeur, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)


class PublicateurStatut:

    def __init__(self, node, statut):
        self._logger = node.get_logger()
        self._depart = node.create_publisher(String, statut.start_topic, qos_statut())
        self._visite = node.create_publisher(Int32, statut.visit_topic, qos_statut())
        self._image = node.create_publisher(Image, statut.image_topic, qos_statut(1))
        self._fin = node.create_publisher(String, statut.finish_topic, qos_statut())

    @staticmethod
    def _json(**contenu):
        contenu['horodatage'] = time.time()
        return String(data=json.dumps(contenu, ensure_ascii=False))

    def signaler_depart(self, ordre, source):
        self._depart.publish(self._json(
            evenement='depart', ordre=list(ordre), source=source))
        self._logger.info(f'STATUT départ (ordre {list(ordre)}, {source}).')

    def signaler_visite(self, tag_id, image):
        """Image de preuve d'abord, puis l'identifiant qui déclenche l'affichage."""
        if image is not None:
            self._image.publish(image)
        else:
            self._logger.warning(f'Visite du tag {tag_id} sans image de preuve.')
        self._visite.publish(Int32(data=int(tag_id)))
        self._logger.info(f'STATUT visite du tag {tag_id}.')

    def signaler_fin(self, motif, resume):
        self._fin.publish(self._json(evenement='fin', motif=motif, **resume))
        self._logger.info(f'STATUT fin de course ({motif}) : {resume}.')
