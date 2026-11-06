"""Signalement de la course à l'arbitre officiel (départ, visites, fin).

`parc_msgs` est installé dans l'espace de travail de base du robot : les
types sont chargés à l'exécution. Leur absence empêche le départ du match.
"""

from copy import deepcopy

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from caytu_course.arbitre import remplir_depart, remplir_fin, remplir_visite


def qos_statut(profondeur=10):
    return QoSProfile(
        depth=profondeur, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)


def qos_evenements():
    """Historique gardé : un afficheur lancé en retard reçoit tout le récit."""
    return qos_statut(200)


def charger_type(nom):
    """Classe du message `paquet/msg/Type`, ou None s'il est introuvable."""
    try:
        from rosidl_runtime_py.utilities import get_message
        return get_message(nom)
    except (AttributeError, ModuleNotFoundError, ValueError):
        return None


def encoder_jpeg(image, qualite):
    """Octets JPEG d'un sensor_msgs/Image, ou None si l'encodage échoue."""
    try:
        import cv2
        from cv_bridge import CvBridge
        matrice = CvBridge().imgmsg_to_cv2(image, desired_encoding='bgr8')
        reussi, tampon = cv2.imencode(
            '.jpg', matrice, [int(cv2.IMWRITE_JPEG_QUALITY), int(qualite)])
    except Exception:  # noqa: BLE001 - une image ratée ne doit pas arrêter la course
        return None
    return tampon.tobytes() if reussi else None


class PublicateurArbitre:

    def __init__(self, node, arbitre):
        self._logger = node.get_logger()
        self._horloge = node.get_clock()
        self._arbitre = arbitre
        self._types = {}
        self._publicateurs = {}
        for role, (topic, nom_type) in {
                'depart': (arbitre.start_topic, arbitre.start_type),
                'visite': (arbitre.visit_topic, arbitre.visit_type),
                'fin': (arbitre.finish_topic, arbitre.finish_type)}.items():
            classe = charger_type(nom_type)
            if classe is None:
                self._logger.error(
                    f'{nom_type} introuvable : {topic} ne sera pas publié '
                    '(sourcer ~/parc_ws/install/setup.bash du robot).')
                continue
            self._types[role] = classe
            self._publicateurs[role] = node.create_publisher(classe, topic, qos_statut())

    def pret(self):
        return len(self._publicateurs) == 3

    def _nouveau(self, role, description):
        """Message vide du rôle, ou None (journalisé) si parc_msgs manque."""
        classe = self._types.get(role)
        if classe is None:
            self._logger.error(f'ARBITRE indisponible : {description} non signalé.')
            return None
        return classe()

    def _publier(self, role, message, description):
        self._publicateurs[role].publish(message)
        self._logger.info(f'ARBITRE {description}.')

    def signaler_depart(self, vocal, entendu):
        message = self._nouveau('depart', 'départ')
        if message is None:
            return
        methode = (self._arbitre.start_method_voice if vocal
                   else self._arbitre.start_method_command)
        remplir_depart(message, methode, entendu)
        if hasattr(message, 'stamp'):
            message.stamp = self._horloge.now().to_msg()
        self._publier('depart', message, f'départ (méthode {methode}, « {entendu or ""} »)')

    def signaler_visite(self, tag_id, image):
        message = self._nouveau('visite', f'visite du tag {tag_id}')
        if message is None:
            return False
        jpeg = None
        if image is not None:
            jpeg = encoder_jpeg(image, self._arbitre.jpeg_quality)
        if not jpeg:
            self._logger.error(f'Visite du tag {tag_id} différée : JPEG indisponible.')
            return False
        remplis = remplir_visite(
            message, tag_id, jpeg, image, image.header if image is not None else None)
        if not remplis:
            self._logger.error(f'Visite du tag {tag_id} différée : champ image incompatible.')
            return False
        if hasattr(message, 'header'):
            message.header = deepcopy(image.header)
            message.header.stamp = self._horloge.now().to_msg()
        self._publier('visite', message,
                       f'visite du tag {tag_id} (image : {", ".join(remplis) or "aucune"})')
        return True

    def signaler_fin(self, motif, resume):
        message = self._nouveau('fin', f'fin ({motif})')
        if message is None:
            return
        remplir_fin(message, self._arbitre.finish_reason)
        if hasattr(message, 'stamp'):
            message.stamp = self._horloge.now().to_msg()
        self._publier('fin', message, f'fin de course ({motif}) : {resume}')
