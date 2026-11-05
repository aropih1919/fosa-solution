"""Arbitre officiel PARC : ce que la course lui envoie, et ce qu'elle en écoute.

Messages du paquet parc_msgs (dépôt PARC2026-Engineers-League, déjà installé
dans l'espace de travail de base du robot) :

  /parc/tag_order   TagOrder     arbitre -> robot   lu par course_tags.recevoir_ordre
  /parc/start       StartSignal  robot -> arbitre   une fois, au départ (commande ou voix)
  /parc/tag_visit   TagVisit     robot -> arbitre   à chaque tag, robot arrêté, image JPEG
  /parc/finish      Finish       robot -> arbitre   seulement pour finir en avance
  /parc/run_status  RunStatus    arbitre -> robot   la course s'arrête à FINISHED,
                                                    TIMED_OUT ou TERMINATED

QoS imposée par les organisateurs : RELIABLE, profondeur 10 pour les envois ;
RELIABLE + TRANSIENT_LOCAL, profondeur 1 pour l'ordre et le statut.

Sans parc_msgs (simulateur 2D, PC sans le dépôt PARC), l'arbitre n'est pas
prévenu et un avertissement le dit : la course tourne quand même.
"""

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

VERROUILLE = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
EVENEMENTS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

# RunStatus.state (parc_msgs/msg/RunStatus.msg)
NOMS_ETATS = {0: 'IDLE', 1: 'ARMED', 2: 'RUNNING', 3: 'FINISHED', 4: 'TIMED_OUT',
              5: 'TERMINATED'}
EN_COURSE = 2
ETATS_FINAUX = (3, 4, 5)


def encoder_jpeg(encodage, hauteur, largeur, pas, donnees, qualite):
    """Image ROS brute (sensor_msgs/Image) -> octets JPEG ; None si impossible."""
    import cv2
    import numpy as np
    canaux = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1, '8UC1': 1,
              '8UC3': 3}.get(encodage)
    if canaux is None or hauteur <= 0 or largeur <= 0:
        return None
    brut = np.frombuffer(bytes(donnees), dtype=np.uint8)
    if pas < largeur * canaux or brut.size < pas * hauteur:
        return None
    lignes = brut[:pas * hauteur].reshape(hauteur, pas)[:, :largeur * canaux]
    image = lignes.reshape(hauteur, largeur, canaux) if canaux > 1 else lignes
    conversions = {'rgb8': cv2.COLOR_RGB2BGR, 'rgba8': cv2.COLOR_RGBA2BGR,
                   'bgra8': cv2.COLOR_BGRA2BGR}
    if encodage in conversions:
        image = cv2.cvtColor(image, conversions[encodage])
    succes, jpeg = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), int(qualite)])
    return jpeg.tobytes() if succes else None


class ArbitreParc:
    """Envois à l'arbitre et suivi de son statut, pour course_tags."""

    def __init__(self, noeud, configuration):
        self._noeud = noeud
        self._configuration = configuration
        self.disponible = False
        self.etat = None                 # dernier RunStatus.state reçu
        self._course_vue = False         # RUNNING vu après notre départ
        self._depart_envoye = False
        self._faux_depart_signale = False
        if not configuration.enabled:
            noeud.get_logger().warn('Arbitre PARC désactivé (course.referee.enabled).')
            return
        try:
            from parc_msgs.msg import Finish, RunStatus, StartSignal, TagVisit
        except ImportError:
            noeud.get_logger().warn(
                'parc_msgs introuvable : l\'arbitre PARC ne sera pas prévenu. Sur le robot, '
                'sourcer ~/ros2_ws/install/setup.bash avant notre espace de travail ; '
                'sur un PC, compiler le dépôt PARC2026-Engineers-League.')
            return
        from sensor_msgs.msg import CompressedImage
        self._Finish, self._StartSignal = Finish, StartSignal
        self._TagVisit, self._CompressedImage = TagVisit, CompressedImage
        self._depart = noeud.create_publisher(StartSignal, configuration.start_topic, EVENEMENTS)
        self._visite = noeud.create_publisher(TagVisit, configuration.visit_topic, EVENEMENTS)
        self._fin = noeud.create_publisher(Finish, configuration.finish_topic, EVENEMENTS)
        noeud.create_subscription(RunStatus, configuration.status_topic,
                                  self.recevoir_statut, VERROUILLE)
        self.disponible = True

    # ------------------------------------------------------------ arbitre -> nous
    def recevoir_statut(self, message):
        etat = int(message.state)
        if etat != self.etat:
            self._noeud.get_logger().info(
                f'Arbitre : {NOMS_ETATS.get(etat, etat)} '
                f'({message.elapsed_s:.0f} s, tags signalés {list(message.tags_reported)}).')
        self.etat = etat
        if self._depart_envoye and etat == EN_COURSE:
            self._course_vue = True
        if message.false_start and not self._faux_depart_signale:
            self._faux_depart_signale = True
            self._noeud.get_logger().error('Arbitre : FAUX DÉPART (le robot a bougé avant le départ).')

    def course_terminee(self):
        """Vrai si l'arbitre a clos la course que nous avons lancée."""
        return self._course_vue and self.etat in ETATS_FINAUX

    def nom_etat(self):
        return NOMS_ETATS.get(self.etat, 'inconnu') if self.etat is not None else 'aucun statut'

    # ------------------------------------------------------------ nous -> arbitre
    def signaler_depart(self, mot_entendu=''):
        """Une fois, robot encore immobile. Avec un mot entendu : départ vocal (bonus)."""
        self._depart_envoye = True
        if not self.disponible:
            return
        if self.etat is not None and self.etat != 1:
            self._noeud.get_logger().warn(
                f'Arbitre dans l\'état {self.nom_etat()} et non ARMED : '
                'il ignorera probablement ce départ.')
        message = self._StartSignal()
        message.stamp = self._noeud.get_clock().now().to_msg()
        message.method = (self._StartSignal.METHOD_VOICE if mot_entendu
                          else self._StartSignal.METHOD_COMMAND)
        message.heard = mot_entendu or ''
        self._depart.publish(message)
        self._noeud.get_logger().info(
            'Arbitre prévenu du départ : '
            + (f'voix (« {mot_entendu} »).' if mot_entendu else 'commande.'))

    def signaler_visite(self, identifiant, image):
        """Robot arrêté devant le tag ; l'image doit montrer le tag (JPEG)."""
        if not self.disponible:
            return False
        message = self._TagVisit()
        message.header.stamp = self._noeud.get_clock().now().to_msg()
        message.tag_id = int(identifiant)
        compressee = self._image_jpeg(image)
        if compressee is not None:
            message.header.frame_id = compressee.header.frame_id
            message.image = compressee
        else:
            self._noeud.get_logger().warn(
                f'Visite du tag {identifiant} signalée SANS image (aucune image caméra).')
        self._visite.publish(message)
        self._noeud.get_logger().info(
            f'Arbitre prévenu de la visite du tag {identifiant}'
            + (f' (image JPEG {len(compressee.data) // 1024} ko).' if compressee else '.'))
        return compressee is not None

    def signaler_fin(self, abandon=False):
        if not self.disponible or self.course_terminee():
            return
        message = self._Finish()
        message.stamp = self._noeud.get_clock().now().to_msg()
        message.reason = self._Finish.REASON_GAVE_UP if abandon else self._Finish.REASON_COMPLETED
        self._fin.publish(message)
        self._noeud.get_logger().info(
            'Arbitre prévenu de la fin : ' + ('abandon.' if abandon else 'course terminée.'))

    def _image_jpeg(self, image):
        if image is None:
            return None
        if isinstance(image, self._CompressedImage):
            if 'jpeg' in image.format.lower() or 'jpg' in image.format.lower():
                return image
            return None
        try:
            jpeg = encoder_jpeg(image.encoding, image.height, image.width, image.step,
                                image.data, self._configuration.jpeg_quality)
        except Exception as erreur:  # noqa: BLE001 — une image ratée ne doit pas arrêter la course
            self._noeud.get_logger().error(f'Encodage JPEG impossible : {erreur}')
            return None
        if jpeg is None:
            self._noeud.get_logger().error(f'Encodage d\'image « {image.encoding} » non géré.')
            return None
        compressee = self._CompressedImage()
        compressee.header = image.header
        compressee.format = 'jpeg'
        compressee.data = jpeg
        return compressee
