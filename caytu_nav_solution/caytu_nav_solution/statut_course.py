"""Statut de la course : départ, chaque visite (identifiant + image), fin.

Le guide PARC annonce un « paquet de statut » des organisateurs, sans encore
le décrire. En attendant, tout part sur nos topics (caytu_config, course.*) :
  status_topic       std_msgs/String, JSON : {"evenement": "depart" | "visite"
                     | "saute" | "fin", "tag": 7, "ordre": [...], "temps_s": ...}
  visited_tag_topic  std_msgs/Int32 : identifiant du tag visité ;
  visit_image_topic  sensor_msgs/Image (ou CompressedImage sur .../compressed) :
                     image de la caméra au moment de la visite.
Quand les organisateurs publieront leur paquet, seul ce fichier change.
"""

import json

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32, String


class PublicateurStatutCourse:

    def __init__(self, noeud, course, equipe='Fosa'):
        self._noeud = noeud
        self._equipe = equipe
        fiable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._statut = noeud.create_publisher(String, course.status_topic, fiable)
        self._tag = noeud.create_publisher(Int32, course.visited_tag_topic, fiable)
        self._image = noeud.create_publisher(Image, course.visit_image_topic, fiable)
        self._image_compressee = noeud.create_publisher(
            CompressedImage, course.visit_image_topic + '/compressed', fiable)
        self.visites = []

    def _publier(self, evenement, temps_s, **champs):
        contenu = {'equipe': self._equipe, 'evenement': evenement,
                   'temps_s': round(float(temps_s), 1), 'visites': list(self.visites)}
        contenu.update(champs)
        self._statut.publish(String(data=json.dumps(contenu, ensure_ascii=False)))
        self._noeud.get_logger().info(f'STATUT {json.dumps(contenu, ensure_ascii=False)}')

    def depart(self, ordre, temps_s=0.0):
        self._publier('depart', temps_s, ordre=list(ordre))

    def visite(self, identifiant, image, temps_s, ecart_distance_m, ecart_angle_deg):
        self.visites.append(int(identifiant))
        self._tag.publish(Int32(data=int(identifiant)))
        if isinstance(image, CompressedImage):
            self._image_compressee.publish(image)
        elif image is not None:
            self._image.publish(image)
        self._publier('visite', temps_s, tag=int(identifiant),
                      image=image is not None,
                      ecart_distance_m=round(float(ecart_distance_m), 3),
                      ecart_angle_deg=round(float(ecart_angle_deg), 1))

    def saute(self, identifiant, raison, temps_s):
        self._publier('saute', temps_s, tag=int(identifiant), raison=raison)

    def fin(self, raison, temps_s):
        self._publier('fin', temps_s, raison=raison)
