"""Remplissage des messages de l'arbitre (`parc_msgs`) sans ROS.

Seuls les noms `method`, `heard`, `tag_id` et `reason` sont connus par les
consignes des organisateurs. Le champ de l'image de `TagVisit` et le type de
`/parc/tag_order` sont lus dans la définition du message chargée sur le
robot (`get_fields_and_field_types`) : le code suit la vraie définition au
lieu de la deviner.
"""

import array
from numbers import Integral

TYPES_IMAGE_COMPRESSEE = ('sensor_msgs/CompressedImage', 'sensor_msgs/msg/CompressedImage')
TYPES_IMAGE_BRUTE = ('sensor_msgs/Image', 'sensor_msgs/msg/Image')
TYPES_ENTETE = ('std_msgs/Header', 'std_msgs/msg/Header')
TYPES_INSTANT = ('builtin_interfaces/Time', 'builtin_interfaces/msg/Time')
CHAMPS_FORMAT = ('format', 'image_format', 'encoding')


def _champs(message):
    lire = getattr(message, 'get_fields_and_field_types', None)
    return dict(lire()) if lire is not None else {}


def _est_octets(type_champ):
    return type_champ.startswith(('sequence<uint8', 'uint8[', 'sequence<byte', 'byte['))


def remplir_depart(message, methode, entendu):
    champs = _champs(message)
    message.method = int(methode)
    if 'heard' in champs:
        message.heard = entendu or ''
    _remplir_horodatage(message, champs, None)
    return message


def remplir_fin(message, raison):
    message.reason = int(raison)
    _remplir_horodatage(message, _champs(message), None)
    return message


def remplir_visite(message, tag_id, jpeg, image_brute, entete):
    """Identifiant + image de preuve. Retourne les champs image remplis.

    jpeg : octets JPEG (ou None) ; image_brute : sensor_msgs/Image d'origine ;
    entete : std_msgs/Header de l'image (horodatage et repère de la caméra).
    """
    champs = _champs(message)
    message.tag_id = int(tag_id)
    remplis = []
    for nom, type_champ in champs.items():
        if nom == 'tag_id':
            continue
        if type_champ in TYPES_IMAGE_COMPRESSEE and jpeg is not None:
            image = getattr(message, nom)
            if entete is not None:
                image.header = entete
            image.format = 'jpeg'
            image.data = array.array('B', jpeg)
            remplis.append(nom)
        elif _est_octets(type_champ) and jpeg is not None:
            setattr(message, nom, array.array('B', jpeg))
            remplis.append(nom)
        elif type_champ in TYPES_IMAGE_BRUTE and image_brute is not None:
            setattr(message, nom, image_brute)
            remplis.append(nom)
        elif type_champ == 'string' and nom in CHAMPS_FORMAT:
            setattr(message, nom, 'jpeg')
    _remplir_horodatage(message, champs, entete)
    return remplis


def _remplir_horodatage(message, champs, entete):
    if entete is None:
        return
    for nom, type_champ in champs.items():
        if type_champ in TYPES_ENTETE:
            setattr(message, nom, entete)
        elif type_champ in TYPES_INSTANT:
            setattr(message, nom, entete.stamp)


def contenu_ordre(message):
    """Texte ou liste d'entiers portant l'ordre, quel que soit le type du message.

    Premier champ non vide qui est un texte, une liste d'entiers ou une liste
    de textes ; les sous-messages (sauf l'en-tête) sont parcourus dans l'ordre.
    """
    if isinstance(message, (str, list, tuple)):
        return message
    champs = _champs(message)
    if not champs:
        return None
    for nom, type_champ in champs.items():
        if type_champ in TYPES_ENTETE:
            continue
        valeur = getattr(message, nom)
        if isinstance(valeur, str):
            if valeur.strip():
                return valeur
            continue
        if _champs(valeur):
            trouve = contenu_ordre(valeur)
            if trouve is not None:
                return trouve
            continue
        if isinstance(valeur, (bytes, bytearray)) or not hasattr(valeur, '__len__'):
            continue
        elements = list(valeur)
        if elements and all(isinstance(element, Integral) for element in elements):
            return [int(element) for element in elements]
        if elements and all(isinstance(element, str) for element in elements):
            return ','.join(elements)
    return None
