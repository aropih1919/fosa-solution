"""Remplissage des messages de l'arbitre, sur des messages simulés."""

import array
from types import SimpleNamespace

import numpy as np

from caytu_course.arbitre import (
    contenu_ordre,
    remplir_depart,
    remplir_fin,
    remplir_visite,
)


class Message:
    """Imite un message rosidl : champs déclarés dans CHAMPS."""

    CHAMPS = {}

    def __init__(self, **valeurs):
        for nom in self.CHAMPS:
            setattr(self, nom, valeurs.get(nom))

    @classmethod
    def get_fields_and_field_types(cls):
        return dict(cls.CHAMPS)


class ImageCompressee(Message):
    CHAMPS = {'header': 'std_msgs/Header', 'format': 'string', 'data': 'sequence<uint8>'}


class VisiteCompressee(Message):
    CHAMPS = {'tag_id': 'int32', 'image': 'sensor_msgs/CompressedImage'}

    def __init__(self):
        super().__init__(image=ImageCompressee())


class VisiteOctets(Message):
    CHAMPS = {'stamp': 'builtin_interfaces/Time', 'tag_id': 'uint8',
              'image_format': 'string', 'jpeg': 'sequence<uint8>'}


class VisiteBrute(Message):
    CHAMPS = {'tag_id': 'int32', 'image': 'sensor_msgs/Image'}


class Depart(Message):
    CHAMPS = {'method': 'uint8', 'heard': 'string'}


class DepartSansTexte(Message):
    CHAMPS = {'method': 'uint8'}


class Fin(Message):
    CHAMPS = {'reason': 'uint8'}


ENTETE = SimpleNamespace(stamp='instant', frame_id='camera1_color_optical_frame')
JPEG = b'\xff\xd8\xff\xe0jpeg'


def test_l_image_compressee_porte_le_jpeg_et_l_entete():
    message = VisiteCompressee()
    assert remplir_visite(message, 7, JPEG, None, ENTETE) == ['image']
    assert message.tag_id == 7
    assert message.image.format == 'jpeg'
    assert bytes(message.image.data) == JPEG
    assert message.image.header is ENTETE


def test_un_tableau_d_octets_recoit_le_jpeg_et_son_format():
    message = VisiteOctets()
    assert remplir_visite(message, 4, JPEG, None, ENTETE) == ['jpeg']
    assert isinstance(message.jpeg, array.array)
    assert bytes(message.jpeg) == JPEG
    assert message.image_format == 'jpeg'
    assert message.stamp == 'instant'


def test_un_champ_image_brute_recoit_l_image_d_origine():
    message = VisiteBrute()
    brute = object()
    assert remplir_visite(message, 9, None, brute, ENTETE) == ['image']
    assert message.image is brute


def test_sans_image_seul_l_identifiant_part():
    message = VisiteCompressee()
    assert remplir_visite(message, 1, None, None, None) == []
    assert message.tag_id == 1


def test_le_depart_vocal_porte_le_mot_entendu():
    message = remplir_depart(Depart(), 1, 'allez')
    assert (message.method, message.heard) == (1, 'allez')
    assert remplir_depart(DepartSansTexte(), 0, None).method == 0


def test_la_fin_porte_la_raison():
    assert remplir_fin(Fin(), 0).reason == 0


class OrdreTexte(Message):
    CHAMPS = {'header': 'std_msgs/Header', 'data': 'string'}


class Dimension(Message):
    CHAMPS = {'label': 'string', 'size': 'uint32'}


class Disposition(Message):
    CHAMPS = {'dim': 'sequence<std_msgs/MultiArrayDimension>', 'data_offset': 'uint32'}


class OrdreTableau(Message):
    CHAMPS = {'layout': 'std_msgs/MultiArrayLayout', 'data': 'sequence<int32>'}


class OrdreFixe(Message):
    CHAMPS = {'tag_ids': 'int32[4]'}


class OrdreTextes(Message):
    CHAMPS = {'order': 'sequence<string>'}


def test_l_ordre_est_lu_dans_un_texte_en_sautant_l_entete():
    message = OrdreTexte(header=OrdreTexte(data='99'), data='7,1,9,4')
    assert contenu_ordre(message) == '7,1,9,4'


def test_l_ordre_est_lu_dans_un_tableau_d_entiers():
    message = OrdreTableau(
        layout=Disposition(dim=[], data_offset=0),
        data=array.array('i', [7, 1, 9, 4]))
    assert contenu_ordre(message) == [7, 1, 9, 4]


def test_l_ordre_est_lu_dans_un_tableau_numpy_de_taille_fixe():
    message = OrdreFixe(tag_ids=np.array([4, 9, 1, 7], dtype=np.int32))
    assert contenu_ordre(message) == [4, 9, 1, 7]


def test_l_ordre_est_lu_dans_une_liste_de_textes():
    assert contenu_ordre(OrdreTextes(order=['9', '4', '7', '1'])) == '9,4,7,1'


def test_un_message_sans_ordre_donne_none():
    assert contenu_ordre(OrdreTexte(header=None, data='  ')) is None
