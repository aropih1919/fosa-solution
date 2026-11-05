"""Image de visite pour l'arbitre : JPEG dans lequel le tag reste lisible."""

import cv2
import numpy as np

from caytu_nav_solution.arbitre_parc import encoder_jpeg


def _image_tag(identifiant, canaux=3):
    dictionnaire = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        marqueur = cv2.aruco.generateImageMarker(dictionnaire, identifiant, 200)
    else:
        marqueur = cv2.aruco.drawMarker(dictionnaire, identifiant, 200)
    gris = np.full((480, 640), 230, np.uint8)
    gris[140:340, 220:420] = marqueur
    return gris if canaux == 1 else np.repeat(gris[:, :, None], canaux, axis=2)


def _identifiants(jpeg):
    image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
    dictionnaire = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, 'ArucoDetector'):
        _, ids, _ = cv2.aruco.ArucoDetector(
            dictionnaire, cv2.aruco.DetectorParameters()).detectMarkers(image)
    else:
        _, ids, _ = cv2.aruco.detectMarkers(
            image, dictionnaire, parameters=cv2.aruco.DetectorParameters_create())
    return [] if ids is None else ids.ravel().tolist()


def test_rgb8_devient_un_jpeg_ou_le_tag_se_lit():
    image = _image_tag(7)
    jpeg = encoder_jpeg('rgb8', 480, 640, 640 * 3, image.tobytes(), 90)
    assert jpeg[:2] == b'\xff\xd8'
    assert _identifiants(jpeg) == [7]


def test_lignes_avec_remplissage_et_niveaux_de_gris():
    image = _image_tag(4)
    pas = 640 * 3 + 64                               # octets de remplissage en fin de ligne
    brut = np.zeros((480, pas), np.uint8)
    brut[:, :640 * 3] = image.reshape(480, -1)
    assert _identifiants(encoder_jpeg('bgr8', 480, 640, pas, brut.tobytes(), 90)) == [4]
    gris = _image_tag(9, canaux=1)
    assert _identifiants(encoder_jpeg('mono8', 480, 640, 640, gris.tobytes(), 90)) == [9]


def test_encodage_inconnu_ou_image_tronquee():
    assert encoder_jpeg('16UC1', 480, 640, 1280, bytes(480 * 1280), 90) is None
    assert encoder_jpeg('rgb8', 480, 640, 640 * 3, bytes(100), 90) is None
