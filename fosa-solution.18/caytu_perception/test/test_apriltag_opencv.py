"""Tests du module pur de détection AprilTag sur des images synthétiques.

Les images sont rendues par projection perspective d'un vrai tag 36h11
(marge blanche d'un bit, comme les tags imprimés officiels) posé sur un fond
gris : c'est le cas qui faisait échouer les paramètres OpenCV par défaut.
"""

import math

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')

from caytu_perception.apriltag_opencv import (  # noqa: E402
    DetecteurAprilTag,
    estimer_pose,
)

TAILLE_TAG = 0.165
LARGEUR, HAUTEUR, CHAMP_DEG = 640, 480, 69.4
FOCALE = (LARGEUR / 2) / math.tan(math.radians(CHAMP_DEG) / 2)
MATRICE = np.array([[FOCALE, 0, LARGEUR / 2], [0, FOCALE, HAUTEUR / 2], [0, 0, 1.0]])
DISTORSION = np.zeros(5)


def _texture(tag_id, pixels=400):
    dictionnaire = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        interieur = cv2.aruco.generateImageMarker(dictionnaire, tag_id, pixels)
    else:
        interieur = cv2.aruco.drawMarker(dictionnaire, tag_id, pixels)
    bit = pixels // 8
    return cv2.copyMakeBorder(
        interieur, bit, bit, bit, bit, cv2.BORDER_CONSTANT, value=255), bit


def _rendre(tag_id, distance, lacet_deg=0.0, fond=140):
    """Tag vertical face à la caméra, tourné de `lacet_deg` autour de la verticale."""
    texture, bit = _texture(tag_id)
    cote = texture.shape[0]
    demi = TAILLE_TAG * cote / (cote - 2 * bit) / 2
    rotation_tag = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1.0]])
    angle = math.radians(lacet_deg)
    rotation_lacet = np.array([
        [math.cos(angle), 0, math.sin(angle)], [0, 1, 0],
        [-math.sin(angle), 0, math.cos(angle)]])
    coins = np.array([[-demi, demi, 0], [demi, demi, 0], [demi, -demi, 0], [-demi, -demi, 0]])
    points = (rotation_lacet @ rotation_tag @ coins.T).T + np.array([0.0, 0.0, distance])
    projetes = (MATRICE @ points.T).T
    projetes = (projetes[:, :2] / projetes[:, 2:3]).astype(np.float32)
    source = np.float32([[0, 0], [cote, 0], [cote, cote], [0, cote]])
    homographie = cv2.getPerspectiveTransform(source, projetes)
    image = cv2.warpPerspective(
        texture, homographie, (LARGEUR, HAUTEUR), flags=cv2.INTER_AREA,
        borderValue=fond)
    bruit = np.random.default_rng(tag_id).normal(0, 2, image.shape)
    return np.clip(image + bruit, 0, 255).astype(np.uint8)


@pytest.mark.parametrize('distance', [1.0, 3.0])
@pytest.mark.parametrize('tag_id', [1, 4, 7, 9])
def test_les_tags_officiels_sont_detectes_sur_fond_sombre(tag_id, distance):
    detecteur = DetecteurAprilTag('36h11', 0.01, (1, 4, 7, 9))
    detections = detecteur.detecter(_rendre(tag_id, distance))
    assert [detection.tag_id for detection in detections] == [tag_id]


def test_un_tag_hors_liste_est_ignore():
    detecteur = DetecteurAprilTag('36h11', 0.01, (1, 4, 7, 9))
    assert detecteur.detecter(_rendre(3, 1.0)) == []


# Précision exigée surtout à 0,5 m (validation de la visite) ; de loin, la
# pose sert seulement à choisir le point d'approche.
@pytest.mark.parametrize('distance, tolerance', [(0.5, 0.01), (1.0, 0.02), (2.0, 0.06)])
def test_la_distance_utilise_la_taille_reelle_du_tag(distance, tolerance):
    detecteur = DetecteurAprilTag('36h11', 0.01, (7,))
    detection = detecteur.detecter(_rendre(7, distance))[0]
    rotation, translation = estimer_pose(
        detection.coins, TAILLE_TAG, MATRICE, DISTORSION)
    assert np.linalg.norm(translation) == pytest.approx(distance, abs=tolerance)
    # Axe z du tag (normale) dirigé vers la caméra : composante z négative.
    assert rotation[2, 2] == pytest.approx(-1.0, abs=0.02)


def test_l_orientation_du_tag_est_mesuree():
    detecteur = DetecteurAprilTag('36h11', 0.01, (4,))
    detection = detecteur.detecter(_rendre(4, 1.2, lacet_deg=30.0))[0]
    rotation, _ = estimer_pose(detection.coins, TAILLE_TAG, MATRICE, DISTORSION)
    normale = rotation[:, 2]
    angle = math.degrees(math.atan2(-normale[0], -normale[2]))
    assert abs(angle) == pytest.approx(30.0, abs=3.0)


def test_une_image_vide_ne_donne_rien():
    detecteur = DetecteurAprilTag('36h11', 0.01, (1, 4, 7, 9))
    assert detecteur.detecter(np.full((HAUTEUR, LARGEUR), 140, np.uint8)) == []


def test_une_famille_inconnue_est_refusee():
    with pytest.raises(ValueError, match='famille'):
        DetecteurAprilTag('99h99', 0.01, (1,))
