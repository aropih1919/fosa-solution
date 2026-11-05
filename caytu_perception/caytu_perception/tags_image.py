"""Détection et annotation AprilTag réutilisables sans ROS, TF ni calibration."""

from pathlib import Path

import cv2
import numpy as np


def creer_dictionnaire(famille):
    """Respecte la casse des constantes OpenCV (36h11, et non 36H11)."""
    nom = f'DICT_APRILTAG_{famille.lower()}'
    try:
        return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, nom))
    except AttributeError as erreur:
        raise RuntimeError(
            f'OpenCV avec cv2.aruco et la famille {famille} est requis.') from erreur


class DetecteurImage:
    """Conserve un détecteur OpenCV pour toutes les images."""

    def __init__(self, famille):
        self.dictionnaire = creer_dictionnaire(famille)
        # OpenCV < 4.7 (python3-opencv 4.6 d'Ubuntu 24.04) : la classe
        # DetectorParameters existe, mais DetectorParameters() donne un objet
        # inutilisable et la détection plante (segmentation fault). Il faut
        # DetectorParameters_create(), supprimé à partir de 4.7.
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.parametres = cv2.aruco.DetectorParameters_create()
        else:
            self.parametres = cv2.aruco.DetectorParameters()
        self.detecteur = (
            cv2.aruco.ArucoDetector(self.dictionnaire, self.parametres)
            if hasattr(cv2.aruco, 'ArucoDetector') else None)

    def detecter(self, image):
        if self.detecteur is not None:
            coins, ids, _ = self.detecteur.detectMarkers(image)
        else:
            coins, ids, _ = cv2.aruco.detectMarkers(
                image, self.dictionnaire, parameters=self.parametres)
        return coins, ids


def annoter_image(image, coins, ids):
    annotee = (cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
               if image.ndim == 2 else image.copy())
    if ids is not None and len(ids):
        cv2.aruco.drawDetectedMarkers(annotee, coins, ids)
    return annotee


def enregistrer_image(chemin, image):
    chemin = Path(chemin)
    chemin.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(chemin), image):
        raise OSError(f'Impossible d’enregistrer {chemin}')


def generer_scenes(dictionnaire, identifiants):
    """Fixtures déterministes : tags isolés, groupés, déformés et image vide.

    Les dimensions et déformations ci-dessous définissent le jeu d'essai,
    pas les réglages du robot. Aucune dimension physique n'est impliquée.
    """
    scenes = {'vide': (np.full((400, 400), 255, np.uint8), [])}
    planches = []
    for identifiant in identifiants:
        if hasattr(cv2.aruco, 'generateImageMarker'):
            marqueur = cv2.aruco.generateImageMarker(dictionnaire, identifiant, 240)
        else:
            marqueur = cv2.aruco.drawMarker(dictionnaire, identifiant, 240)
        image = np.full((400, 400), 255, np.uint8)
        image[80:320, 80:320] = marqueur
        scenes[f'tag_{identifiant}'] = (image, [identifiant])
        rotation = cv2.getRotationMatrix2D((200, 200), 25, 1)
        scenes[f'tag_{identifiant}_rotation'] = (
            cv2.warpAffine(image, rotation, (400, 400), borderValue=255), [identifiant])
        origine = np.float32([[0, 0], [399, 0], [399, 399], [0, 399]])
        cible = np.float32([[40, 25], [360, 65], [385, 370], [20, 340]])
        perspective = cv2.warpPerspective(
            image, cv2.getPerspectiveTransform(origine, cible),
            (400, 400), borderValue=255)
        scenes[f'tag_{identifiant}_perspective_flou'] = (
            cv2.GaussianBlur(perspective, (5, 5), 1), [identifiant])
        planches.append(image)
    if planches:
        scenes['ensemble'] = (np.hstack(planches), list(identifiants))
    return scenes
