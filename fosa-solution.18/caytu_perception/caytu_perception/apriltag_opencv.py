"""Détection AprilTag et estimation de pose avec OpenCV, sans ROS.

Compatibilité vérifiée :
  - OpenCV ≥ 4.7 : `cv2.aruco.ArucoDetector` ;
  - OpenCV 4.6 (apt Ubuntu 24.04 / ROS Jazzy) : `DetectorParameters_create()`
    + `detectMarkers()`. Construire `DetectorParameters()` puis appeler
    `detectMarkers()` provoque une erreur de segmentation sur cette version.

Repère du tag retourné par `estimer_pose` : x à droite, y en haut, z sortant
du tag vers la caméra (normale de la surface).
"""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class DetectionBrute:
    tag_id: int
    coins: np.ndarray  # (4, 2) : haut-gauche, haut-droit, bas-droit, bas-gauche


def _dictionnaire(famille):
    nom = f'DICT_APRILTAG_{famille}'
    identifiant = getattr(cv2.aruco, nom, None)
    if identifiant is None:
        identifiant = getattr(cv2.aruco, nom.upper(), None)
    if identifiant is None:
        raise ValueError(f'famille AprilTag inconnue pour OpenCV : {famille}')
    return cv2.aruco.getPredefinedDictionary(identifiant)


def _parametres(taux_distance_min):
    if hasattr(cv2.aruco, 'ArucoDetector'):
        parametres = cv2.aruco.DetectorParameters()
    else:
        parametres = cv2.aruco.DetectorParameters_create()
    # Un tag officiel n'a qu'un bit de marge blanche : sur un fond plus sombre,
    # le bord du papier forme un second carré trop proche du vrai tag.
    parametres.minMarkerDistanceRate = float(taux_distance_min)
    # Les coins sont affinés ensuite avec une fenêtre adaptée à la taille du
    # tag (voir _affiner_coins) : une fenêtre fixe biaise la distance de ±8 %.
    parametres.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    return parametres


_CRITERE_AFFINAGE = (
    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)


def _affiner_coins(image_gris, coins):
    """Affinage sous-pixel avec une demi-cellule du tag comme fenêtre.

    Un tag 36h11 mesure 8 cellules de côté (bordure noire comprise). Une
    fenêtre plus grande que la demi-cellule irait chercher le coin du papier.
    """
    cote = np.mean([
        np.linalg.norm(coins[index] - coins[(index + 1) % 4])
        for index in range(4)])
    fenetre = int(max(2, min(12, round(cote / 8.0 * 0.5))))
    affines = cv2.cornerSubPix(
        image_gris, coins.astype(np.float32).reshape(-1, 1, 2),
        (fenetre, fenetre), (-1, -1), _CRITERE_AFFINAGE)
    return affines.reshape(4, 2).astype(np.float64)


def _aire(coins):
    x, y = coins[:, 0], coins[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))


class DetecteurAprilTag:
    """Détecte les tags d'une famille et ne garde que les identifiants utiles."""

    def __init__(self, famille, taux_distance_min, ids_autorises):
        self._dictionnaire = _dictionnaire(famille)
        self._parametres = _parametres(taux_distance_min)
        self._ids_autorises = frozenset(int(tag) for tag in ids_autorises)
        self._detecteur = None
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self._detecteur = cv2.aruco.ArucoDetector(
                self._dictionnaire, self._parametres)

    def detecter(self, image_gris):
        """Retourne les tags autorisés, triés par identifiant."""
        if self._detecteur is not None:
            coins, ids, _ = self._detecteur.detectMarkers(image_gris)
        else:
            coins, ids, _ = cv2.aruco.detectMarkers(
                image_gris, self._dictionnaire, parameters=self._parametres)
        if ids is None:
            return []
        # Un même tag peut être décodé deux fois (bord noir et bord du papier) :
        # on garde le plus petit quadrilatère, qui est le vrai bord du tag.
        retenus = {}
        for coin, tag_id in zip(coins, ids.flatten()):
            tag_id = int(tag_id)
            if tag_id not in self._ids_autorises:
                continue
            coin = coin.reshape(4, 2).astype(np.float64)
            if tag_id not in retenus or _aire(coin) < _aire(retenus[tag_id]):
                retenus[tag_id] = coin
        return [
            DetectionBrute(tag_id, _affiner_coins(image_gris, retenus[tag_id]))
            for tag_id in sorted(retenus)]


def estimer_pose(coins, taille_m, matrice_camera, distorsion):
    """Pose du tag dans le repère optique : (rotation 3x3, translation 3).

    `SOLVEPNP_IPPE_SQUARE` est la méthode dédiée aux carrés plans : elle évite
    les solutions retournées de la méthode itérative. Retourne None si OpenCV
    ne trouve pas de solution.
    """
    demi = taille_m / 2.0
    points_objet = np.array([
        [-demi, demi, 0.0], [demi, demi, 0.0],
        [demi, -demi, 0.0], [-demi, -demi, 0.0]], dtype=np.float64)
    trouve, vecteur_rotation, translation = cv2.solvePnP(
        points_objet, np.asarray(coins, dtype=np.float64),
        np.asarray(matrice_camera, dtype=np.float64),
        np.asarray(distorsion, dtype=np.float64),
        flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not trouve:
        return None
    rotation, _ = cv2.Rodrigues(vecteur_rotation)
    return rotation, translation.reshape(3)
