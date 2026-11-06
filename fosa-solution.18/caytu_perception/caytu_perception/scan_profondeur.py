"""Obstacles vus en profondeur, résumés en LaserScan horizontal (sans ROS).

Le lidar, monté bas, ne voit que les pieds des tables : un plateau qui
dépasse touche le robot (contact mesuré en simulation). Les points de
profondeur des caméras sont ramenés dans le repère du robot, gardés entre
deux hauteurs (ni le sol, ni au-dessus du robot), puis réduits à la distance
la plus proche par secteur angulaire, comme un lidar.
"""

import math

import numpy as np


def points_depuis_profondeur(profondeur, matrice, pas, echelle):
    """Points (N x 3) du repère optique depuis une image de profondeur.

    profondeur : image H x W (uint16 en mm ou float en m) ; echelle : facteur
    vers les mètres ; pas : un pixel sur `pas` dans chaque direction.
    """
    hauteur, largeur = profondeur.shape[:2]
    lignes, colonnes = np.mgrid[0:hauteur:pas, 0:largeur:pas]
    z = profondeur[::pas, ::pas].astype(np.float64) * echelle
    valides = np.isfinite(z) & (z > 0.0)
    z = z[valides]
    fx, fy, cx, cy = matrice[0, 0], matrice[1, 1], matrice[0, 2], matrice[1, 2]
    x = (colonnes[valides] - cx) * z / fx
    y = (lignes[valides] - cy) * z / fy
    return np.stack([x, y, z], axis=1)


def _matrice_rotation(quaternion):
    x, y, z, w = quaternion
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def transformer_points(points, translation, quaternion):
    """Applique la transformation cible <- capteur à des points (N x 3)."""
    return points @ _matrice_rotation(quaternion).T + np.asarray(translation)


def scan_depuis_points(points, hauteur_min, hauteur_max, angle_min, angle_max,
                       increment, portee_min, portee_max, points_min=1):
    """Distances par secteur (inf si libre) des points du repère du robot.

    Un secteur n'est occupé que s'il contient au moins `points_min` points :
    un point isolé est du bruit de profondeur, pas un obstacle.
    """
    nombre = int(round((angle_max - angle_min) / increment))
    distances = np.full(nombre, math.inf)
    if len(points) == 0:
        return distances.tolist()
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    portee = np.hypot(x, y)
    angle = np.arctan2(y, x)
    secteur = np.floor((angle - angle_min) / increment).astype(np.int64)
    garde = ((z >= hauteur_min) & (z <= hauteur_max)
             & (portee >= portee_min) & (portee <= portee_max)
             & (secteur >= 0) & (secteur < nombre))
    np.minimum.at(distances, secteur[garde], portee[garde])
    effectifs = np.bincount(secteur[garde], minlength=nombre)
    distances[effectifs < points_min] = math.inf
    return distances.tolist()
