"""Quaternions et changements de repère, sans ROS (convention x, y, z, w)."""

import math

import numpy as np

def quaternion_depuis_matrice(matrice):
    """Quaternion (x, y, z, w) d'une matrice de rotation 3x3."""
    trace = matrice[0, 0] + matrice[1, 1] + matrice[2, 2]
    if trace > 0.0:
        echelle = math.sqrt(trace + 1.0) * 2.0
        return (float((matrice[2, 1] - matrice[1, 2]) / echelle),
                float((matrice[0, 2] - matrice[2, 0]) / echelle),
                float((matrice[1, 0] - matrice[0, 1]) / echelle),
                float(0.25 * echelle))
    index = int(np.argmax(np.diag(matrice)))
    suivant = (index + 1) % 3
    dernier = (index + 2) % 3
    echelle = math.sqrt(
        1.0 + matrice[index, index] - matrice[suivant, suivant]
        - matrice[dernier, dernier]) * 2.0
    resultat = [0.0, 0.0, 0.0, 0.0]
    resultat[index] = 0.25 * echelle
    resultat[3] = (matrice[dernier, suivant] - matrice[suivant, dernier]) / echelle
    resultat[suivant] = (matrice[suivant, index] + matrice[index, suivant]) / echelle
    resultat[dernier] = (matrice[dernier, index] + matrice[index, dernier]) / echelle
    return tuple(float(valeur) for valeur in resultat)


def multiplier_quaternions(gauche, droite):
    """Produit gauche ⊗ droite : applique `droite` puis `gauche`."""
    gx, gy, gz, gw = gauche
    dx, dy, dz, dw = droite
    return (
        gw * dx + gx * dw + gy * dz - gz * dy,
        gw * dy - gx * dz + gy * dw + gz * dx,
        gw * dz + gx * dy - gy * dx + gz * dw,
        gw * dw - gx * dx - gy * dy - gz * dz,
    )


def tourner_vecteur(quaternion, vecteur):
    """Applique la rotation d'un quaternion unitaire à un vecteur 3D."""
    x, y, z, w = quaternion
    vx, vy, vz = vecteur
    croix1 = (y * vz - z * vy, z * vx - x * vz, x * vy - y * vx)
    croix2 = (y * croix1[2] - z * croix1[1],
              z * croix1[0] - x * croix1[2],
              x * croix1[1] - y * croix1[0])
    return tuple(
        vecteur[i] + 2.0 * (w * croix1[i] + croix2[i]) for i in range(3))


def composer(translation, rotation, position, orientation):
    """Exprime dans le repère A une pose donnée dans le repère B.

    translation, rotation : transformation A <- B (position de B dans A).
    position, orientation : pose dans B.
    Retourne (position, orientation) dans A.
    """
    tournee = tourner_vecteur(rotation, position)
    position_a = tuple(translation[i] + tournee[i] for i in range(3))
    return position_a, multiplier_quaternions(rotation, orientation)
