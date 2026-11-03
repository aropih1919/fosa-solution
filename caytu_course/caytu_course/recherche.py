"""Points de recherche des tags dans la carte construite en direct.

On propose des points libres, éloignés des obstacles et de l'inconnu (au-delà
d'un mur, la carte est inconnue), espacés régulièrement et triés du plus
proche au plus lointain. Le robot y fait un tour sur lui-même pour balayer
les murs avec les caméras.
"""

from dataclasses import dataclass
import math

import numpy as np

SEUIL_OCCUPE = 50


@dataclass(frozen=True)
class GrilleOccupation:
    largeur: int
    hauteur: int
    resolution: float
    origine_x: float
    origine_y: float
    donnees: list   # nav_msgs/OccupancyGrid.data : ligne 0 = y minimal


def _dilater(masque, rayon):
    """Vrai pour chaque cellule à moins de `rayon` cellules d'un vrai (carré)."""
    if rayon <= 0:
        return masque.copy()
    integrale = np.pad(masque.astype(np.int32), ((1, 0), (1, 0))).cumsum(0).cumsum(1)
    hauteur, largeur = masque.shape
    lignes = np.arange(hauteur)
    colonnes = np.arange(largeur)
    haut = np.clip(lignes - rayon, 0, hauteur)[:, None]
    bas = np.clip(lignes + rayon + 1, 0, hauteur)[:, None]
    gauche = np.clip(colonnes - rayon, 0, largeur)[None, :]
    droite = np.clip(colonnes + rayon + 1, 0, largeur)[None, :]
    somme = (integrale[bas, droite] - integrale[haut, droite]
             - integrale[bas, gauche] + integrale[haut, gauche])
    return somme > 0


def points_recherche(grille, clairance_m, pas_m, visites, rayon_visite_m, position,
                     centre=None, rayon_max=None):
    """Liste de (x, y) dans le repère de la grille, du plus proche au plus loin.

    centre, rayon_max : zone autorisée (cercle autour du départ), pour ne pas
    partir vers le public si le lidar voit au-delà de l'arène.
    """
    attendu = grille.largeur * grille.hauteur
    if len(grille.donnees) != attendu:
        raise ValueError(
            f'grille incohérente : {len(grille.donnees)} cellules pour {attendu}')
    cellules = np.asarray(grille.donnees, dtype=np.int16).reshape(
        grille.hauteur, grille.largeur)
    libre = cellules == 0
    interdit = _dilater(
        (cellules >= SEUIL_OCCUPE) | (cellules < 0),
        int(math.ceil(clairance_m / grille.resolution)))
    autorise = libre & ~interdit

    pas = max(1, int(round(pas_m / grille.resolution)))
    lignes, colonnes = np.nonzero(autorise[pas // 2::pas, pas // 2::pas])
    lignes = lignes * pas + pas // 2
    colonnes = colonnes * pas + pas // 2
    xs = grille.origine_x + (colonnes + 0.5) * grille.resolution
    ys = grille.origine_y + (lignes + 0.5) * grille.resolution

    points = [
        (float(x), float(y)) for x, y in zip(xs, ys)
        if all(math.dist((x, y), visite) > rayon_visite_m for visite in visites)
        and (rayon_max is None or math.dist((x, y), centre) <= rayon_max)]
    return sorted(points, key=lambda point: math.dist(point, position))
