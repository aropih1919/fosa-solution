"""Points de recherche des tags dans la grille d'occupation SLAM."""

import math

import numpy as np
import pytest

from caytu_course.recherche import GrilleOccupation, points_recherche

RESOLUTION = 0.05


def _arene(cote_m=5.0, epaisseur_mur=2, inconnu_bord=0):
    """Arène carrée libre entourée de murs, origine en (0, 0)."""
    cellules = int(round(cote_m / RESOLUTION))
    donnees = np.zeros((cellules, cellules), dtype=np.int16)
    donnees[:epaisseur_mur, :] = 100
    donnees[-epaisseur_mur:, :] = 100
    donnees[:, :epaisseur_mur] = 100
    donnees[:, -epaisseur_mur:] = 100
    if inconnu_bord:
        donnees[:, -inconnu_bord:] = -1
    return GrilleOccupation(
        cellules, cellules, RESOLUTION, 0.0, 0.0, donnees.flatten().tolist())


def test_les_points_restent_loin_des_murs_et_sont_espaces():
    points = points_recherche(_arene(), 0.45, 0.8, [], 0.8, (2.5, 2.5))
    assert len(points) >= 9
    mur = 2 * RESOLUTION
    for x, y in points:
        assert min(x - mur, y - mur, 5.0 - mur - x, 5.0 - mur - y) >= 0.45 - RESOLUTION
    for i, premier in enumerate(points):
        for second in points[i + 1:]:
            assert math.dist(premier, second) >= 0.8 - 1e-6


def test_les_points_sont_tries_par_distance_au_robot():
    position = (1.0, 1.0)
    points = points_recherche(_arene(), 0.45, 0.8, [], 0.8, position)
    distances = [math.dist(point, position) for point in points]
    assert distances == sorted(distances)


def test_les_zones_deja_visitees_sont_exclues():
    tous = points_recherche(_arene(), 0.45, 0.8, [], 0.8, (2.5, 2.5))
    visite = tous[0]
    restants = points_recherche(_arene(), 0.45, 0.8, [visite], 0.8, (2.5, 2.5))
    assert all(math.dist(point, visite) > 0.8 for point in restants)
    assert len(restants) < len(tous)


def test_l_inconnu_est_evite_comme_un_mur():
    points = points_recherche(_arene(inconnu_bord=40), 0.45, 0.8, [], 0.8, (1.0, 2.5))
    assert points
    assert all(x <= 5.0 - 40 * RESOLUTION - 0.45 + RESOLUTION for x, _ in points)


def test_les_points_restent_dans_le_rayon_autour_du_depart():
    tous = points_recherche(_arene(), 0.45, 0.8, [], 0.8, (2.5, 2.5))
    proches = points_recherche(
        _arene(), 0.45, 0.8, [], 0.8, (2.5, 2.5), centre=(1.0, 1.0), rayon_max=1.5)
    assert proches and len(proches) < len(tous)
    assert all(math.dist(point, (1.0, 1.0)) <= 1.5 for point in proches)


def test_une_grille_inconnue_ne_donne_aucun_point():
    grille = GrilleOccupation(20, 20, RESOLUTION, 0.0, 0.0, [-1] * 400)
    assert points_recherche(grille, 0.45, 0.8, [], 0.8, (0.5, 0.5)) == []


def test_une_grille_incoherente_est_refusee():
    with pytest.raises(ValueError):
        points_recherche(GrilleOccupation(10, 10, RESOLUTION, 0.0, 0.0, [0] * 5),
                         0.45, 0.8, [], 0.8, (0.0, 0.0))
