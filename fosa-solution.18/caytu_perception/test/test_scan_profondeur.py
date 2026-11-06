"""Profondeur caméra -> obstacles par tranche de hauteur -> LaserScan."""

import math

import numpy as np
import pytest

from caytu_perception.scan_profondeur import (
    points_depuis_profondeur,
    scan_depuis_points,
    transformer_points,
)

K = np.array([[400.0, 0.0, 160.0], [0.0, 400.0, 120.0], [0.0, 0.0, 1.0]])
REGLAGES = dict(hauteur_min=0.10, hauteur_max=1.30, angle_min=-0.6, angle_max=0.6,
                increment=0.01, portee_min=0.2, portee_max=2.5)


def test_un_mur_plat_donne_des_points_a_la_bonne_profondeur():
    profondeur = np.full((240, 320), 2000, dtype=np.uint16)     # 2 m en mm
    points = points_depuis_profondeur(profondeur, K, pas=8, echelle=0.001)
    assert points.shape[1] == 3
    assert np.allclose(points[:, 2], 2.0)
    centre = points[np.argmin(np.hypot(points[:, 0], points[:, 1]))]
    assert abs(centre[0]) < 0.05 and abs(centre[1]) < 0.05


def test_les_pixels_sans_mesure_sont_ignores():
    profondeur = np.zeros((240, 320), dtype=np.uint16)
    profondeur[:10, :10] = 1500
    profondeur[20, 20] = 0
    points = points_depuis_profondeur(profondeur, K, pas=1, echelle=0.001)
    assert len(points) == 100


def test_la_transformation_place_les_points_dans_le_repere_du_robot():
    # Repère optique (z devant) vers repère robot (x devant), caméra à 1 m.
    points = transformer_points(
        np.array([[0.0, 0.0, 2.0]]), (0.1, 0.0, 1.0), (-0.5, 0.5, -0.5, 0.5))
    assert points[0] == pytest.approx([2.1, 0.0, 1.0], abs=1e-9)


def test_un_plateau_de_table_est_vu_mais_pas_le_sol_ni_le_plafond():
    plateau = np.array([[1.0, y, 0.75] for y in np.linspace(-0.3, 0.3, 21)])
    sol = np.array([[x, 0.0, 0.02] for x in np.linspace(0.5, 2.0, 20)])
    plafond = np.array([[1.5, 0.0, 2.0]])
    distances = scan_depuis_points(np.vstack([plateau, sol, plafond]), **REGLAGES)
    milieu = len(distances) // 2
    assert distances[milieu] == pytest.approx(1.0, abs=0.01)
    assert sum(math.isfinite(d) for d in distances) <= 40
    assert all(math.isinf(d) or d < 1.2 for d in distances)


def test_chaque_secteur_garde_l_obstacle_le_plus_proche():
    points = np.array([[2.0, 0.0, 0.5], [1.2, 0.0, 0.5], [0.1, 0.0, 0.5]])
    distances = scan_depuis_points(points, **REGLAGES)
    assert distances[len(distances) // 2] == pytest.approx(1.2)


def test_aucun_point_donne_un_scan_vide():
    distances = scan_depuis_points(np.zeros((0, 3)), **REGLAGES)
    assert len(distances) == 120 and all(math.isinf(d) for d in distances)


def test_un_point_isole_n_est_pas_un_obstacle():
    """Mesuré dans Gazebo : le bruit de profondeur faisait du sol un mur."""
    isole = np.array([[1.5, 0.0, 0.5]])
    groupe = np.array([[1.0, 0.002 * i, 0.8] for i in range(3)])
    distances = scan_depuis_points(np.vstack([isole, groupe]), points_min=3, **REGLAGES)
    assert distances[len(distances) // 2] == pytest.approx(1.0, abs=0.01)
    seul = scan_depuis_points(isole, points_min=3, **REGLAGES)
    assert all(math.isinf(d) for d in seul)


def _secteurs_vides_sur_un_mur(pas, increment):
    matrice = np.array([[420.0, 0.0, 424.0], [0.0, 420.0, 240.0], [0.0, 0.0, 1.0]])
    profondeur = np.full((480, 848), 1200, dtype=np.uint16)
    optique = points_depuis_profondeur(profondeur, matrice, pas=pas, echelle=0.001)
    robot = transformer_points(optique, (0.0, 0.0, 0.8), (-0.5, 0.5, -0.5, 0.5))
    reglages = dict(REGLAGES, angle_min=-0.7, angle_max=0.7, increment=increment)
    distances = scan_depuis_points(robot, points_min=3, **reglages)
    return sum(math.isinf(d) for d in distances)


def test_un_mur_ne_laisse_aucun_secteur_vide_si_le_secteur_couvre_une_colonne():
    """Mesuré : pas 6 px à fx 420 (0,014 rad) pour des secteurs de 0,0087 rad
    laissait 45 secteurs sur 184 vides, lus comme libres par les costmaps."""
    assert _secteurs_vides_sur_un_mur(pas=6, increment=0.0087) > 0
    assert _secteurs_vides_sur_un_mur(pas=4, increment=0.0175) == 0
