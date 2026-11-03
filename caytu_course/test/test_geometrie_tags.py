"""Géométrie de visite : normale du tag, pose d'approche, erreurs."""

import math

import numpy as np
import pytest

from caytu_course.geometrie_tags import (
    cap_normale,
    erreurs_visite,
    pose_approche,
    visite_valide,
)


def _quaternion(matrice):
    """Quaternion (x, y, z, w) d'une matrice de rotation (méthode de Shepperd)."""
    trace = float(np.trace(matrice))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return ((matrice[2, 1] - matrice[1, 2]) / s, (matrice[0, 2] - matrice[2, 0]) / s,
                (matrice[1, 0] - matrice[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(matrice)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + matrice[i, i] - matrice[j, j] - matrice[k, k]) * 2.0
    q = [0.0] * 4
    q[i] = 0.25 * s
    q[3] = (matrice[k, j] - matrice[j, k]) / s
    q[j] = (matrice[j, i] + matrice[i, j]) / s
    q[k] = (matrice[k, i] + matrice[i, k]) / s
    return tuple(q)


def _tag_vertical(cap):
    """Tag au mur dont la normale (axe z) pointe horizontalement vers `cap`."""
    normale = np.array([math.cos(cap), math.sin(cap), 0.0])
    haut = np.array([0.0, 0.0, 1.0])
    droite = np.cross(haut, normale)
    return _quaternion(np.column_stack([droite, haut, normale]))


def test_regression_audit_tag_face_au_robot():
    """Tag en (2, 0) face au robot : approche en (1,5 ; 0), cap 0 (bug de 90°)."""
    cap = cap_normale(*_tag_vertical(math.pi))
    assert abs(abs(cap) - math.pi) < 1e-9
    x, y, cap_robot = pose_approche(2.0, 0.0, cap, 0.5)
    assert (x, y) == pytest.approx((1.5, 0.0), abs=1e-9)
    assert math.cos(cap_robot) == pytest.approx(1.0, abs=1e-9)


@pytest.mark.parametrize('cap', [0.0, 0.7, -2.0, math.pi / 2])
def test_la_pose_d_approche_fait_face_au_tag(cap):
    x, y, cap_robot = pose_approche(1.0, -1.0, cap_normale(*_tag_vertical(cap)), 0.5)
    assert math.hypot(x - 1.0, y + 1.0) == pytest.approx(0.5)
    # Le robot regarde le tag.
    assert math.atan2(-1.0 - y, 1.0 - x) == pytest.approx(
        math.atan2(math.sin(cap_robot), math.cos(cap_robot)), abs=1e-9)


def test_un_tag_pose_a_plat_n_a_pas_de_normale_horizontale():
    assert cap_normale(0.0, 0.0, 0.0, 1.0) is None


def test_erreurs_nulles_quand_le_robot_est_bien_place():
    erreurs = erreurs_visite(0.5, 0.0, math.pi, 0.0, 0.5)
    assert erreurs.distance == pytest.approx(0.0, abs=1e-9)
    assert erreurs.laterale == pytest.approx(0.0, abs=1e-9)
    assert erreurs.cap == pytest.approx(0.0, abs=1e-9)


def test_erreur_de_distance_positive_quand_le_robot_est_trop_loin():
    assert erreurs_visite(0.6, 0.0, math.pi, 0.0, 0.5).distance == pytest.approx(0.1)


def test_erreur_de_cap_signee():
    """Robot tourné de +10° : il doit tourner de -10° (vers la droite)."""
    angle = math.radians(10.0)
    x, y = 0.5 * math.cos(-angle), 0.5 * math.sin(-angle)
    erreurs = erreurs_visite(x, y, math.pi - angle, 0.0, 0.5)
    assert math.degrees(erreurs.cap) == pytest.approx(-10.0, abs=1e-6)
    assert erreurs.distance == pytest.approx(0.0, abs=1e-9)


def test_erreur_laterale():
    assert abs(erreurs_visite(0.5, 0.05, math.pi, 0.0, 0.5).laterale) == pytest.approx(0.05)


def test_le_centre_du_robot_est_pris_en_compte():
    """Centre géométrique 5 cm derrière base_link : tag à 0,45 m de base_link."""
    assert erreurs_visite(0.45, 0.0, math.pi, -0.05, 0.5).distance == pytest.approx(0.0)


def test_visite_valide_selon_les_tolerances():
    bonne = erreurs_visite(0.52, 0.03, math.pi, 0.0, 0.5)
    assert visite_valide(bonne, 0.04, 0.06, math.radians(5.0))
    mauvaise = erreurs_visite(0.5, 0.0, math.pi - math.radians(8.0), 0.0, 0.5)
    assert not visite_valide(mauvaise, 0.04, 0.06, math.radians(5.0))
