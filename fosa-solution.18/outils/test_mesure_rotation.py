"""Calculs de outils/mesure_rotation.py (sans ROS en marche)."""

import math

import numpy as np
import pytest

from mesure_rotation import angle_lidar, decalage_scans, integrer, vitesse_lacet

INCREMENT = math.radians(0.5)


def _scan(cap):
    """Scan d'une pièce rectangulaire 4 x 6 m vue du centre, robot au cap `cap`."""
    angles = -math.pi + INCREMENT * np.arange(720) + cap
    ux, uy = np.cos(angles), np.sin(angles)
    with np.errstate(divide='ignore'):
        return np.minimum(np.abs(2.0 / ux), np.abs(3.0 / uy)).tolist()


def test_le_decalage_des_scans_donne_la_rotation_en_trop():
    debut = _scan(0.3)
    fin = _scan(0.3 + math.radians(7.0))
    decalage, _ = decalage_scans(debut, fin, INCREMENT, math.radians(30))
    assert math.degrees(decalage) == pytest.approx(7.0, abs=0.5)
    decalage, _ = decalage_scans(debut, _scan(0.3 - math.radians(4.0)), INCREMENT,
                                 math.radians(30))
    assert math.degrees(decalage) == pytest.approx(-4.0, abs=0.5)


def test_les_portees_nulles_ou_infinies_sont_ignorees():
    debut, fin = _scan(0.0), _scan(math.radians(3.0))
    fin[100:200] = [math.inf] * 100
    debut[300:340] = [0.0] * 40
    decalage, _ = decalage_scans(debut, fin, INCREMENT, math.radians(30))
    assert math.degrees(decalage) == pytest.approx(3.0, abs=0.5)


def test_le_lidar_donne_le_tour_choisi_par_l_odometrie():
    # Odométrie 728° ; le lidar voit 3° de plus qu'un nombre entier de tours.
    assert math.degrees(angle_lidar(math.radians(728), math.radians(3))) == pytest.approx(723)
    assert math.degrees(angle_lidar(math.radians(700), math.radians(-25))) == pytest.approx(695)


def test_le_gyrometre_est_ramene_sur_l_axe_vertical_du_robot():
    # Repère optique (z devant, y vers le bas) : le lacet du robot est -y capteur.
    rotation = (-0.5, 0.5, -0.5, 0.5)
    assert vitesse_lacet((0.0, -0.4, 0.0), rotation) == pytest.approx(0.4)
    assert vitesse_lacet((0.0, 0.0, 0.4), (0.0, 0.0, 0.0, 1.0)) == pytest.approx(0.4)


def test_l_integration_retire_le_biais():
    echantillons = [(0.1 * i, 0.5 + 0.01) for i in range(101)]     # 10 s
    assert integrer(echantillons, biais=0.01) == pytest.approx(5.0)

