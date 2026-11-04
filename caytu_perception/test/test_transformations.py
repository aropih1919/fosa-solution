"""Tests des quaternions et changements de repère (sans ROS)."""

import math

import numpy as np

from caytu_perception.transformations import (
    composer,
    multiplier_quaternions,
    quaternion_depuis_matrice,
    tourner_vecteur,
)

DEMI = math.sqrt(2.0) / 2.0


def _proches(gauche, droite, tolerance=1e-9):
    return all(
        abs(a - b) < tolerance for a, b in zip(gauche, droite))


def test_la_matrice_identite_donne_le_quaternion_unite():
    assert _proches(quaternion_depuis_matrice(np.eye(3)), (0.0, 0.0, 0.0, 1.0))


def test_une_rotation_de_90_degres_autour_de_z():
    matrice = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ])
    assert _proches(
        quaternion_depuis_matrice(matrice), (0.0, 0.0, DEMI, DEMI), 1e-7)


def test_une_rotation_de_180_degres_autour_de_x():
    matrice = np.array([
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
    ])
    assert _proches(
        quaternion_depuis_matrice(matrice), (1.0, 0.0, 0.0, 0.0), 1e-7)


def test_le_produit_par_l_identite_ne_change_rien():
    quaternion = (0.1, 0.2, 0.3, math.sqrt(0.86))
    identite = (0.0, 0.0, 0.0, 1.0)
    assert _proches(multiplier_quaternions(quaternion, identite), quaternion)
    assert _proches(multiplier_quaternions(identite, quaternion), quaternion)


def test_deux_rotations_de_90_degres_donnent_un_demi_tour():
    quart = (0.0, 0.0, DEMI, DEMI)
    assert _proches(multiplier_quaternions(quart, quart), (0.0, 0.0, 1.0, 0.0))


def test_une_rotation_de_90_degres_envoie_x_sur_y():
    quart = (0.0, 0.0, DEMI, DEMI)
    assert _proches(tourner_vecteur(quart, (1.0, 0.0, 0.0)), (0.0, 1.0, 0.0))
    assert _proches(
        tourner_vecteur((0.0, 0.0, 1.0, 0.0), (1.0, 0.0, 0.0)), (-1.0, 0.0, 0.0))


def test_une_pose_est_exprimee_dans_le_repere_parent():
    position, orientation = composer(
        (2.0, 0.0, 0.0), (0.0, 0.0, DEMI, DEMI),
        (1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert _proches(position, (2.0, 1.0, 0.0))
    assert _proches(orientation, (0.0, 0.0, DEMI, DEMI))
