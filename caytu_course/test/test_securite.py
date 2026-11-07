"""Les commandes directes protègent aussi les côtés et les pertes de capteurs."""

import math

import pytest

from caytu_course.securite import donnees_fraiches, trajectoire_libre

EMPREINTE = ((0.22, 0.25), (0.22, -0.25), (-0.31, -0.25), (-0.31, 0.25))


def libre(points, v, w):
    return trajectoire_libre(points, v, w, EMPREINTE, 0.02, 1.0, 0.05)


def libre_avec_tolerance(points, v, w, tolerance=0.02):
    return trajectoire_libre(
        points, v, w, EMPREINTE, 0.08, 1.0, 0.05, tolerance)


@pytest.mark.parametrize('points,v,w', [
    ([(0.4, 0.0)], 0.3, 0.0),
    ([(-0.45, 0.0)], -0.2, 0.0),
    ([(0.1, 0.33)], 0.0, 0.8),
    ([], math.nan, 0.0),
])
def test_collision_ou_commande_invalide(points, v, w):
    assert not libre(points, v, w)


def test_une_trajectoire_libre_reste_possible():
    assert libre([(2.0, 0.0), (-2.0, 0.0)], 0.2, 0.4)


@pytest.mark.parametrize('instants', [[], [None], [10.0, None], [9.0], [11.0]])
def test_capteur_absent_perime_ou_futur(instants):
    assert not donnees_fraiches(instants, 10.0, 0.6)


def test_tous_les_capteurs_doivent_etre_frais():
    assert donnees_fraiches([9.5, 9.8, 10.0], 10.0, 0.6)


def test_un_obstacle_lateral_n_immobilise_plus():
    """Déjà dans la marge : avance, recul et rotation lente restent possibles."""
    assert libre_avec_tolerance([(0.0, 0.30)], 0.1, 0.0)
    assert libre_avec_tolerance([(0.0, 0.30)], -0.1, 0.0)
    assert libre_avec_tolerance([(0.0, 0.30)], 0.0, 0.2)
    # Une rotation rapide aggraverait trop la pénétration : elle est refusée.
    assert not libre_avec_tolerance([(0.0, 0.30)], 0.0, 0.8)


def test_un_obstacle_devant_bloque_l_avance_mais_autorise_le_recul():
    points = [(0.35, 0.0)]
    assert not libre_avec_tolerance(points, 0.1, 0.0)
    assert libre_avec_tolerance(points, -0.1, 0.0)


def test_un_obstacle_derriere_bloque_le_recul_mais_autorise_l_avance():
    points = [(-0.40, 0.0)]
    assert not libre_avec_tolerance(points, -0.1, 0.0)
    assert libre_avec_tolerance(points, 0.1, 0.0)


def test_la_tolerance_borne_l_aggravation():
    """Sans tolérance, la rotation lente elle-même est refusée."""
    assert not libre_avec_tolerance([(0.0, 0.30)], 0.0, 0.2, tolerance=0.0)
