"""Commande d'alignement final devant un tag."""

from dataclasses import replace
import math

import pytest

from caytu_course.alignement import (
    ReglagesAlignement,
    commande_alignement,
    obstacle_devant,
)
from caytu_course.geometrie_tags import ErreursVisite

REGLAGES = ReglagesAlignement(
    tolerance_distance=0.04, tolerance_laterale=0.06,
    tolerance_cap=math.radians(5.0), gain_lineaire=0.6, gain_angulaire=1.2,
    max_lineaire=0.08, max_angulaire=0.35, min_lineaire=0.03, min_angulaire=0.10,
    precision_distance=0.015, precision_cap=math.radians(2.0))


def test_un_cap_faux_donne_une_rotation_seule():
    commande = commande_alignement(
        ErreursVisite(0.2, 0.0, math.radians(10.0)), REGLAGES)
    assert commande.etat == 'en_cours'
    assert commande.lineaire == 0.0
    assert commande.angulaire > 0.0
    assert commande_alignement(
        ErreursVisite(0.0, 0.0, math.radians(-10.0)), REGLAGES).angulaire < 0.0


def test_la_rotation_respecte_les_vitesses_min_et_max():
    gain_faible = replace(REGLAGES, gain_angulaire=0.5)
    lente = commande_alignement(
        ErreursVisite(0.0, 0.0, math.radians(6.0)), gain_faible)
    assert lente.angulaire == pytest.approx(0.10)
    rapide = commande_alignement(ErreursVisite(0.0, 0.0, 1.5), REGLAGES)
    assert rapide.angulaire == pytest.approx(0.35)


def test_trop_loin_le_robot_avance_doucement():
    commande = commande_alignement(ErreursVisite(0.3, 0.0, 0.0), REGLAGES)
    assert commande.etat == 'en_cours'
    assert commande.lineaire == pytest.approx(0.08)


def test_trop_pres_le_robot_recule():
    commande = commande_alignement(ErreursVisite(-0.06, 0.0, 0.0), REGLAGES)
    assert commande.lineaire == pytest.approx(-0.036)


def test_un_grand_ecart_lateral_demande_une_nouvelle_navigation():
    commande = commande_alignement(ErreursVisite(0.0, 0.15, 0.0), REGLAGES)
    assert commande.etat == 'renaviguer'
    assert (commande.lineaire, commande.angulaire) == (0.0, 0.0)


def test_le_robot_s_arrete_seulement_a_l_objectif_de_precision():
    commande = commande_alignement(
        ErreursVisite(0.01, 0.05, math.radians(1.5)), REGLAGES)
    assert commande.etat == 'aligne'
    assert (commande.lineaire, commande.angulaire) == (0.0, 0.0)


def test_dans_la_tolerance_mais_hors_precision_le_robot_continue():
    """Mesuré en simulation : s'arrêter au bord de la tolérance (+4 cm) laissait
    le robot à 0,56 m du tag."""
    commande = commande_alignement(ErreursVisite(0.035, 0.0, 0.0), REGLAGES)
    assert commande.etat == 'en_cours'
    assert commande.lineaire == pytest.approx(0.03)


def test_distance_atteinte_le_cap_est_affine_a_vitesse_minimale():
    commande = commande_alignement(
        ErreursVisite(0.005, 0.0, math.radians(3.0)), REGLAGES)
    assert commande.etat == 'en_cours'
    assert commande.lineaire == 0.0
    assert commande.angulaire == pytest.approx(0.10)


def test_la_garde_avant_detecte_un_obstacle_dans_le_couloir():
    points = [(0.30, 0.05), (1.0, 0.0), (0.25, 0.6)]
    assert obstacle_devant(points, 0.22, 0.25, 0.10)
    assert not obstacle_devant([(0.45, 0.0), (0.2, 0.4)], 0.22, 0.25, 0.10)
    assert not obstacle_devant([(-0.1, 0.0)], 0.22, 0.25, 0.10)
