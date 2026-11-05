"""Commande d'alignement final devant un tag.

Les situations sont décrites par la pose du tag dans le repère du robot
(x devant, y à gauche) ; cap_normale = pi : tag face au robot.
"""

from dataclasses import replace
import math

import pytest

from caytu_course.alignement import (
    CommandeAlignement,
    ReglagesAlignement,
    commande_alignement,
    obstacle_derriere,
    obstacle_devant,
    obstacle_pivot,
    scan_couvre_tout_autour,
    vitesse_sure,
)
from caytu_course.geometrie_tags import erreurs_visite

REGLAGES = ReglagesAlignement(
    tolerance_cap=math.radians(5.0), gain_lineaire=0.6, gain_angulaire=1.2,
    max_lineaire=0.08, max_angulaire=0.35, min_lineaire=0.03, min_angulaire=0.10,
    precision_point=0.025, precision_cap=math.radians(2.0),
    erreur_distance_max=0.75, erreur_laterale_max=0.30)


def _erreurs(x_tag, y_tag, cap_normale=math.pi):
    return erreurs_visite(x_tag, y_tag, cap_normale, 0.0, 0.5)


def _commande(x_tag, y_tag, cap_normale=math.pi, reglages=REGLAGES, au_point=False):
    return commande_alignement(_erreurs(x_tag, y_tag, cap_normale), reglages, au_point)


def test_depuis_la_pre_approche_le_robot_avance_vers_le_point():
    """Arrivée de Nav2 à 0,9 m, bien dans l'axe : rouler, plus de renavigation."""
    commande = _commande(0.9, 0.0)
    assert commande.etat == 'en_cours'
    assert commande.lineaire == pytest.approx(0.08)
    assert commande.angulaire == pytest.approx(0.0)


def test_un_ecart_lateral_se_corrige_en_roulant():
    """Régression de la simulation : 8 cm d'écart à 0,9 m relançait Nav2 en boucle."""
    tourne = _commande(0.9, 0.08)
    assert tourne.etat == 'en_cours'
    assert tourne.lineaire == 0.0
    assert tourne.angulaire > 0.0
    # Une fois tourné vers le point (11°), le robot roule en corrigeant le cap.
    angle = math.atan2(0.08, 0.4)
    x = 0.9 * math.cos(angle) + 0.08 * math.sin(angle)
    y = -0.9 * math.sin(angle) + 0.08 * math.cos(angle)
    roule = _commande(x, y, math.pi - angle)
    assert roule.lineaire > 0.0
    assert roule.etat == 'en_cours'


def test_trop_pres_le_robot_recule_vers_le_point():
    commande = _commande(0.4, 0.0)
    assert commande.etat == 'en_cours'
    assert commande.lineaire == pytest.approx(-0.06)
    assert commande.angulaire == pytest.approx(0.0)


def test_en_marche_arriere_le_cap_tourne_l_arriere_vers_le_point():
    """Point derrière à gauche : l'arrière doit pivoter vers la gauche, donc
    le robot tourne à droite (vitesse angulaire négative)."""
    commande = _commande(0.4, 0.02, reglages=replace(REGLAGES, tolerance_cap=0.5))
    assert commande.lineaire < 0.0
    assert commande.angulaire < 0.0


def test_au_point_le_robot_tourne_face_au_tag():
    angle = math.radians(10.0)
    x, y = 0.5 * math.cos(-angle), 0.5 * math.sin(-angle)
    commande = _commande(x, y, math.pi - angle)
    assert commande.etat == 'en_cours'
    assert commande.au_point
    assert commande.lineaire == 0.0
    assert commande.angulaire < 0.0


def test_la_rotation_respecte_les_vitesses_min_et_max():
    angle = math.radians(3.0)
    x, y = 0.5 * math.cos(angle), 0.5 * math.sin(angle)
    lente = _commande(x, y, math.pi + angle, replace(REGLAGES, gain_angulaire=0.5))
    assert lente.angulaire == pytest.approx(0.10)
    angle = 1.2
    x, y = 0.5 * math.cos(angle), 0.5 * math.sin(angle)
    assert _commande(x, y, math.pi + angle).angulaire == pytest.approx(0.35)


def test_aligne_au_point_et_face_au_tag():
    commande = _commande(0.51, 0.01)
    assert commande.etat == 'aligne'
    assert (commande.lineaire, commande.angulaire) == (0.0, 0.0)


def test_hors_precision_le_robot_continue_meme_dans_la_tolerance():
    """Mesuré en simulation : s'arrêter au bord de la tolérance (+4 cm) laissait
    le robot à 0,56 m du tag."""
    commande = _commande(0.535, 0.0)
    assert commande.etat == 'en_cours'
    assert commande.lineaire == pytest.approx(0.03)


def test_l_hysteresis_evite_de_repartir_pendant_la_rotation_finale():
    assert _commande(0.54, 0.0).etat == 'en_cours'
    assert _commande(0.54, 0.0, au_point=True).etat == 'aligne'


def test_un_tag_vu_tres_de_biais_redemande_nav2():
    commande = _commande(0.9, 0.35)
    assert commande.etat == 'renaviguer'
    assert (commande.lineaire, commande.angulaire) == (0.0, 0.0)


def test_trop_loin_pour_l_asservissement_visuel_on_renavigue():
    """Revue : sans borne, le robot pouvait rouler 1,6 m sans planification."""
    assert _commande(1.3, 0.0).etat == 'renaviguer'
    assert _commande(1.2, 0.0).etat == 'en_cours'


def test_la_garde_avant_detecte_un_obstacle_dans_le_couloir():
    points = [(0.30, 0.05), (1.0, 0.0), (0.25, 0.6)]
    assert obstacle_devant(points, 0.22, 0.25, 0.10)
    assert not obstacle_devant([(0.45, 0.0), (0.2, 0.4)], 0.22, 0.25, 0.10)
    assert not obstacle_devant([(-0.1, 0.0)], 0.22, 0.25, 0.10)


def test_la_garde_arriere_protege_le_recul():
    assert obstacle_derriere([(-0.38, 0.1)], -0.31, 0.25, 0.10)
    assert not obstacle_derriere([(-0.5, 0.0), (-0.35, 0.4), (0.3, 0.0)], -0.31, 0.25, 0.10)


def test_la_garde_de_pivot_verifie_le_disque_balaye():
    assert obstacle_pivot([(0.2, 0.1)], 0.25)
    assert not obstacle_pivot([(0.3, 0.2)], 0.25)


def test_une_commande_est_bloquee_si_le_scan_est_inconnu_ou_incomplet():
    commande = _commande(0.9, 0.0)
    empreinte = ((0.22, 0.25), (0.22, -0.25), (-0.31, -0.25), (-0.31, 0.25))

    assert vitesse_sure(
        commande, None, empreinte, 0.22, -0.31, 0.25, 0.10, True) == (0.0, 0.0)
    assert vitesse_sure(
        commande, [], empreinte, 0.22, -0.31, 0.25, 0.10, False) == (0.0, 0.0)


def test_un_obstacle_dans_le_couloir_bloque_toute_la_commande():
    commande = _commande(0.9, 0.0)
    empreinte = ((0.22, 0.25), (0.22, -0.25), (-0.31, -0.25), (-0.31, 0.25))

    assert vitesse_sure(
        commande, [(0.30, 0.0)], empreinte, 0.22, -0.31, 0.25,
        0.10, True) == (0.0, 0.0)


def test_un_pivot_est_bloque_si_le_disque_balaye_est_occupe():
    commande = CommandeAlignement(0.0, 0.1, 'en_cours')
    empreinte = ((0.22, 0.25), (0.22, -0.25), (-0.31, -0.25), (-0.31, 0.25))

    assert vitesse_sure(
        commande, [(0.3, 0.0)], empreinte, 0.22, -0.31, 0.25,
        0.10, True) == (0.0, 0.0)
    assert vitesse_sure(
        commande, [(0.8, 0.0)], empreinte, 0.22, -0.31, 0.25,
        0.10, True) == (0.0, 0.1)


def test_la_couverture_complete_exige_360_degres_et_aucun_secteur_invalide():
    assert scan_couvre_tout_autour(-math.pi, math.pi, 0.1, [1.0] * 63, 0.1, 5.0)
    assert not scan_couvre_tout_autour(-math.pi / 2, math.pi / 2, 0.1,
                                       [1.0] * 32, 0.1, 5.0)
    assert not scan_couvre_tout_autour(-math.pi, math.pi, 0.1,
                                       [1.0, math.inf], 0.1, 5.0)


def test_le_recul_reste_interdit_sauf_activation_explicite_et_garde():
    commande = _commande(0.4, 0.0)
    empreinte = ((0.22, 0.25), (0.22, -0.25), (-0.31, -0.25), (-0.31, 0.25))

    assert vitesse_sure(
        commande, [(0.8, 0.0)], empreinte, 0.22, -0.31, 0.25,
        0.10, True) == (0.0, 0.0)
    assert vitesse_sure(
        commande, [(-0.42, 0.0)], empreinte, 0.22, -0.31, 0.25,
        0.10, True, recul_autorise=True) == (0.0, 0.0)
    assert vitesse_sure(
        commande, [(0.8, 0.0)], empreinte, 0.22, -0.31, 0.25,
        0.10, True, recul_autorise=True)[0] < 0.0
