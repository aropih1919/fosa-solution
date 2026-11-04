"""Stratégie de visite : choix des tags à tenter selon le temps restant."""

import pytest

from caytu_course.strategie import (
    ModeleStrategie,
    TagRestant,
    decider,
    duree_tag,
    planifier,
    probabilite,
)

# Détour 1 et marge nulle : les durées se calculent de tête.
MODELE = ModeleStrategie(
    points_par_tag=25.0, vitesse_mps=0.2, facteur_detour=1.0,
    temps_visite_s=30.0, temps_recherche_s=90.0, trajet_inconnu_m=2.0,
    marge=0.0, reussite_connu=0.9, reussite_inconnu=0.6, baisse_par_echec=0.5,
    gain_min_saut=5.0)
DEPART = (0.0, 0.0)
NORD, PRES = (0.0, 4.0), (0.0, 0.4)
# Le 7 loin au nord-ouest, les trois autres près du départ.
DISPERSES = (TagRestant(7, (-2.0, 4.5)), TagRestant(1, (0.0, 0.4)),
             TagRestant(9, (0.5, 1.5)), TagRestant(4, (0.3, 0.0)))


def test_un_tag_connu_coute_le_trajet_plus_la_visite():
    assert duree_tag(DEPART, TagRestant(7, NORD), MODELE) == (pytest.approx(50.0), NORD)
    assert duree_tag(None, TagRestant(7, NORD), MODELE)[0] == pytest.approx(40.0)
    assert duree_tag(DEPART, TagRestant(7), MODELE) == (pytest.approx(120.0), None)


def test_avec_assez_de_temps_tous_les_tags_sont_gardes_dans_l_ordre():
    garder, plan = decider(DEPART, DISPERSES, 190.0, MODELE)
    assert garder
    assert plan.tags == (7, 1, 9, 4)


def test_un_premier_tag_lointain_est_saute_s_il_coute_des_tags():
    """120 s restantes : les quatre demandent 181 s ; tout plan à trois tags
    avec le 7 dépasse 139 s ; 1-9-4 tient en 106 s. Garder le 7 = 2 tags
    (50 points), le sauter = 3 tags (75 points)."""
    garder, plan = decider(DEPART, DISPERSES, 120.0, MODELE)
    assert not garder
    assert plan.tags == (1, 9, 4)


def test_a_valeur_egale_la_cible_est_gardee():
    """Régression de la simulation : au départ, aucun tag connu, tous les
    plans à trois tags se valent ; le tag 7 était sauté au bout de 2 s."""
    reel = ModeleStrategie(**{**MODELE.__dict__, 'temps_visite_s': 45.0, 'marge': 0.2})
    debut = (TagRestant(7, temps_passe_s=2.0),) + tuple(
        TagRestant(tag) for tag in (1, 9, 4))
    assert len(planifier(DEPART, debut, 593.0, reel).tags) == 3
    garder, plan = decider(DEPART, debut, 593.0, reel)
    assert garder
    assert plan.tags[0] == 7


def test_un_tag_introuvable_est_saute_quand_les_suivants_ne_tiennent_plus():
    """Le 7 n'a jamais été vu ; 1, 9, 4 ont été aperçus pendant la recherche."""
    restants = (TagRestant(7, temps_passe_s=120.0), TagRestant(1, PRES),
                TagRestant(9, (1.0, 1.0)), TagRestant(4, PRES))
    assert decider(DEPART, restants, 400.0, MODELE) == (True, planifier(
        DEPART, restants, 400.0, MODELE))
    garder, plan = decider(DEPART, restants, 200.0, MODELE)
    assert not garder
    assert plan.tags == (1, 9, 4)


def test_les_echecs_font_preferer_un_tag_suivant_plus_sur():
    """Une seule visite tient encore : le 7 a échoué deux fois, le 1 est sûr."""
    restants = (TagRestant(7, PRES, echecs=2), TagRestant(1, PRES))
    garder, plan = decider(DEPART, restants, 40.0, MODELE)
    assert not garder
    assert plan.tags == (1,)
    sans_echec = (TagRestant(7, PRES), TagRestant(1, PRES))
    assert decider(DEPART, sans_echec, 40.0, MODELE)[0]


def test_chercher_au_dela_du_temps_prevu_baisse_la_probabilite():
    assert probabilite(TagRestant(7), MODELE) == pytest.approx(0.6)
    assert probabilite(TagRestant(7, temps_passe_s=60.0), MODELE) == pytest.approx(0.6)
    assert probabilite(TagRestant(7, temps_passe_s=180.0), MODELE) == pytest.approx(0.3)


def test_la_marge_rend_le_plan_prudent():
    restants = (TagRestant(7, PRES),)
    assert planifier(DEPART, restants, 33.0, MODELE).tags == (7,)
    prudent = ModeleStrategie(**{**MODELE.__dict__, 'marge': 0.2})
    assert planifier(DEPART, restants, 33.0, prudent).tags == ()


def test_sans_plan_possible_on_continue_la_cible():
    garder, plan = decider(DEPART, (TagRestant(7), TagRestant(1)), 10.0, MODELE)
    assert garder
    assert plan.tags == ()
