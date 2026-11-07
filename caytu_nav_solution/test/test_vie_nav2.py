"""Nœuds lifecycle exigés avant d'envoyer un but Nav2."""

from caytu_nav_solution.vie_nav2 import (
    ETAT_ACTIF,
    NOEUDS_GERES,
    etats_inactifs,
    noeuds_attendus,
)


def test_le_mode_carte_exige_slam_et_nav2():
    assert noeuds_attendus('carte') == NOEUDS_GERES
    assert 'slam_toolbox' in noeuds_attendus('carte')


def test_le_mode_sans_carte_ignore_slam():
    attendus = noeuds_attendus('sans_carte')
    assert 'slam_toolbox' not in attendus
    assert 'controller_server' in attendus
    assert 'bt_navigator' in attendus


def test_tous_actifs_ne_laisse_aucun_inactif():
    etats = {nom: ETAT_ACTIF for nom in NOEUDS_GERES}
    assert etats_inactifs(etats, 'carte') == ()


def test_un_noeud_inactif_ou_absent_est_signale():
    etats = {nom: ETAT_ACTIF for nom in NOEUDS_GERES}
    etats['planner_server'] = 1
    assert etats_inactifs(etats, 'carte') == ('planner_server',)

    del etats['bt_navigator']
    assert etats_inactifs(etats, 'carte') == ('planner_server', 'bt_navigator')


def test_slam_inactif_ne_bloque_pas_le_mode_sans_carte():
    etats = {nom: ETAT_ACTIF for nom in NOEUDS_GERES}
    etats['slam_toolbox'] = 1
    assert etats_inactifs(etats, 'sans_carte') == ()
