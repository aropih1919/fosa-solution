"""Surveillance technique : événements Nav2, rotations, arrêts, blocage."""

import math

from caytu_course.surveillance import Surveillance


def test_les_statuts_nav2_sont_signales_une_fois():
    surveillance = Surveillance()
    assert surveillance.statut_nav2(1, 0.0) == ['[NAV2] trajet en cours']
    assert surveillance.statut_nav2(1, 1.0) == []
    assert surveillance.statut_nav2(2, 1.5) == []
    assert surveillance.nav2_actif
    assert surveillance.statut_nav2(6, 2.0) == ["[NAV2] ÉCHEC : Nav2 a abandonné le trajet"]
    assert not surveillance.nav2_actif


def test_seuls_le_premier_chemin_et_les_detours_sont_affiches():
    """Régression : 244 lignes [CHEMIN] en 2 minutes (replanification à 2 Hz)."""
    surveillance = Surveillance()
    surveillance.statut_nav2(1, 0.0)
    assert surveillance.chemin(40, 2.31, 0.0) == ['[CHEMIN] trajet planifié : 2,31 m']
    assert surveillance.chemin(38, 2.10, 0.5) == []
    assert surveillance.chemin(36, 1.90, 1.0) == []
    assert surveillance.chemin(50, 2.70, 1.5)[0].startswith(
        '[CHEMIN] détour : chemin rallongé de 0,80 m (2,70 m)')
    assert surveillance.chemin(0, 0.0, 2.0) == ['[CHEMIN] aucun chemin trouvé vers le but']
    assert surveillance.replanifications == 4
    surveillance.statut_nav2(4, 3.0)
    surveillance.statut_nav2(1, 4.0)
    assert surveillance.chemin(20, 1.0, 4.0) == ['[CHEMIN] trajet planifié : 1,00 m']


def test_une_rotation_sur_place_est_resumee_a_sa_fin():
    surveillance = Surveillance()
    assert surveillance.commande(0.0, 0.4, 0.0) == []
    assert surveillance.commande(0.0, 0.4, 3.0) == []
    assert surveillance.commande(0.2, 0.0, 3.2) == ['[ROTATION] rotation sur place de 3 s']


def test_un_arret_prolonge_pendant_un_trajet_est_signale():
    surveillance = Surveillance()
    surveillance.statut_nav2(1, 0.0)
    assert surveillance.commande(0.0, 0.0, 0.0) == []
    assert surveillance.commande(0.0, 0.0, 6.5) == [
        '[ARRÊT] immobile depuis 6 s pendant un trajet Nav2']


def test_des_commandes_sans_mouvement_signalent_un_blocage():
    """Cas de la simulation : robot coincé entre un tag et un meuble."""
    surveillance = Surveillance()
    surveillance.commande(-0.08, 0.0, 0.0)
    assert surveillance.mouvement(1.0, 1.0, 0.0, 0.0) == []
    assert surveillance.mouvement(1.0, 1.01, 0.0, 5.5)[0].startswith('[BLOQUÉ]')
    assert surveillance.mouvement(1.0, 1.01, 0.0, 7.0) == []


def test_une_rotation_qui_change_de_sens_sans_progresser_est_une_oscillation():
    """Régression du tag 9 : gauche, droite, gauche... pendant 4 minutes."""
    surveillance = Surveillance()
    surveillance.commande(0.0, 0.35, 0.0)
    surveillance.mouvement(1.0, 1.0, 0.0, 0.0)
    surveillance.commande(0.0, -0.35, 1.0)
    surveillance.commande(0.0, 0.35, 2.0)
    surveillance.commande(0.0, -0.35, 3.0)
    lignes = surveillance.mouvement(1.0, 1.0, 0.02, 5.5)
    assert lignes[0].startswith('[OSCILLATION]')
    assert '3 changements de sens' in lignes[0]


def test_un_robot_qui_avance_n_est_pas_bloque():
    surveillance = Surveillance()
    surveillance.commande(0.2, 0.0, 0.0)
    surveillance.mouvement(0.0, 0.0, 0.0, 0.0)
    assert surveillance.mouvement(1.0, 0.0, 0.0, 5.5) == []
    surveillance.commande(0.0, 0.3, 6.0)
    surveillance.mouvement(1.0, 0.0, 0.0, 6.0)
    assert surveillance.mouvement(1.0, 0.0, math.radians(30), 11.5) == []


def test_la_ligne_d_etat():
    surveillance = Surveillance()
    surveillance.contact({'table', 'sitoe_robot'})
    ligne = surveillance.ligne_etat((1.234, -0.5, math.radians(90)))
    assert ligne == ('[ÉTAT] position (1,23 ; -0,50), cap +90° | v=+0.00 w=+0.00 | '
                     'trajet Nav2 inactif | chemins 0 | contacts 1')
