"""Politique de conservation de la même cible avant décision de saut."""

from caytu_course.politique_blocage import doit_conserver_cible


def test_la_strategie_peut_garder_la_cible_sans_echec():
    assert doit_conserver_cible(True, 0, 2)


def test_une_cible_echouee_est_reprise_avant_d_etre_sautable():
    assert doit_conserver_cible(False, 1, 2)
    assert doit_conserver_cible(False, 2, 2)
    assert not doit_conserver_cible(False, 3, 2)


def test_aucune_reprise_configuree_n_impose_pas_de_tentative():
    assert not doit_conserver_cible(False, 1, 0)
