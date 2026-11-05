"""Validation des horodatages utilisés pour les obstacles."""

from caytu_perception.fraicheur_capteurs import horodatage_recent


def test_un_horodatage_dans_la_fenetre_est_recent():
    assert horodatage_recent(9_700_000_000, 10_000_000_000, 0.5)


def test_un_horodatage_perime_est_refuse():
    assert not horodatage_recent(9_000_000_000, 10_000_000_000, 0.5)


def test_un_horodatage_nul_futur_ou_sans_horloge_est_refuse():
    assert not horodatage_recent(0, 10_000_000_000, 0.5)
    assert not horodatage_recent(10_100_000_000, 10_000_000_000, 0.5)
    assert not horodatage_recent(9_900_000_000, 0, 0.5)
