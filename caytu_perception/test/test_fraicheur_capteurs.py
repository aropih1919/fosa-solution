"""Régression issue de la branche integration/phase-reelle-securisee."""

import pytest

from caytu_perception.fraicheur_capteurs import horodatage_recent


@pytest.mark.parametrize('stamp', [0, 8_000_000_000, 11_000_000_000])
def test_horodatage_inutilisable(stamp):
    assert not horodatage_recent(stamp, 10_000_000_000, 0.6)


def test_mesure_fraiche():
    assert horodatage_recent(9_500_000_000, 10_000_000_000, 0.6)
