"""Ordre publié par les organisateurs et séquence de visite."""

import pytest

from caytu_course.ordre_tags import OrdreInvalide, lire_ordre
from caytu_course.sequence import SequenceTags

TAGS = (1, 4, 7, 9)


@pytest.mark.parametrize('valeur', [
    '7,1,9,4', '[7, 1, 9, 4]', '7 1 9 4', '{"order": [7, 1, 9, 4]}',
    '{"ordre": [7, 1, 9, 4]}', 'ordre : 7 - 1 - 9 - 4', [7, 1, 9, 4], (7, 1, 9, 4),
])
def test_l_ordre_est_lu_sous_plusieurs_formes(valeur):
    assert lire_ordre(valeur, TAGS) == (7, 1, 9, 4)


@pytest.mark.parametrize('valeur', [
    '7,1,9', '7,1,9,9', '7,1,9,5', '', 'aucun', [7, 1, 9, 4, 1], '{"order": []}',
])
def test_un_ordre_incomplet_ou_inconnu_est_refuse(valeur):
    with pytest.raises(OrdreInvalide):
        lire_ordre(valeur, TAGS)


def test_la_sequence_suit_l_ordre_et_n_y_revient_jamais():
    sequence = SequenceTags((7, 1, 9, 4))
    assert sequence.cible() == 7
    assert sequence.restants() == (7, 1, 9, 4)
    sequence.marquer_visite(7)
    assert sequence.cible() == 1
    sequence.sauter()
    assert sequence.sautes == [1]
    assert sequence.cible() == 9
    assert not sequence.est_attendu(1)
    assert sequence.est_attendu(9)
    sequence.marquer_visite(9)
    sequence.marquer_visite(4)
    assert sequence.terminee()
    assert sequence.cible() is None
    assert sequence.visites == [7, 9, 4]


def test_une_visite_hors_ordre_est_refusee():
    sequence = SequenceTags((7, 1, 9, 4))
    with pytest.raises(ValueError):
        sequence.marquer_visite(9)


def test_sauter_une_sequence_terminee_est_refuse():
    sequence = SequenceTags((7,))
    sequence.marquer_visite(7)
    with pytest.raises(ValueError):
        sequence.sauter()

