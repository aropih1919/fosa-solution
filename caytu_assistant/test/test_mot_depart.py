"""Démarrage vocal : seul un mot de départ court déclenche la course."""

import numpy as np

from caytu_assistant.mot_depart import (
    est_mot_depart, niveau_rms, pcm16_vers_float, trouver_mot_depart)

MOTS = ('allez', 'go', 'vai')


def test_only_short_utterances_with_the_start_word_count():
    for texte in ('Allez !', 'allez', 'Allez-y', ' ALLEZ.', 'go', 'Go!', 'vai', 'allez allez'):
        assert est_mot_depart(texte, MOTS, 3), texte
    for texte in ('', 'bonjour', 'on y va', 'il faut aller manger maintenant',
                  'gogo', 'allons-y', 'ils vont aller au stade ce soir'):
        assert not est_mot_depart(texte, MOTS, 3), texte


def test_a_single_configured_word_is_enough():
    assert est_mot_depart('Allez', ('allez',), 3)
    assert not est_mot_depart('go', ('allez',), 3)


def test_pcm16_is_converted_and_resampled_to_16_khz():
    signal_48k = (np.sin(np.linspace(0, 100, 48000)) * 16000).astype('<i2')
    converti = pcm16_vers_float(signal_48k.tobytes(), 48000)
    assert abs(len(converti) - 16000) <= 1
    assert converti.dtype == np.float32
    assert 0.3 < niveau_rms(converti) < 0.4
    assert niveau_rms(np.zeros(10, np.float32)) == 0.0


def test_le_mot_officiel_est_rendu_pour_l_arbitre():
    assert trouver_mot_depart('Allez !', ('allez', 'go', 'vai'), 3) == 'allez'
    assert trouver_mot_depart('Aller.', ('allez', 'go', 'vai'), 3) == 'allez'
    assert trouver_mot_depart('Go!', ('allez', 'go', 'vai'), 3) == 'go'
    assert trouver_mot_depart('bonjour', ('allez', 'go', 'vai'), 3) is None
    assert trouver_mot_depart('allez on y va maintenant', ('allez',), 3) is None
