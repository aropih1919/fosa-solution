"""Décision « mot de départ » à partir des résultats Vosk et du flux audio."""

import json
import struct

import pytest

from caytu_course.mot_depart import est_mot_depart, extraire_canal, grammaire

MOTS = ('allez',)


def _resultat(texte, *confiances):
    mots = texte.split()
    return json.dumps({
        'text': texte,
        'result': [{'word': mot, 'conf': conf} for mot, conf in zip(mots, confiances)],
    })


def test_le_mot_seul_et_sur_declenche_le_depart():
    assert est_mot_depart(_resultat('allez', 0.97), MOTS, 0.8)
    assert est_mot_depart(_resultat('Allez', 0.97), MOTS, 0.8)


@pytest.mark.parametrize('resultat', [
    _resultat('allez', 0.5),                 # confiance insuffisante
    _resultat('[unk]', 1.0),                 # bruit
    _resultat('[unk] allez', 1.0, 0.99),     # phrase du public contenant le mot
    _resultat('go', 0.99),                   # autre mot de départ non configuré
    json.dumps({'text': ''}),                # rien reconnu
    json.dumps({'text': 'allez'}),           # pas de confiance par mot
    'pas du json',
])
def test_le_bruit_et_les_phrases_ne_declenchent_rien(resultat):
    assert not est_mot_depart(resultat, MOTS, 0.8)


def test_la_grammaire_limite_le_vocabulaire():
    assert json.loads(grammaire(MOTS)) == ['allez', '[unk]']


def test_extraire_un_canal_d_un_flux_entrelace():
    echantillons = [10, -10, 20, -20, 30, -30]           # 3 trames stéréo
    donnees = struct.pack('<6h', *echantillons)
    assert struct.unpack('<3h', extraire_canal(donnees, 2, 2, 0)) == (10, 20, 30)
    assert struct.unpack('<3h', extraire_canal(donnees, 2, 2, 1)) == (-10, -20, -30)
    assert extraire_canal(donnees, 1, 2, 0) == donnees


def test_une_trame_incomplete_est_ignoree():
    donnees = struct.pack('<5h', 1, 2, 3, 4, 5)
    assert struct.unpack('<2h', extraire_canal(donnees, 2, 2, 0)) == (1, 3)
