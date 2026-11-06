"""Décision « mot de départ » (sans ROS ni Vosk) : robuste au bruit du public.

Règlement : un départ provoqué par le bruit fait recommencer la course et
perdre le bonus. On n'accepte donc qu'un énoncé composé uniquement du mot de
départ, reconnu avec une confiance suffisante pour chaque mot.
"""

import json

INCONNU = '[unk]'


def grammaire(mots):
    """Vocabulaire Vosk restreint : les mots de départ et « inconnu »."""
    return json.dumps(list(mots) + [INCONNU])


def est_mot_depart(resultat_json, mots, confiance_min):
    try:
        resultat = json.loads(resultat_json)
    except (TypeError, ValueError):
        return False
    texte = str(resultat.get('text', '')).strip().casefold()
    details = resultat.get('result')
    if not texte or not isinstance(details, list) or not details:
        return False
    mots = {mot.casefold() for mot in mots}
    if any(mot not in mots for mot in texte.split()):
        return False
    return all(
        str(detail.get('word', '')).casefold() in mots
        and float(detail.get('conf', 0.0)) >= confiance_min
        for detail in details)


def extraire_canal(donnees, canaux, largeur, index):
    """Garde un canal d'un flux PCM entrelacé (trames incomplètes ignorées)."""
    if canaux <= 1:
        return bytes(donnees)
    trame = canaux * largeur
    utile = len(donnees) - len(donnees) % trame
    debut = index * largeur
    return b''.join(
        bytes(donnees[position + debut:position + debut + largeur])
        for position in range(0, utile, trame))
