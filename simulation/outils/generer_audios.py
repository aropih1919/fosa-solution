#!/usr/bin/env python3
"""Jeu d'essai du mot de départ, pour simulation/outils/test_voix.py.

    python3 simulation/outils/generer_audios.py simulation/audios

Écrit des WAV 16 kHz, 16 bits, mono :
  - bruits (toujours) : non_silence, non_bruit_blanc, non_applaudissements ;
  - voix de synthèse, si espeak-ng est installé (sudo apt install espeak-ng) :
    oui_allez, oui_go, oui_vai, non_bonjour, non_on_y_va, non_arrete,
    non_phrase_longue (« allez on y va maintenant les amis » : trop de mots).

La voix de synthèse est un premier contrôle : Whisper la comprend moins bien
qu'une vraie voix. Le jeu de référence, ce sont vos voix, enregistrées au
micro avec le même nommage, par exemple :
    arecord -f S16_LE -r 16000 -c 1 -d 2 simulation/audios/oui_allez_hasina.wav
    arecord -f S16_LE -r 16000 -c 1 -d 2 simulation/audios/non_bonjour_hasina.wav
"""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import wave

import numpy as np

FREQUENCE = 16000
PHRASES = [
    ('oui_allez', 'fr', 'allez'),
    ('oui_go', 'en', 'go'),
    ('oui_vai', 'pt', 'vai'),
    ('non_bonjour', 'fr', 'bonjour'),
    ('non_on_y_va', 'fr', 'on y va'),
    ('non_arrete', 'fr', 'arrête'),
    ('non_phrase_longue', 'fr', 'allez on y va maintenant les amis'),
]


def ecrire(chemin, signal_audio):
    donnees = np.clip(signal_audio * 32767, -32768, 32767).astype('<i2')
    with wave.open(str(chemin), 'wb') as fichier:
        fichier.setnchannels(1)
        fichier.setsampwidth(2)
        fichier.setframerate(FREQUENCE)
        fichier.writeframes(donnees.tobytes())


def main():
    dossier = Path(sys.argv[1] if len(sys.argv) > 1 else 'simulation/audios')
    dossier.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(0)
    ecrire(dossier / 'non_silence.wav', rng.normal(0, 0.002, FREQUENCE))
    ecrire(dossier / 'non_bruit_blanc.wav', rng.normal(0, 0.2, FREQUENCE))
    applaudissements = np.zeros(int(1.5 * FREQUENCE))
    for debut in np.arange(0.05, 1.4, 0.18):
        i = int(debut * FREQUENCE)
        applaudissements[i:i + 600] = rng.normal(0, 0.6, 600) * np.exp(-np.arange(600) / 120)
    ecrire(dossier / 'non_applaudissements.wav', applaudissements)
    print(f'3 bruits écrits dans {dossier}.')

    espeak = shutil.which('espeak-ng') or shutil.which('espeak')
    if not espeak:
        print('espeak-ng absent : pas de voix de synthèse (sudo apt install espeak-ng), '
              'ou enregistrez vos voix avec arecord (voir --help).')
        return
    with tempfile.TemporaryDirectory() as temporaire:
        for nom, langue, texte in PHRASES:
            brut = Path(temporaire) / f'{nom}.wav'
            subprocess.run([espeak, '-v', langue, '-s', '140', '-w', str(brut), texte],
                           check=True, capture_output=True)
            with wave.open(str(brut), 'rb') as fichier:
                frequence = fichier.getframerate()
                signal_audio = np.frombuffer(fichier.readframes(fichier.getnframes()),
                                             dtype='<i2').astype(np.float32) / 32768.0
            temps = np.arange(int(len(signal_audio) / frequence * FREQUENCE)) / FREQUENCE
            signal_audio = np.interp(temps, np.arange(len(signal_audio)) / frequence, signal_audio)
            # Un peu de silence autour, comme un segment découpé par le ReSpeaker.
            marge = np.zeros(int(0.2 * FREQUENCE))
            ecrire(dossier / f'{nom}.wav', np.concatenate([marge, signal_audio, marge]))
    print(f'{len(PHRASES)} voix de synthèse écrites ({Path(espeak).name}).')


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] in ('-h', '--help'):
        print(__doc__)
        sys.exit(0)
    main()
