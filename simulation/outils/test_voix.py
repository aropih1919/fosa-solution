#!/usr/bin/env python3
"""Banc du mot de départ : rejoue des fichiers WAV comme le ReSpeaker et vérifie
que seul le mot de départ fait partir la course.

    python3 simulation/outils/generer_audios.py simulation/audios      # jeu d'essai (une fois)
    python3 simulation/outils/test_voix.py simulation/audios

Nom des fichiers : « oui_... .wav » doit donner le départ, « non_... .wav » ne
doit pas. Pour chaque fichier, un nœud mot_depart neuf est lancé (il ne donne
le départ qu'une fois), le fichier est publié en un seul segment sur
course.voice.audio_topic (audio_common_msgs/AudioData, PCM 16 bits mono à
course.voice.sample_rate, comme le nœud ReSpeaker des organisateurs), puis on
attend course.voice.start_topic (le mot reconnu) quelques secondes.

Il faut : faster-whisper et son modèle (hors ligne), audio_common_msgs, le
workspace sourcé. Écrit <dossier>/resultat_voix.json ; rend 0 si tout est juste.
"""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import wave

import numpy as np
import rclpy
from std_msgs.msg import String


def lire_wav(chemin, frequence_cible):
    with wave.open(str(chemin), 'rb') as fichier:
        canaux, largeur, frequence = (fichier.getnchannels(), fichier.getsampwidth(),
                                      fichier.getframerate())
        brut = fichier.readframes(fichier.getnframes())
    if largeur != 2:
        sys.exit(f'{chemin.name} : seul le PCM 16 bits est lu (arecord -f S16_LE).')
    signal_audio = np.frombuffer(brut, dtype='<i2').astype(np.float32)
    if canaux > 1:
        signal_audio = signal_audio.reshape(-1, canaux).mean(axis=1)
    if frequence != frequence_cible:
        duree = len(signal_audio) / frequence
        temps_cible = np.arange(int(duree * frequence_cible)) / frequence_cible
        signal_audio = np.interp(temps_cible, np.arange(len(signal_audio)) / frequence, signal_audio)
    return np.clip(signal_audio, -32768, 32767).astype('<i2').tobytes()


def lancer_mot_depart(journal):
    processus = subprocess.Popen(['ros2', 'run', 'caytu_assistant', 'mot_depart'],
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                 start_new_session=True)
    pret = threading.Event()
    lignes = []

    def lire():
        for ligne in processus.stdout:
            lignes.append(ligne.rstrip())
            journal.write(ligne)
            if 'Écoute des segments' in ligne:
                pret.set()
    threading.Thread(target=lire, daemon=True).start()
    return processus, pret, lignes


def arreter(processus):
    if processus.poll() is None:
        processus.send_signal(signal.SIGINT)
        try:
            processus.wait(timeout=8)
        except subprocess.TimeoutExpired:
            processus.kill()


def main():
    try:
        from caytu_config import load_config
        configuration = load_config().course
    except Exception as erreur:
        sys.exit(f'caytu_config introuvable (workspace sourcé ?) : {erreur}')
    from audio_common_msgs.msg import AudioData

    lecteur = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    lecteur.add_argument('dossier', type=Path, help='dossier des fichiers oui_*.wav / non_*.wav')
    lecteur.add_argument('--attente', type=float, default=8.0, help='secondes d\'attente du départ')
    args, reste = lecteur.parse_known_args()
    fichiers = sorted(args.dossier.glob('oui_*.wav')) + sorted(args.dossier.glob('non_*.wav'))
    if not fichiers:
        sys.exit(f'Aucun fichier oui_*.wav ou non_*.wav dans {args.dossier}.')

    rclpy.init(args=reste)
    noeud = rclpy.create_node('test_voix')
    pub_audio = noeud.create_publisher(AudioData, configuration.voice.audio_topic, 10)
    departs = []
    noeud.create_subscription(String, configuration.voice.start_topic,
                              lambda _: departs.append(time.monotonic()), 10)
    resultats = []
    with open(args.dossier / 'journal_mot_depart.log', 'w', encoding='utf-8') as journal:
        for chemin in fichiers:
            attendu = chemin.name.startswith('oui_')
            processus, pret, lignes = lancer_mot_depart(journal)
            debut = time.monotonic()
            while not pret.is_set() and time.monotonic() - debut < 120 and processus.poll() is None:
                rclpy.spin_once(noeud, timeout_sec=0.2)
            if not pret.is_set():
                arreter(processus)
                print('\n'.join(lignes[-15:]))
                sys.exit('mot_depart ne démarre pas (faster-whisper, modèle, audio_common_msgs ?).')
            time.sleep(0.5)
            departs.clear()
            pub_audio.publish(AudioData(data=lire_wav(chemin, configuration.voice.sample_rate)))
            fin = time.monotonic() + args.attente
            while time.monotonic() < fin and not departs:
                rclpy.spin_once(noeud, timeout_sec=0.1)
            obtenu = bool(departs)
            entendu = next((l.split('Entendu : ', 1)[1] for l in reversed(lignes)
                            if 'Entendu : ' in l), '(segment ignoré : durée ou volume)')
            juste = obtenu == attendu
            resultats.append({'fichier': chemin.name, 'attendu': attendu, 'depart': obtenu,
                              'entendu': entendu, 'juste': juste})
            print(f'{"OK    " if juste else "ERREUR"} {chemin.name:32} départ attendu : '
                  f'{"oui" if attendu else "non"}, obtenu : {"oui" if obtenu else "non"} ; '
                  f'entendu {entendu}', flush=True)
            arreter(processus)
    erreurs = [r for r in resultats if not r['juste']]
    print(f'RESULTAT : {len(resultats) - len(erreurs)}/{len(resultats)} justes'
          + (f' ; erreurs : {[r["fichier"] for r in erreurs]}' if erreurs else ''))
    (args.dossier / 'resultat_voix.json').write_text(
        json.dumps(resultats, ensure_ascii=False, indent=2), encoding='utf-8')
    noeud.destroy_node()
    rclpy.try_shutdown()
    sys.exit(1 if erreurs else 0)


if __name__ == '__main__':
    main()
