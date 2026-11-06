#!/usr/bin/env python3
"""Lance et arrête uniquement les groupes de processus créés par cette solution.

Un jeton hérité identifie les descendants, même si le lanceur est déjà mort.
Le fichier d'état ne suffit jamais, à lui seul, pour envoyer un signal à un PID.
"""

import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid


def membres(etat):
    if not etat:
        return []
    marque = f'FOSA_SESSION_ID={etat["jeton"]}'.encode()
    resultat = []
    for dossier in Path('/proc').iterdir():
        if not dossier.name.isdigit():
            continue
        try:
            if (marque in (dossier / 'environ').read_bytes().split(b'\0')
                    and os.getpgid(int(dossier.name)) == etat['pgid']):
                resultat.append(int(dossier.name))
        except (OSError, ProcessLookupError):
            pass
    return resultat


def lire_etat(chemin):
    if not chemin.exists():
        return None
    return json.loads(chemin.read_text(encoding='utf-8'))


def arreter(etat, delai):
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        if not membres(etat):
            return
        try:
            os.killpg(etat['pgid'], sig)
        except ProcessLookupError:
            return
        fin = time.monotonic() + delai
        while membres(etat) and time.monotonic() < fin:
            time.sleep(0.1)
    if membres(etat):
        raise RuntimeError('Des processus Fosa ne se sont pas arrêtés.')


def main():
    analyseur = argparse.ArgumentParser(description=__doc__)
    analyseur.add_argument('action', choices=('lancer', 'arreter', 'actif'))
    analyseur.add_argument('--etat', type=Path, required=True)
    analyseur.add_argument('--journal', type=Path)
    arguments = sys.argv[1:]
    separation = arguments.index('--') if '--' in arguments else len(arguments)
    commande = arguments[separation + 1:]
    args = analyseur.parse_args(arguments[:separation])
    chemin = args.etat.expanduser().resolve()
    chemin.parent.mkdir(parents=True, exist_ok=True)
    with chemin.with_suffix('.lock').open('w') as verrou:
        fcntl.flock(verrou, fcntl.LOCK_EX)
        etat = lire_etat(chemin)
        if args.action == 'actif':
            return 0 if membres(etat) else 1
        if args.action == 'arreter':
            regles = Path(__file__).resolve().parents[1] / 'caytu_config/config/exploitation.json'
            delai = json.loads(regles.read_text())['arret_delai_sec']
            arreter(etat, delai)
            chemin.unlink(missing_ok=True)
            return 0
        if membres(etat):
            analyseur.error(f'Une session Fosa est déjà active : {chemin}')
        if not commande or args.journal is None:
            analyseur.error('lancer exige --journal et -- COMMANDE')
        args.journal.parent.mkdir(parents=True, exist_ok=True)
        jeton = uuid.uuid4().hex
        with args.journal.open('ab', buffering=0) as journal:
            processus = subprocess.Popen(
                commande, stdin=subprocess.DEVNULL, stdout=journal,
                stderr=subprocess.STDOUT, start_new_session=True,
                env={**os.environ, 'FOSA_SESSION_ID': jeton})
        chemin.write_text(json.dumps({'pgid': processus.pid, 'jeton': jeton,
                                     'journal': str(args.journal)}), encoding='utf-8')
        print(f'Session Fosa {processus.pid} ; journal : {args.journal}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
