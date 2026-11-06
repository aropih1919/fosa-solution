#!/usr/bin/env python3
"""Lance et arrête uniquement les groupes de processus créés par cette solution.

Un jeton hérité identifie les descendants, même si le lanceur est déjà mort et
même si ROS 2 les a placés dans une autre session. Le fichier d'état ne suffit
jamais, à lui seul, pour envoyer un signal à un PID : le jeton et la portée du
workspace sont vérifiés avant tout arrêt.
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


def _pids():
    return [int(dossier.name) for dossier in Path('/proc').iterdir()
            if dossier.name.isdigit()]


def _stat(pid):
    try:
        return Path(f'/proc/{pid}/stat').read_text(encoding='utf-8', errors='replace')
    except OSError:
        return None


def _vivant(pid):
    """Un zombie n'est plus un processus à arrêter."""
    stat = _stat(pid)
    if stat is None:
        return False
    try:
        return stat.rsplit(') ', 1)[1].split()[0] != 'Z'
    except (IndexError, ValueError):
        return False


def _parent(pid):
    stat = _stat(pid)
    if stat is None:
        return None
    try:
        return int(stat.rsplit(') ', 1)[1].split()[1])
    except (IndexError, ValueError):
        return None


def _cmdline(pid):
    try:
        return (Path(f'/proc/{pid}/cmdline').read_bytes()
                .replace(b'\0', b' ').decode('utf-8', 'replace'))
    except OSError:
        return ''


def _environ(pid):
    try:
        return Path(f'/proc/{pid}/environ').read_bytes().split(b'\0')
    except OSError:
        return []


def _ancetres():
    """Le shell d'arrêt ne doit jamais se tuer lui-même."""
    vus = set()
    pid = os.getpid()
    while pid > 1 and pid not in vus:
        vus.add(pid)
        pid = _parent(pid) or 0
    return vus


def _descendants(racine, parents):
    enfants = {}
    for pid, parent in parents.items():
        enfants.setdefault(parent, []).append(pid)
    resultat = set()
    pile = [racine]
    while pile:
        for enfant in enfants.get(pile.pop(), ()):
            if enfant not in resultat:
                resultat.add(enfant)
                pile.append(enfant)
    return resultat


def _dans_portee(pid, portees):
    """Processus ROS lancé depuis ce workspace (actuel ou ancienne livraison).

    Un éditeur ouvert sur un fichier du dépôt ne doit jamais être arrêté :
    seuls les exécutables de build/install, les lancements de la course et
    notre propre script de lancement sont reconnus.
    """
    commande = _cmdline(pid)
    if not any(portee in commande for portee in portees):
        return False
    return ('/install/' in commande or '/build/' in commande
            or 'course.launch.py' in commande or 'course_parc' in commande
            or 'fosa-solution/run.sh course-premier-plan' in commande)


def membres(etat, portees=None):
    """PIDs de la session Fosa : jeton, descendants et portée du workspace."""
    portees = [str(portee) for portee in (portees or []) if portee]
    exclus = _ancetres()
    parents = {}
    for pid in _pids():
        if pid in exclus:
            continue
        parent = _parent(pid)
        if parent is not None:
            parents[pid] = parent
    resultat = set()
    if etat:
        marque = f'FOSA_SESSION_ID={etat["jeton"]}'.encode()
        racine = int(etat.get('pid') or 0)
        descendants = _descendants(racine, parents) if racine else set()
        pgid = int(etat.get('pgid') or 0)
        # Ancien état (sans pid) : on n'utilise le groupe que si son chef porte
        # encore le jeton, pour ne jamais tuer un PID réutilisé par un autre.
        groupe_sur = bool(pgid and _vivant(pgid) and marque in _environ(pgid))
        for pid in parents:
            appartient = marque in _environ(pid) or pid in descendants
            if not appartient and groupe_sur:
                try:
                    appartient = os.getpgid(pid) == pgid
                except ProcessLookupError:
                    appartient = False
            if appartient:
                resultat.add(pid)
    if portees:
        for pid in parents:
            if _dans_portee(pid, portees):
                resultat.add(pid)
    return sorted(pid for pid in resultat if _vivant(pid))


def lire_etat(chemin):
    if not chemin.exists():
        return None
    return json.loads(chemin.read_text(encoding='utf-8'))


def arreter(etat, delai, portees=None):
    if not membres(etat, portees):
        return
    for sig, attente in ((signal.SIGINT, max(2.0, delai - 4.0)),
                         (signal.SIGTERM, 2.0),
                         (signal.SIGKILL, 2.0)):
        cibles = membres(etat, portees)
        if not cibles:
            return
        for pid in cibles:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        fin = time.monotonic() + attente
        while membres(etat, portees) and time.monotonic() < fin:
            time.sleep(0.1)
    restants = membres(etat, portees)
    if restants:
        details = '\n'.join(f'  {pid}: {_cmdline(pid)}' for pid in restants)
        raise RuntimeError(f'Des processus Fosa ne se sont pas arrêtés :\n{details}')


def main():
    analyseur = argparse.ArgumentParser(description=__doc__)
    analyseur.add_argument('action', choices=('lancer', 'arreter', 'actif'))
    analyseur.add_argument('--etat', type=Path, required=True)
    analyseur.add_argument('--journal', type=Path)
    analyseur.add_argument(
        '--portee', type=str, action='append', default=None,
        help='racine du workspace : arrête aussi les processus Fosa sans état')
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
            return 0 if membres(etat, args.portee) else 1
        if args.action == 'arreter':
            regles = Path(__file__).resolve().parents[1] / 'caytu_config/config/exploitation.json'
            delai = json.loads(regles.read_text(encoding='utf-8'))['arret_delai_sec']
            arreter(etat, delai, args.portee)
            chemin.unlink(missing_ok=True)
            return 0
        if membres(etat, args.portee):
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
        chemin.write_text(json.dumps({
            'pid': processus.pid, 'pgid': processus.pid, 'jeton': jeton,
            'journal': str(args.journal)}), encoding='utf-8')
        print(f'Session Fosa {processus.pid} ; journal : {args.journal}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
