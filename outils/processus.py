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


def _champs_stat(pid):
    """Champs de /proc/<pid>/stat après le nom, ou None si illisible."""
    stat = _stat(pid)
    if stat is None:
        return None
    try:
        return stat.rsplit(') ', 1)[1].split()
    except (IndexError, ValueError):
        return None


def _starttime(pid):
    """Date de démarrage du PID : distingue un PID réutilisé.

    Le champ 22 de /proc/<pid>/stat est le 20e après le nom du processus.
    """
    champs = _champs_stat(pid)
    if champs is None or len(champs) < 20:
        return None
    return champs[19]


def _vivant(pid):
    """Un zombie n'est plus un processus à arrêter."""
    champs = _champs_stat(pid)
    if not champs:
        return False
    return champs[0] != 'Z'


def _parent(pid):
    champs = _champs_stat(pid)
    if champs is None or len(champs) < 2:
        return None
    try:
        return int(champs[1])
    except ValueError:
        return None


def _cmdline(pid):
    try:
        return (Path(f'/proc/{pid}/cmdline').read_bytes()
                .replace(b'\0', b' ').decode('utf-8', 'replace'))
    except OSError:
        return ''


def _argv(pid):
    try:
        brut = Path(f'/proc/{pid}/cmdline').read_bytes()
    except OSError:
        return []
    return [morceau.decode('utf-8', 'replace')
            for morceau in brut.split(b'\0') if morceau]


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

    Un éditeur, un `less` ou un `grep` ouvert sur un fichier du dépôt ne doit
    jamais être arrêté : on regarde les arguments réels, pas la ligne de
    commande concaténée. Seuls les exécutables de build/install, `ros2 launch`
    et notre propre script de lancement sont reconnus.
    """
    argv = _argv(pid)
    if not argv:
        return False
    racines = {str(portee).rstrip('/') for portee in portees if portee}
    for portee in racines:
        for argument in argv:
            if (argument.startswith(f'{portee}/install/')
                    or argument.startswith(f'{portee}/build/')):
                return True
        for index, argument in enumerate(argv):
            if (argument.endswith('/run.sh')
                    and argument.startswith(f'{portee}/')
                    and 'course-premier-plan' in argv[index + 1:]):
                return True
    # `ros2 launch` ne contient pas forcément le chemin du workspace dans sa
    # ligne de commande ; ses nœuds enfants, eux, sont sous install/build.
    if (Path(argv[0]).name == 'ros2' and len(argv) >= 3
            and argv[1] == 'launch' and 'course.launch.py' in argv[2:]):
        return True
    return False


def _racine_valide(etat):
    """Le PID racine de l'état est-il encore celui de notre session ?

    Le jeton seul suffit pour un ancien état ; l'empreinte `starttime` évite
    qu'un PID réutilisé par un programme extérieur soit suivi avec ses
    descendants.
    """
    if not etat:
        return False
    racine = int(etat.get('pid') or 0)
    if not racine or not _vivant(racine):
        return False
    empreinte = etat.get('starttime')
    if empreinte is not None:
        return _starttime(racine) == str(empreinte)
    jeton = etat.get('jeton')
    return bool(jeton) and f'FOSA_SESSION_ID={jeton}'.encode() in _environ(racine)


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
        # Les descendants ne sont suivis que si le PID racine est bien le
        # nôtre. Un PID réutilisé par un programme extérieur ne doit jamais
        # entraîner l'arrêt de ses enfants.
        descendants = (_descendants(racine, parents)
                       if _racine_valide(etat) else set())
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


def _delai_arret():
    regles = (Path(__file__).resolve().parents[1]
              / 'caytu_config/config/exploitation.json')
    return json.loads(regles.read_text(encoding='utf-8'))['arret_delai_sec']


def main():
    analyseur = argparse.ArgumentParser(description=__doc__)
    analyseur.add_argument(
        'action', choices=('lancer', 'arreter', 'actif', 'lister'))
    analyseur.add_argument('--etat', type=Path, required=True)
    analyseur.add_argument('--journal', type=Path)
    analyseur.add_argument(
        '--jeton', type=str, default=None,
        help='actif : ne répond oui que pour cette session précise')
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
        # Un état sans aucun membre réel ne doit jamais bloquer la relance.
        if etat and not membres(etat):
            chemin.unlink(missing_ok=True)
            etat = None
        if args.action == 'actif':
            if args.jeton and (not etat or etat.get('jeton') != args.jeton):
                return 1
            return 0 if membres(etat, args.portee) else 1
        if args.action == 'lister':
            cibles = membres(etat, args.portee)
            for pid in cibles:
                print(f'{pid}\t{_cmdline(pid)}')
            return 0 if cibles else 1
        if args.action == 'arreter':
            arreter(etat, _delai_arret(), args.portee)
            chemin.unlink(missing_ok=True)
            return 0
        # Un nouveau lancement remplace toute session ou tout nœud Fosa déjà
        # en cours : une seule source de commandes doit rester active.
        restes = membres(etat, args.portee)
        if restes:
            if etat and not _racine_valide(etat):
                message = ('Ancienne session sans lanceur : arrêt des '
                           'processus restants.')
            else:
                message = 'Session Fosa déjà en cours : arrêt avant le nouveau lancement.'
            print(message, file=sys.stderr)
            arreter(etat, _delai_arret(), args.portee)
            chemin.unlink(missing_ok=True)
            etat = None
        if membres(etat, args.portee):
            analyseur.error('Des processus Fosa résistent à l’arrêt.')
        if not commande or args.journal is None:
            analyseur.error('lancer exige --journal et -- COMMANDE')
        args.journal.parent.mkdir(parents=True, exist_ok=True)
        jeton = uuid.uuid4().hex
        with args.journal.open('ab', buffering=0) as journal:
            processus = subprocess.Popen(
                commande, stdin=subprocess.DEVNULL, stdout=journal,
                stderr=subprocess.STDOUT, start_new_session=True,
                env={**os.environ, 'FOSA_SESSION_ID': jeton})
        empreinte = _starttime(processus.pid)
        chemin.write_text(json.dumps({
            'pid': processus.pid, 'pgid': processus.pid, 'jeton': jeton,
            'starttime': empreinte, 'journal': str(args.journal)}),
            encoding='utf-8')
        print(f'Session Fosa {processus.pid} ; journal : {args.journal}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
