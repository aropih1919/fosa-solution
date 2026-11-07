"""Arrêter Fosa ne doit jamais arrêter un processus extérieur à sa session."""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

SCRIPT = Path(__file__).resolve().parents[1] / 'processus.py'


def _vivant(pid):
    """Vrai seulement si le processus existe et n'est pas un zombie."""
    try:
        stat = Path(f'/proc/{pid}/stat').read_text(encoding='utf-8')
    except OSError:
        return False
    try:
        return stat.rsplit(') ', 1)[1].split()[0] != 'Z'
    except (IndexError, ValueError):
        return False


def _attendre_mort(pid, delai=5.0):
    fin = time.monotonic() + delai
    while _vivant(pid) and time.monotonic() < fin:
        time.sleep(0.1)
    return not _vivant(pid)


def _arreter_processus(pid):
    try:
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        pass


def _faux_ros2(dossier):
    """Faux binaire nommé `ros2`, lancé dans son propre groupe de processus."""
    binaire = dossier / 'bin' / 'ros2'
    binaire.parent.mkdir(parents=True)
    binaire.write_text('#!/usr/bin/env bash\nsleep 60\n', encoding='utf-8')
    binaire.chmod(0o755)
    return binaire


def _lancer_faux_launch(binaire, dossier, env=None):
    processus = subprocess.Popen(
        [str(binaire), 'launch', 'caytu_course', 'course.launch.py'],
        cwd=str(dossier), env=env, start_new_session=True)
    fin = time.monotonic() + 5.0
    while time.monotonic() < fin:
        try:
            ligne = Path(f'/proc/{processus.pid}/cmdline').read_bytes()
        except OSError:
            ligne = b''
        if b'course.launch.py' in ligne:
            return processus
        time.sleep(0.05)
    _terminer(processus)
    raise AssertionError('Le faux lancement ROS ne démarre pas')


def _terminer(processus):
    """Arrête aussi les enfants du faux lancement, sans toucher au reste."""
    try:
        os.killpg(processus.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        processus.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(processus.pid, signal.SIGKILL)
        processus.wait(timeout=5)


def _lister(tmp_path, portee):
    return subprocess.run(
        [sys.executable, str(SCRIPT), 'lister',
         '--etat', str(tmp_path / 'absent.json'), '--portee', str(portee)],
        capture_output=True, text=True)


def test_arret_cible_et_pid_reutilise(tmp_path):
    etat, journal = tmp_path / 'session.json', tmp_path / 'session.log'
    autre = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
    commande = [sys.executable, str(SCRIPT)]
    try:
        subprocess.run(commande + ['lancer', '--etat', str(etat), '--journal', str(journal),
                       '--', sys.executable, '-c', 'import time; time.sleep(60)'], check=True)
        session = json.loads(etat.read_text(encoding='utf-8'))
        assert session['starttime']
        subprocess.run(commande + ['actif', '--etat', str(etat)], check=True)
        subprocess.run(commande + ['actif', '--etat', str(etat),
                                   '--jeton', session['jeton']], check=True)
        autre_jeton = subprocess.run(
            commande + ['actif', '--etat', str(etat), '--jeton', 'inconnu'])
        assert autre_jeton.returncode == 1
        liste = subprocess.run(commande + ['lister', '--etat', str(etat)],
                               check=True, capture_output=True, text=True)
        assert str(session['pid']) in liste.stdout
        subprocess.run(commande + ['arreter', '--etat', str(etat)], check=True)
        assert autre.poll() is None
        etat.write_text(json.dumps({'pgid': autre.pid, 'jeton': 'ancien-pid'}))
        subprocess.run(commande + ['arreter', '--etat', str(etat)], check=True)
        assert autre.poll() is None
    finally:
        autre.terminate()
        autre.wait(timeout=5)
        subprocess.run(commande + ['arreter', '--etat', str(etat)], check=True)


def test_un_nouveau_lancement_arrete_l_ancien(tmp_path):
    """Lancer un mode remplace la session précédente au lieu de la refuser."""
    etat, journal = tmp_path / 'session.json', tmp_path / 'session.log'
    commande = [sys.executable, str(SCRIPT)]
    try:
        subprocess.run(commande + [
            'lancer', '--etat', str(etat), '--journal', str(journal),
            '--', sys.executable, '-c', 'import time; time.sleep(60)'],
            check=True)
        ancien = json.loads(etat.read_text(encoding='utf-8'))
        subprocess.run(commande + [
            'lancer', '--etat', str(etat), '--journal', str(journal),
            '--', sys.executable, '-c', 'import time; time.sleep(60)'],
            check=True)
        nouveau = json.loads(etat.read_text(encoding='utf-8'))
        assert nouveau['pid'] != ancien['pid']
        assert _attendre_mort(ancien['pid'])
        subprocess.run(commande + ['actif', '--etat', str(etat),
                                   '--jeton', nouveau['jeton']], check=True)
    finally:
        subprocess.run(commande + ['arreter', '--etat', str(etat)], check=True)


def test_pid_reutilise_ne_suit_pas_ses_descendants(tmp_path):
    """Un PID racine réutilisé ne doit pas entraîner l'arrêt de ses enfants."""
    etat = tmp_path / 'session.json'
    programme = tmp_path / 'parent.py'
    programme.write_text(
        'import subprocess, sys, time\n'
        "enfant = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])\n"
        'print(enfant.pid, flush=True)\n'
        'time.sleep(60)\n',
        encoding='utf-8')
    parent = subprocess.Popen(
        [sys.executable, str(programme)], stdout=subprocess.PIPE, text=True)
    enfant = None
    commande = [sys.executable, str(SCRIPT)]
    try:
        enfant = int(parent.stdout.readline())
        etat.write_text(json.dumps({
            'pid': parent.pid, 'pgid': parent.pid, 'jeton': 'ancien-pid',
            'starttime': 'empreinte-perimee'}), encoding='utf-8')
        subprocess.run(commande + ['arreter', '--etat', str(etat)], check=True)
        assert parent.poll() is None
        assert _vivant(enfant)
    finally:
        if enfant is not None:
            _arreter_processus(enfant)
        parent.terminate()
        parent.wait(timeout=5)
        subprocess.run(commande + ['arreter', '--etat', str(etat)], check=True)


def test_arret_termine_un_enfant_orphelin_dans_une_autre_session(tmp_path):
    """Un enfant qui survit au lanceur (nouvelle session) doit être arrêté."""
    etat, journal = tmp_path / 'session.json', tmp_path / 'session.log'
    trace = tmp_path / 'enfant.pid'
    programme = tmp_path / 'lanceur.py'
    programme.write_text(
        'import subprocess, sys\n'
        'enfant = subprocess.Popen(\n'
        "    [sys.executable, '-c', 'import time; time.sleep(60)'],\n"
        '    start_new_session=True)\n'
        'open(sys.argv[1], "w", encoding="utf-8").write(str(enfant.pid))\n',
        encoding='utf-8')
    commande = [sys.executable, str(SCRIPT)]
    enfant = None
    try:
        subprocess.run(
            commande + ['lancer', '--etat', str(etat), '--journal', str(journal),
                        '--', sys.executable, str(programme), str(trace)],
            check=True)
        fin = time.monotonic() + 5.0
        while not trace.is_file() and time.monotonic() < fin:
            time.sleep(0.05)
        enfant = int(trace.read_text(encoding='utf-8'))
        assert _vivant(enfant)
        subprocess.run(commande + ['arreter', '--etat', str(etat)], check=True)
        assert _attendre_mort(enfant)
    finally:
        if enfant is not None:
            _arreter_processus(enfant)
        subprocess.run(commande + ['arreter', '--etat', str(etat)], check=True)


def test_etat_perime_est_supprime(tmp_path):
    etat = tmp_path / 'session.json'
    fini = subprocess.Popen([sys.executable, '-c', 'pass'])
    fini.wait(timeout=5)
    etat.write_text(json.dumps({
        'pid': fini.pid, 'pgid': fini.pid, 'jeton': 'ancien-pid'}),
        encoding='utf-8')
    resultat = subprocess.run(
        [sys.executable, str(SCRIPT), 'actif', '--etat', str(etat)])
    assert resultat.returncode == 1
    assert not etat.exists()


def test_lister_reconnait_le_lanceur_du_workspace(tmp_path):
    """Le lanceur `run.sh course-premier-plan` est reconnu par sa portée."""
    lanceur = tmp_path / 'run.sh'
    lanceur.write_text('#!/usr/bin/env bash\nsleep 60\n', encoding='utf-8')
    processus = subprocess.Popen(['bash', str(lanceur), 'course-premier-plan'],
                                 start_new_session=True)
    try:
        resultat = subprocess.run(
            [sys.executable, str(SCRIPT), 'lister',
             '--etat', str(tmp_path / 'absent.json'), '--portee', str(tmp_path)],
            capture_output=True, text=True)
        assert resultat.returncode == 0
        assert str(processus.pid) in resultat.stdout
    finally:
        _terminer(processus)


def test_lister_ignore_un_editeur_ouvert_sur_le_lancement(tmp_path):
    """Un processus qui ne fait qu'afficher un fichier ne doit pas être tué."""
    cible = tmp_path / 'course.launch.py'
    cible.write_text('', encoding='utf-8')
    processus = subprocess.Popen(
        [sys.executable, '-c', 'import time; time.sleep(60)', str(cible)])
    try:
        resultat = subprocess.run(
            [sys.executable, str(SCRIPT), 'lister',
             '--etat', str(tmp_path / 'absent.json'), '--portee', str(tmp_path)],
            capture_output=True, text=True)
        assert resultat.returncode == 1
        assert str(processus.pid) not in resultat.stdout
    finally:
        processus.terminate()
        processus.wait(timeout=5)


def test_lister_reconnait_notre_ros2_launch(tmp_path):
    """Un `ros2 launch course.launch.py` lancé depuis notre workspace est nôtre."""
    notre = tmp_path / 'notre'
    notre.mkdir()
    processus = _lancer_faux_launch(_faux_ros2(notre), notre)
    try:
        resultat = _lister(tmp_path, notre)
        assert resultat.returncode == 0
        assert str(processus.pid) in resultat.stdout
    finally:
        _terminer(processus)


def test_lister_ignore_un_ros2_launch_homonyme_hors_portee(tmp_path):
    """Le lancement homonyme d'une autre équipe ne doit jamais être arrêté."""
    notre, autre = tmp_path / 'notre', tmp_path / 'autre'
    notre.mkdir()
    autre.mkdir()
    processus = _lancer_faux_launch(_faux_ros2(autre), autre)
    try:
        resultat = _lister(tmp_path, notre)
        assert resultat.returncode == 1
        assert str(processus.pid) not in resultat.stdout
    finally:
        _terminer(processus)


def test_lister_reconnait_un_noeud_avec_environnement_de_notre_workspace(tmp_path):
    """Les nœuds lancés ailleurs restent nôtres via l'environnement exporté."""
    notre, autre = tmp_path / 'notre', tmp_path / 'autre'
    notre.mkdir()
    autre.mkdir()
    env = {**os.environ, 'PYTHONPATH': f'{notre}/caytu_config'}
    processus = _lancer_faux_launch(_faux_ros2(autre), autre, env=env)
    try:
        resultat = _lister(tmp_path, notre)
        assert resultat.returncode == 0
        assert str(processus.pid) in resultat.stdout
    finally:
        _terminer(processus)
