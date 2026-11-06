"""Arrêter Fosa ne doit jamais arrêter un processus extérieur à sa session."""

import json
from pathlib import Path
import subprocess
import sys

SCRIPT = Path(__file__).resolve().parents[1] / 'processus.py'


def test_arret_cible_et_pid_reutilise(tmp_path):
    etat, journal = tmp_path / 'session.json', tmp_path / 'session.log'
    autre = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
    commande = [sys.executable, str(SCRIPT)]
    try:
        subprocess.run(commande + ['lancer', '--etat', str(etat), '--journal', str(journal),
                       '--', sys.executable, '-c', 'import time; time.sleep(60)'], check=True)
        subprocess.run(commande + ['actif', '--etat', str(etat)], check=True)
        subprocess.run(commande + ['arreter', '--etat', str(etat)], check=True)
        assert autre.poll() is None
        etat.write_text(json.dumps({'pgid': autre.pid, 'jeton': 'ancien-pid'}))
        subprocess.run(commande + ['arreter', '--etat', str(etat)], check=True)
        assert autre.poll() is None
    finally:
        autre.terminate()
        autre.wait(timeout=5)
        subprocess.run(commande + ['arreter', '--etat', str(etat)], check=True)
