"""Régression CycloneDDS : exécuter la partie réseau en conteneur isolé."""

from contextlib import ExitStack
import os
from pathlib import Path
import socket
import subprocess
import sys

import pytest


DEPOT = Path(__file__).resolve().parents[2]
CONFIG = DEPOT / 'caytu_config' / 'config'
SCRIPT = DEPOT / 'outils' / 'environnement_dds.sh'
DIAGNOSTIC = DEPOT / 'outils' / 'diagnostic_dds.py'
BASE = '''<CycloneDDS><Domain Id="any">
  <General><Interfaces><NetworkInterface name="lo"/></Interfaces>
    <AllowMulticast>false</AllowMulticast></General>
  <Discovery><ParticipantIndex>auto</ParticipantIndex>
    <MaxAutoParticipantIndex>99</MaxAutoParticipantIndex></Discovery>
</Domain></CycloneDDS>'''


def environnement(**valeurs):
    resultat = dict(os.environ)
    for nom in ('CYCLONEDDS_URI', 'FOSA_DDS_ETENDU', 'RMW_IMPLEMENTATION'):
        resultat.pop(nom, None)
    resultat.update(FOSA_CONFIG_DIR=str(CONFIG), ROS_DOMAIN_ID='0')
    resultat.update(valeurs)
    return resultat


def configurer(env):
    return subprocess.run(
        ['bash', '-c', 'set -eu; source "$1"; configurer_dds; '
         'configurer_dds; printf "%s" "${CYCLONEDDS_URI-absent}"',
         'verification', str(SCRIPT)],
        env=env, text=True, capture_output=True, timeout=15)


def test_surcharge_facultative_et_idempotente():
    assert configurer(environnement()).stdout == 'absent'
    env = environnement(CYCLONEDDS_URI=BASE, RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
    assert configurer(env).stdout == BASE
    env['FOSA_DDS_ETENDU'] = '1'
    resultat = configurer(env)
    assert resultat.returncode == 0, resultat.stderr
    assert resultat.stdout == BASE + ',' + (CONFIG / 'cyclonedds.xml').as_uri()


def test_surcharge_refuse_un_autre_rmw():
    resultat = configurer(environnement(
        FOSA_DDS_ETENDU='1', RMW_IMPLEMENTATION='rmw_fastrtps_cpp'))
    assert resultat.returncode != 0
    assert 'RMW actif : rmw_fastrtps_cpp' in resultat.stderr


def diagnostiquer(env):
    return subprocess.run([sys.executable, str(DIAGNOSTIC)], env=env,
                          text=True, capture_output=True, timeout=20)


@pytest.mark.skipif(os.environ.get('FOSA_TEST_DDS_ISOLE') != '1',
                    reason='Réserver les ports DDS uniquement en conteneur isolé')
def test_saturation_reproduite_puis_contournee():
    env = environnement(CYCLONEDDS_URI=BASE, RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
    with ExitStack() as ressources:
        # Occuper les ports de découverte unicast de tous les indices 0 à 99.
        # Constantes du protocole DDSI standard, pour le domaine 0 du test.
        for indice in range(100):
            port = ressources.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
            port.bind(('0.0.0.0', 7410 + 2 * indice))
        initial = diagnostiquer(env)
        assert initial.returncode != 0
        assert '[OK] rclpy importé' in initial.stdout
        assert 'Failed to find a free participant index' in initial.stderr
        assert '[ÉCHEC DDS/ROS]' in initial.stderr

        env['FOSA_DDS_ETENDU'] = '1'
        surcharge = configurer(env)
        assert surcharge.returncode == 0, surcharge.stderr
        env['CYCLONEDDS_URI'] = surcharge.stdout
        corrige = diagnostiquer(env)
        assert corrige.returncode == 0, corrige.stderr
        assert '[OK] Nœud ROS créé' in corrige.stdout
        # Le processus précédent doit libérer son indice en sortant.
        assert diagnostiquer(env).returncode == 0

        for indice in range(100, 120):
            port = ressources.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
            port.bind(('0.0.0.0', 7410 + 2 * indice))
        sature = diagnostiquer(env)
        assert sature.returncode != 0
        assert 'Failed to find a free participant index' in sature.stderr


@pytest.mark.skipif(os.environ.get('FOSA_TEST_DDS_ISOLE') != '1',
                    reason='Vérification du parseur CycloneDDS en conteneur isolé')
def test_configuration_reseau_heritee_encore_appliquee(tmp_path):
    # Une interface inexistante doit encore provoquer un échec après la
    # surcharge : elle ne doit pas être remplacée par une interface par défaut.
    fichier = tmp_path / 'reseau.xml'
    fichier.write_text(BASE.replace('name="lo"', 'name="interface_absente_fosa"'))
    env = environnement(CYCLONEDDS_URI=fichier.as_uri(), FOSA_DDS_ETENDU='1',
                        RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
    resultat = configurer(env)
    assert resultat.returncode == 0, resultat.stderr
    env['CYCLONEDDS_URI'] = resultat.stdout
    resultat = diagnostiquer(env)
    assert resultat.returncode != 0
    assert 'interface_absente_fosa' in resultat.stderr
