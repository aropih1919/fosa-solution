"""Livraison limitée et outils exécutables après transfert."""

import importlib.util
import json
import os
from pathlib import Path
import zipfile

import pytest

RACINE = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('livraison', RACINE / 'outils/livraison.py')
livraison = importlib.util.module_from_spec(spec)
spec.loader.exec_module(livraison)


def test_archive_limitee_aux_sources_et_verifiable(tmp_path):
    depot = tmp_path / 'depot'
    config = depot / 'caytu_config/config'
    config.mkdir(parents=True)
    (config / 'exploitation.json').write_text(json.dumps({
        'livraison_racines': ['caytu_config', 'run.sh'],
        'livraison_exclusions': ['__pycache__', '*.pyc']}))
    (depot / 'run.sh').write_text('#!/bin/bash\nexit 0\n')
    (depot / 'run.sh').chmod(0o755)
    (config / 'ancien.pyc').write_bytes(b'artefact')
    (depot / 'autre_equipe').mkdir()
    (depot / 'autre_equipe/secret').write_text('exclu')
    archive = tmp_path / 'fosa.zip'
    manifeste = livraison.creer_archive(depot, archive)
    assert len(manifeste['fichiers']) == 2
    with zipfile.ZipFile(archive) as contenu:
        contenu.extractall(tmp_path / 'extrait')
        assert contenu.getinfo('src/fosa-solution/run.sh').external_attr >> 16 == 0o755
    livraison.verifier(tmp_path / 'extrait')
    (tmp_path / 'extrait/src/fosa-solution/run.sh').write_text('altéré')
    with pytest.raises(ValueError, match='altérée'):
        livraison.verifier(tmp_path / 'extrait')


def test_la_pose_de_verite_gazebo_est_pontee_vers_un_topic_distinct():
    regles = json.loads((RACINE / 'caytu_config/config/exploitation.json').read_text())
    simulation = regles['simulation']
    assert simulation['gz_pose_topic'] == '/model/sitoe_robot/pose'
    assert simulation['jugement']['topic_pose'] == '/sim/pose_verite'


def test_les_outils_lances_directement_sont_executables():
    for nom in ('run.sh', 'deployer_robot.sh', 'outils/diagnostic_dds.py',
                'outils/diagnostic_robot.sh', 'outils/verifier.sh',
                'simulation/simulation.sh'):
        assert os.access(RACINE / nom, os.X_OK), f'{nom} doit être exécutable'


def test_un_lien_vers_l_exterieur_est_refuse(tmp_path):
    config = tmp_path / 'caytu_config/config'
    config.mkdir(parents=True)
    (config / 'exploitation.json').write_text(json.dumps({
        'livraison_racines': ['caytu_config'], 'livraison_exclusions': []}))
    (config / 'fuite').symlink_to('/etc/passwd')
    with pytest.raises(ValueError, match='Lien symbolique'):
        livraison.creer_archive(tmp_path, tmp_path / 'archive.zip')
