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


def _racine_robot(tmp_path):
    racine = tmp_path / 'fosa'
    ancienne = racine / 'livraisons/ancienne'
    nouvelle = racine / 'livraisons/nouvelle'
    (ancienne / 'src').mkdir(parents=True)
    (nouvelle / 'src').mkdir(parents=True)
    (racine / 'donnees/sessions').mkdir(parents=True)
    (racine / 'donnees/sessions/course.json').write_text('{}')
    (racine / 'actuel').symlink_to(ancienne)
    (racine / 'precedent').symlink_to(ancienne)
    return racine, ancienne, nouvelle


def test_activation_remplace_tout_sauf_les_donnees(tmp_path):
    racine, ancienne, nouvelle = _racine_robot(tmp_path)
    livraison.activer(racine, nouvelle)
    assert not ancienne.exists()
    assert (nouvelle / 'src').is_dir()
    assert (racine / 'actuel').resolve() == nouvelle.resolve()
    assert not (racine / 'precedent').exists()
    assert (racine / 'donnees/sessions/course.json').read_text() == '{}'


def test_activation_refuse_une_livraison_hors_du_dossier(tmp_path):
    racine, _, _ = _racine_robot(tmp_path)
    dehors = tmp_path / 'dehors'
    dehors.mkdir()
    with pytest.raises(ValueError, match='hors de'):
        livraison.activer(racine, dehors)
    assert (racine / 'actuel').is_symlink()


def test_activation_refuse_de_supprimer_un_dossier_actuel(tmp_path):
    racine, _, nouvelle = _racine_robot(tmp_path)
    (racine / 'actuel').unlink()
    (racine / 'actuel').mkdir()
    with pytest.raises(ValueError, match='Refus'):
        livraison.activer(racine, nouvelle)
    assert (racine / 'actuel').is_dir()
    assert (racine / 'livraisons/ancienne').is_dir()
