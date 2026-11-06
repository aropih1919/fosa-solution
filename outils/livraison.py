#!/usr/bin/env python3
"""Archive reproductible des sources Fosa présentes, avec manifeste SHA-256."""

import argparse
import fnmatch
import hashlib
import json
from pathlib import Path
import subprocess
import zipfile


def creer_archive(depot, destination):
    depot = depot.resolve()
    regles = json.loads((depot / 'caytu_config/config/exploitation.json').read_text())
    fichiers = []
    for racine in regles['livraison_racines']:
        chemin = depot / racine
        if not chemin.exists():
            raise ValueError(f'Source requise absente : {chemin}')
        for fichier in ([chemin] if chemin.is_file() else sorted(chemin.rglob('*'))):
            relatif = fichier.relative_to(depot)
            if any(fnmatch.fnmatch(partie, motif) for partie in relatif.parts
                   for motif in regles['livraison_exclusions']):
                continue
            if fichier.is_symlink():
                raise ValueError(f'Lien symbolique non livrable : {relatif}')
            if fichier.is_file():
                fichiers.append(fichier)
    try:
        revision = subprocess.check_output(
            ['git', '-C', str(depot), 'rev-parse', 'HEAD'], text=True,
            stderr=subprocess.DEVNULL).strip()
        modifie = bool(subprocess.check_output(
            ['git', '-C', str(depot), 'status', '--porcelain'], text=True))
    except subprocess.CalledProcessError:
        revision, modifie = 'archive', False
    manifeste = {'revision': revision, 'modifications_locales': modifie, 'fichiers': {}}
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporaire = destination.with_suffix('.zip.tmp')
    try:
        with zipfile.ZipFile(temporaire, 'w', zipfile.ZIP_DEFLATED) as archive:
            for fichier in sorted(fichiers):
                nom = 'src/fosa-solution/' + fichier.relative_to(depot).as_posix()
                contenu = fichier.read_bytes()
                info = zipfile.ZipInfo(nom, date_time=(1980, 1, 1, 0, 0, 0))
                info.external_attr = (fichier.stat().st_mode & 0o777) << 16
                info.compress_type = zipfile.ZIP_DEFLATED
                archive.writestr(info, contenu)
                manifeste['fichiers'][nom] = hashlib.sha256(contenu).hexdigest()
            info = zipfile.ZipInfo('MANIFESTE.json', date_time=(1980, 1, 1, 0, 0, 0))
            info.external_attr = 0o644 << 16
            archive.writestr(info, json.dumps(
                manifeste, ensure_ascii=False, indent=2, sort_keys=True))
        temporaire.replace(destination)
    finally:
        temporaire.unlink(missing_ok=True)
    return manifeste


def verifier(dossier):
    manifeste = json.loads((dossier / 'MANIFESTE.json').read_text())
    for nom, empreinte in manifeste['fichiers'].items():
        chemin = dossier / nom
        if (not chemin.resolve().is_relative_to(dossier.resolve())
                or hashlib.sha256(chemin.read_bytes()).hexdigest() != empreinte):
            raise ValueError(f'Livraison altérée : {nom}')
    return manifeste


def main():
    analyseur = argparse.ArgumentParser(description=__doc__)
    analyseur.add_argument('action', choices=('creer', 'verifier'))
    analyseur.add_argument('chemin', type=Path)
    args = analyseur.parse_args()
    if args.action == 'creer':
        manifeste = creer_archive(Path(__file__).resolve().parents[1], args.chemin)
    else:
        manifeste = verifier(args.chemin)
    print(f'{len(manifeste["fichiers"])} fichiers ; révision {manifeste["revision"]} ; '
          f'modifications locales : {manifeste["modifications_locales"]}')


if __name__ == '__main__':
    main()
