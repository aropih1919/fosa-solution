"""Essai du même détecteur sur fichiers, sans ROS ni caméra."""

import argparse
import json
from pathlib import Path

import cv2

from caytu_config import load_config
from caytu_perception.tags_image import (
    DetecteurImage, annoter_image, enregistrer_image, generer_scenes)


def main():
    arguments = argparse.ArgumentParser(description=__doc__)
    arguments.add_argument('--sortie', type=Path, required=True,
                           help='Nouveau dossier de résultats (doit être absent).')
    arguments.add_argument('--images', type=Path, nargs='+',
                           help='Photos à analyser ; sinon génère les scènes de test.')
    options = arguments.parse_args()
    reglages = load_config().perception.apriltag
    detecteur = DetecteurImage(reglages.family)
    options.sortie.mkdir(parents=True, exist_ok=False)
    if options.images:
        scenes = {}
        for numero, chemin in enumerate(options.images):
            image = cv2.imread(str(chemin), cv2.IMREAD_GRAYSCALE)
            if image is None:
                raise ValueError(f'Image illisible : {chemin}')
            scenes[f'{numero:04d}_{chemin.stem}'] = (image, None)
    else:
        scenes = generer_scenes(detecteur.dictionnaire, reglages.test_ids)
    rapport = {'opencv': cv2.__version__, 'famille': reglages.family, 'images': []}
    echec = False
    for nom, (image, attendus) in scenes.items():
        coins, ids = detecteur.detecter(image)
        trouves = [] if ids is None else sorted(int(i) for i in ids.flatten())
        conforme = None if attendus is None else trouves == sorted(attendus)
        echec = echec or conforme is False
        enregistrer_image(options.sortie / 'sources' / f'{nom}.png', image)
        enregistrer_image(options.sortie / 'annotees' / f'{nom}.png',
                          annoter_image(image, coins, ids))
        rapport['images'].append({'nom': nom, 'ids': trouves,
                                  'attendus': attendus, 'conforme': conforme})
        print(f'{nom} : {trouves}')
    (options.sortie / 'rapport.json').write_text(
        json.dumps(rapport, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    return 1 if echec else 0


if __name__ == '__main__':
    raise SystemExit(main())
