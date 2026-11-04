"""Exécute YOLO sur une image ou une vidéo et écrit les annotations."""

import argparse
import os
from pathlib import Path

import cv2

from caytu_config import load_config
from caytu_config.loader import config_directory
from caytu_perception.classificateur_obstacles import (
    _extraire_boites_yolo,
    _preparer_image_yolo,
)


def _charger_modele(chemin, configuration):
    chemin = os.path.expanduser(chemin)
    if not os.path.isabs(chemin):
        chemin = str(config_directory() / chemin)
    if not os.path.isfile(chemin):
        raise FileNotFoundError(f"Modèle YOLO introuvable : {chemin}")
    reseau = cv2.dnn.readNetFromONNX(chemin)
    if configuration.device.lower() == 'cuda':
        reseau.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        reseau.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
    return reseau


def _annoter(image, reseau, configuration):
    hauteur, largeur = image.shape[:2]
    image_yolo, _, _, _ = _preparer_image_yolo(
        image, configuration.input_size)
    blob = cv2.dnn.blobFromImage(
        image_yolo, scalefactor=1.0 / 255.0, swapRB=True)
    reseau.setInput(blob)
    boites = _extraire_boites_yolo(
        reseau.forward(), largeur, hauteur, configuration)
    for categorie, confiance, x1, y1, x2, y2 in boites:
        debut = (round(x1), round(y1))
        fin = (round(x2), round(y2))
        couleur = (0, 200, 0) if categorie == 'personne' else (0, 140, 255)
        cv2.rectangle(image, debut, fin, couleur, 2)
        cv2.putText(
            image, f'{categorie} {confiance:.2f}', (debut[0], max(18, debut[1] - 6)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, couleur, 2)
    return image, len(boites)


def main(args=None):
    analyseur = argparse.ArgumentParser(
        description='Valide les catégories YOLO personne/obstacle sur image ou vidéo.')
    analyseur.add_argument('--source', required=True, help='Image ou vidéo à analyser.')
    analyseur.add_argument('--sortie', required=True, help='Fichier annoté de sortie.')
    analyseur.add_argument('--modele', default='', help='Modèle ONNX (défaut : fosa.yaml).')
    options = analyseur.parse_args(args=args)

    configuration = load_config().perception.yolo
    chemin_modele = options.modele or configuration.model_path
    reseau = _charger_modele(chemin_modele, configuration)
    source = Path(options.source).expanduser()
    sortie = Path(options.sortie).expanduser()
    if not source.is_file():
        raise FileNotFoundError(f"Source introuvable : {source}")
    sortie.parent.mkdir(parents=True, exist_ok=True)

    image = cv2.imread(str(source))
    if image is not None:
        image, nombre = _annoter(image, reseau, configuration)
        if not cv2.imwrite(str(sortie), image):
            raise OSError(f"Impossible d'écrire l'image annotée : {sortie}")
        print(f'{nombre} détection(s) ; image annotée : {sortie}')
        return

    lecteur = cv2.VideoCapture(str(source))
    if not lecteur.isOpened():
        raise ValueError(f"La source n'est ni une image ni une vidéo lisible : {source}")
    fps = lecteur.get(cv2.CAP_PROP_FPS)
    largeur = int(lecteur.get(cv2.CAP_PROP_FRAME_WIDTH))
    hauteur = int(lecteur.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if fps <= 0.0 or largeur <= 0 or hauteur <= 0:
        lecteur.release()
        raise ValueError(f"Métadonnées vidéo invalides : {source}")
    codeur = cv2.VideoWriter_fourcc(*'mp4v')
    ecrivain = cv2.VideoWriter(str(sortie), codeur, fps, (largeur, hauteur))
    if not ecrivain.isOpened():
        lecteur.release()
        raise OSError(f"Impossible de créer la vidéo annotée : {sortie}")
    images = detections = 0
    try:
        while True:
            ok, image = lecteur.read()
            if not ok:
                break
            image, nombre = _annoter(image, reseau, configuration)
            ecrivain.write(image)
            images += 1
            detections += nombre
    finally:
        lecteur.release()
        ecrivain.release()
    print(f'{images} image(s), {detections} détection(s) ; vidéo annotée : {sortie}')


if __name__ == '__main__':
    main()
