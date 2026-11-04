#!/usr/bin/env python3
"""Génère les modèles Gazebo des AprilTags 1, 4, 7 et 9 (36h11).

Chaque modèle : un poteau, un panneau blanc et le tag (carré noir de 16,5 cm
entouré d'une marge blanche d'une case, comme les tags imprimés) dont le
centre est à 1 m du sol. Le tag regarde vers +x du modèle.

Usage : generer_tags.py <dossier de sortie>
"""

import sys
from pathlib import Path

import cv2

TAGS = (1, 4, 7, 9)
COTE_NOIR = 0.165              # règlement : 16,5 cm
CASES_NOIRES, CASES_TOTALES = 8, 10   # 6 cases de données + bord noir, + marge
PIXELS_PAR_CASE = 40
HAUTEUR = 1.0


def image_tag(tag_id):
    dictionnaire = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    cote = CASES_NOIRES * PIXELS_PAR_CASE
    if hasattr(cv2.aruco, 'generateImageMarker'):
        noir = cv2.aruco.generateImageMarker(dictionnaire, tag_id, cote)
    else:
        noir = cv2.aruco.drawMarker(dictionnaire, tag_id, cote)
    marge = PIXELS_PAR_CASE * (CASES_TOTALES - CASES_NOIRES) // 2
    return cv2.copyMakeBorder(noir, marge, marge, marge, marge,
                              cv2.BORDER_CONSTANT, value=255)


def maillage(tag_id, demi):
    """Carré dans le plan y-z, normale +x. Vu depuis +x, la droite est +y."""
    return f"""mtllib tag{tag_id}.mtl
v 0 {-demi} {-demi}
v 0 {demi} {-demi}
v 0 {demi} {demi}
v 0 {-demi} {demi}
vt 0 0
vt 1 0
vt 1 1
vt 0 1
vn 1 0 0
usemtl tag{tag_id}
f 1/1/1 2/2/1 3/3/1 4/4/1
"""


def modele(tag_id, dossier):
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="tag{tag_id}">
    <static>true</static>
    <link name="lien">
      <visual name="tag">
        <pose>0.006 0 {HAUTEUR} 0 0 0</pose>
        <geometry><mesh><uri>file://{dossier}/tag{tag_id}.obj</uri></mesh></geometry>
      </visual>
      <visual name="panneau">
        <pose>-0.005 0 {HAUTEUR} 0 0 0</pose>
        <geometry><box><size>0.02 0.24 0.24</size></box></geometry>
        <material><ambient>0.9 0.9 0.9 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material>
      </visual>
      <collision name="poteau">
        <pose>-0.07 0 0.44 0 0 0</pose>
        <geometry><cylinder><radius>0.03</radius><length>0.88</length></cylinder></geometry>
      </collision>
      <visual name="poteau">
        <pose>-0.07 0 0.44 0 0 0</pose>
        <geometry><cylinder><radius>0.03</radius><length>0.88</length></cylinder></geometry>
        <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
"""


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    racine = Path(sys.argv[1]).expanduser().resolve()
    demi = COTE_NOIR * CASES_TOTALES / CASES_NOIRES / 2.0
    for tag_id in TAGS:
        dossier = racine / f'tag{tag_id}'
        dossier.mkdir(parents=True, exist_ok=True)
        # Noms uniques : Gazebo met en cache maillages et textures par nom de
        # fichier ; des noms communs affichaient le même tag partout.
        cv2.imwrite(str(dossier / f'tag{tag_id}.png'), image_tag(tag_id))
        (dossier / f'tag{tag_id}.mtl').write_text(
            f'newmtl tag{tag_id}\nKa 1 1 1\nKd 1 1 1\nmap_Kd tag{tag_id}.png\n')
        (dossier / f'tag{tag_id}.obj').write_text(maillage(tag_id, demi))
        (dossier / 'model.sdf').write_text(modele(tag_id, dossier))
    print(f'Tags {", ".join(map(str, TAGS))} générés dans {racine}')


if __name__ == '__main__':
    main()
