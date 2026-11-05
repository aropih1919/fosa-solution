#!/usr/bin/env python3
"""Tags fictifs pour la simulation : 4 AprilTags 36h11 posés dans Gazebo.

À lancer après la cartographie, Gazebo ouvert (le monde de la cartographie) :

    python3 tools/tags_fictifs.py ~/fosa_maps/cafe_test             # choisit 4 places, fait apparaître les tags
    python3 tools/tags_fictifs.py ~/fosa_maps/cafe_test --apercu    # choisit seulement : image, rien dans Gazebo
    python3 tools/tags_fictifs.py ~/fosa_maps/cafe_test --graine 3  # autre tirage des places
    python3 tools/tags_fictifs.py ~/fosa_maps/cafe_test \\
        --positions "2.0,1.0,180; -1.5,3.0,-90; 0.5,-2.0,90; 3.0,-1.0,0"   # places imposées
    python3 tools/tags_fictifs.py ~/fosa_maps/cafe_test --retirer   # enlève les tags de Gazebo
    python3 tools/tags_fictifs.py ~/fosa_maps/cafe_test --comparer  # tags.yaml du registre vs vérité
    python3 tools/tags_fictifs.py ~/fosa_maps/cafe_test --tags-yaml # écrit aussi <carte>/tags.yaml

Les places sont choisies sur la carte enregistrée : devant un mur droit, avec
de la place libre devant (pose d'attente et pose de visite atteignables depuis
le départ), bien séparées. Chaque tag est un panneau blanc sur un pied, centre
du tag à `--hauteur` m, posé `--recul` m devant le mur : une petite erreur de
la carte ne l'enfonce jamais dans le mur. Le motif est fait de pavés noirs, sans
texture, donc jamais inversé par le rendu.

Positions imposées : `x,y,cap` dans le repère `map` (celui de RViz pendant la
course), cap en degrés = direction vers laquelle la face du tag regarde.

Fichiers écrits dans <carte>/tags_fictifs/ :
    verite.yaml  positions vraies (repères map et monde Gazebo)
    apercu.png   carte, tags (flèche = face), poses de visite (point vert)
    tag_<id>.sdf modèles Gazebo

Il faut : la carte faite dans Gazebo (fosa_map.yaml contient world_from_map),
la commande `gz` (Gazebo Harmonic), python3 avec numpy, opencv et yaml.
"""

import argparse
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys

import cv2
import numpy as np
import yaml

PREFIXE_MODELE = 'fosa_tag_'


# ------------------------------------------------------------------ réglages
def valeurs_par_defaut():
    """Identifiants, taille et distances de caytu_config, sinon celles du guide."""
    defauts = {'ids': [1, 4, 7, 9], 'taille': 0.165, 'visite': 0.5, 'attente': 1.0,
               'fichier_tags': 'tags.yaml', 'source': 'guide (caytu_config introuvable)'}
    try:
        from caytu_config import load_config
        config = load_config()
        defauts.update(ids=list(config.course.expected_ids),
                       taille=config.perception.apriltag.size_m,
                       visite=config.course.visit_distance_m,
                       attente=config.course.staging_distance_m,
                       fichier_tags=config.course.tags_file, source='caytu_config')
    except Exception:
        pass
    return defauts


# ------------------------------------------------------------------ repères
def composer(a, b):
    """a ∘ b, poses planes (x, y, cap)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1],
            math.atan2(math.sin(a[2] + b[2]), math.cos(a[2] + b[2])))


# ------------------------------------------------------------------ carte
class Carte:
    def __init__(self, dossier):
        self.dossier = Path(dossier).expanduser().resolve()
        meta_chemin = self.dossier / 'fosa_map.yaml'
        if not meta_chemin.is_file():
            sys.exit(f'{meta_chemin} introuvable : donner le dossier d\'une carte enregistrée.')
        self.meta = yaml.safe_load(meta_chemin.read_text(encoding='utf-8')) or {}
        nav = self.dossier / (self.meta.get('nav_map') or 'map.yaml')
        if not nav.is_file():
            nav = self.dossier / 'map.yaml'
        info = yaml.safe_load(nav.read_text(encoding='utf-8'))
        image = Path(info['image'])
        if not image.is_absolute():
            image = nav.parent / image
        pixels = cv2.imread(str(image), cv2.IMREAD_UNCHANGED)
        if pixels is None:
            sys.exit(f'Image de carte illisible : {image}')
        if pixels.ndim == 3:
            pixels = cv2.cvtColor(pixels, cv2.COLOR_BGR2GRAY)
        self.pixels = pixels
        self.resolution = float(info['resolution'])
        self.origine = [float(v) for v in info['origin'][:2]]
        p = pixels.astype(np.float32) / 255.0
        if not int(info.get('negate', 0)):
            p = 1.0 - p
        self.occupe = p > float(info.get('occupied_thresh', 0.65))
        self.libre = p < float(info.get('free_thresh', 0.196))
        self.hauteur_px, self.largeur_px = pixels.shape
        depart = self.meta.get('start_pose') or [0.0, 0.0, 0.0]
        self.depart = tuple(float(v) for v in depart[:3])
        monde = self.meta.get('world_from_map')
        self.monde_depuis_carte = tuple(float(v) for v in monde[:3]) if monde else None

    def cellule(self, x, y):
        colonne = int(math.floor((x - self.origine[0]) / self.resolution))
        ligne = self.hauteur_px - 1 - int(math.floor((y - self.origine[1]) / self.resolution))
        return ligne, colonne

    def centre(self, ligne, colonne):
        return (self.origine[0] + (colonne + 0.5) * self.resolution,
                self.origine[1] + (self.hauteur_px - ligne - 0.5) * self.resolution)

    def dedans(self, ligne, colonne):
        return 0 <= ligne < self.hauteur_px and 0 <= colonne < self.largeur_px

    def valeur(self, masque, x, y):
        ligne, colonne = self.cellule(x, y)
        return bool(masque[ligne, colonne]) if self.dedans(ligne, colonne) else False


def zone_atteignable(carte, degagement):
    """Cellules où le centre du robot peut aller, reliées au départ."""
    rayon = max(1, int(math.ceil(degagement / carte.resolution)))
    noyau = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * rayon + 1, 2 * rayon + 1))
    navigable = cv2.erode(carte.libre.astype(np.uint8), noyau)
    _, etiquettes = cv2.connectedComponents(navigable, connectivity=8)
    ligne, colonne = carte.cellule(carte.depart[0], carte.depart[1])
    if not carte.dedans(ligne, colonne) or not navigable[ligne, colonne]:
        lignes, colonnes = np.nonzero(navigable)
        if not len(lignes):
            sys.exit('Aucune zone navigable sur la carte.')
        plus_proche = np.argmin((lignes - ligne) ** 2 + (colonnes - colonne) ** 2)
        ligne, colonne = lignes[plus_proche], colonnes[plus_proche]
    return etiquettes == etiquettes[ligne, colonne]


def candidats_murs(carte, atteignable, args):
    """Places possibles devant un mur droit : (x, y, cap de la face)."""
    res = carte.resolution
    voisin_libre = np.zeros_like(carte.occupe)
    for dl, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        voisin_libre |= np.roll(carte.libre, (dl, dc), axis=(0, 1))
    bord = carte.occupe & voisin_libre
    lignes, colonnes = np.nonzero(bord)
    pas = max(1, int(round(0.10 / res)))
    fenetre = 4
    candidats = []
    for ligne, colonne in zip(lignes[::pas], colonnes[::pas]):
        # Normale : du côté occupé vers le côté libre.
        nx = ny = 0.0
        for dl in range(-fenetre, fenetre + 1):
            for dc in range(-fenetre, fenetre + 1):
                if (dl or dc) and carte.dedans(ligne + dl, colonne + dc):
                    poids = (1.0 if carte.libre[ligne + dl, colonne + dc]
                             else -1.0 if carte.occupe[ligne + dl, colonne + dc] else 0.0)
                    norme = math.hypot(dc, dl)
                    nx += poids * dc / norme
                    ny -= poids * dl / norme        # ligne vers le bas = y vers le bas
        if math.hypot(nx, ny) < 1e-6:
            continue
        cap = math.atan2(ny, nx)
        n = (math.cos(cap), math.sin(cap))
        t = (-n[1], n[0])
        cx, cy = carte.centre(ligne, colonne)
        surface = (cx + n[0] * res / 2, cy + n[1] * res / 2)

        def point(d, lateral):
            return (surface[0] + n[0] * d + t[0] * lateral,
                    surface[1] + n[1] * d + t[1] * lateral)
        # Mur droit sur 30 cm : occupé juste derrière, libre juste devant.
        laterales = np.arange(-0.15, 0.151, res)
        derriere = sum(carte.valeur(carte.occupe, *point(-res, l)) for l in laterales)
        devant = sum(carte.valeur(carte.libre, *point(2 * res, l)) for l in laterales)
        if derriere < 0.8 * len(laterales) or devant < 0.9 * len(laterales):
            continue
        # Place libre devant le tag, jusqu'au-delà de la pose d'attente.
        profondeur = args.recul + args.attente + 0.3
        libre = all(carte.valeur(carte.libre, *point(d, l))
                    for d in np.arange(res, profondeur, res)
                    for l in (-0.25, 0.0, 0.25))
        if not libre:
            continue
        tag = point(args.recul, 0.0)
        visite = point(args.recul + args.visite, 0.0)
        attente = point(args.recul + args.attente, 0.0)
        if not (carte.valeur(atteignable, *visite) and carte.valeur(atteignable, *attente)):
            continue
        if math.hypot(tag[0] - carte.depart[0], tag[1] - carte.depart[1]) < args.distance_depart:
            continue
        candidats.append((float(tag[0]), float(tag[1]), float(cap)))
    return candidats


def choisir(candidats, nombre, carte, args):
    """Places bien séparées : chacune le plus loin possible des précédentes."""
    if len(candidats) < nombre:
        return []
    points = np.array([(x, y) for x, y, _ in candidats])
    rng = np.random.default_rng(args.graine)
    if args.graine:
        premier = int(rng.integers(len(candidats)))
    else:
        premier = int(np.argmax(np.hypot(points[:, 0] - carte.depart[0],
                                         points[:, 1] - carte.depart[1])))
    choisis = [premier]
    distances = np.hypot(*(points - points[premier]).T)
    while len(choisis) < nombre:
        suivant = int(np.argmax(distances))
        if distances[suivant] < args.separation:
            return []
        choisis.append(suivant)
        distances = np.minimum(distances, np.hypot(*(points - points[suivant]).T))
    return [candidats[i] for i in choisis]


def lire_positions(texte):
    places = []
    for morceau in texte.split(';'):
        if not morceau.strip():
            continue
        valeurs = [float(v) for v in morceau.replace(' ', '').split(',')]
        if len(valeurs) != 3:
            sys.exit(f'--positions : « {morceau} » doit être x,y,cap_degres')
        places.append((valeurs[0], valeurs[1], math.radians(valeurs[2])))
    return places


# ------------------------------------------------------------------ modèle
def motif_tag(identifiant):
    """Cellules 8×8 du tag (bord noir compris), True = noir, ligne 0 en haut."""
    dictionnaire = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        image = cv2.aruco.generateImageMarker(dictionnaire, identifiant, 8, borderBits=1)
    else:
        image = cv2.aruco.drawMarker(dictionnaire, identifiant, 8, borderBits=1)
    return image < 128


def paves_noirs(identifiant, taille):
    """Pavés noirs (y, z, largeur, hauteur) dans le plan de la face, centrés sur le tag.

    Vu de face (caméra regardant vers -x du modèle) : +y vers la droite, +z en haut.
    """
    noir = motif_tag(identifiant)
    cote = taille / 8.0
    paves = []
    for ligne in range(8):
        colonne = 0
        while colonne < 8:
            if not noir[ligne, colonne]:
                colonne += 1
                continue
            debut = colonne
            while colonne < 8 and noir[ligne, colonne]:
                colonne += 1
            largeur = (colonne - debut) * cote
            y = (debut - 4.0) * cote + largeur / 2
            z = (4.0 - ligne - 0.5) * cote
            paves.append((y, z, largeur, cote))
    return paves


def sdf_tag(identifiant, args):
    """Modèle statique : pied, panneau blanc, tag en pavés noirs ; face vers +x."""
    h, taille = args.hauteur, args.taille
    panneau = max(0.30, taille * 10 / 8 + 0.06)
    blanc = ('<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>'
             '<specular>0 0 0 1</specular><emissive>0.35 0.35 0.35 1</emissive></material>')
    noir = ('<material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse>'
            '<specular>0 0 0 1</specular><emissive>0 0 0 1</emissive></material>')
    gris = ('<material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse>'
            '<specular>0.1 0.1 0.1 1</specular></material>')

    def boite(nom, pose, taille_boite, materiau, collision=True):
        geometrie = f'<geometry><box><size>{taille_boite}</size></box></geometry>'
        texte = f'<visual name="{nom}"><pose>{pose}</pose>{geometrie}{materiau}</visual>'
        if collision:
            texte += f'<collision name="{nom}"><pose>{pose}</pose>{geometrie}</collision>'
        return texte

    elements = [
        boite('socle', '-0.03 0 0.01 0 0 0', '0.25 0.25 0.02', gris),
        boite('pied', f'-0.03 0 {(h - panneau / 2) / 2:.4f} 0 0 0',
              f'0.04 0.04 {h - panneau / 2:.4f}', gris),
        boite('panneau', f'-0.006 0 {h:.4f} 0 0 0', f'0.012 {panneau:.3f} {panneau:.3f}', blanc),
    ]
    for i, (y, z, largeur, hauteur) in enumerate(paves_noirs(identifiant, taille)):
        # 0,2 mm de recouvrement : pas de fente blanche entre deux pavés.
        elements.append(boite(f'noir_{i}', f'0.001 {y:.5f} {h + z:.5f} 0 0 0',
                              f'0.002 {largeur + 0.0002:.5f} {hauteur + 0.0002:.5f}',
                              noir, collision=False))
    return ('<?xml version="1.0"?>\n<sdf version="1.9">\n'
            f'<model name="{PREFIXE_MODELE}{identifiant}"><static>true</static>\n'
            '<link name="lien">\n' + '\n'.join(elements) + '\n</link></model>\n</sdf>\n')


# ------------------------------------------------------------------ Gazebo
def gz(*arguments, delai=10):
    try:
        return subprocess.run(['gz', *arguments], capture_output=True, text=True,
                              timeout=delai)
    except FileNotFoundError:
        sys.exit('Commande `gz` introuvable : Gazebo Harmonic doit être installé ici.')
    except subprocess.TimeoutExpired:
        return subprocess.CompletedProcess(arguments, 1, '', 'délai dépassé')


def trouver_monde(nom):
    if nom:
        return nom
    services = gz('service', '-l').stdout
    mondes = re.findall(r'^/world/([^/\s]+)/create$', services, re.MULTILINE)
    if not mondes:
        sys.exit('Aucun monde Gazebo trouvé : lancer d\'abord la simulation '
                 '(même GZ_PARTITION que ce terminal).')
    if len(mondes) > 1:
        print(f'Plusieurs mondes : {mondes} ; « {mondes[0]} » utilisé (--monde pour choisir).')
    return mondes[0]


def retirer(monde, identifiants):
    for identifiant in identifiants:
        gz('service', '-s', f'/world/{monde}/remove', '--reqtype', 'gz.msgs.Entity',
           '--reptype', 'gz.msgs.Boolean', '--timeout', '3000',
           '--req', f'name: "{PREFIXE_MODELE}{identifiant}" type: MODEL')


def faire_apparaitre(monde, identifiant, fichier, pose_monde):
    x, y, cap = pose_monde
    requete = (f'sdf_filename: "{fichier}" name: "{PREFIXE_MODELE}{identifiant}" '
               f'pose: {{position: {{x: {x:.4f}, y: {y:.4f}, z: 0}} '
               f'orientation: {{x: 0, y: 0, z: {math.sin(cap / 2):.6f}, w: {math.cos(cap / 2):.6f}}}}}')
    sortie = gz('service', '-s', f'/world/{monde}/create', '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', requete)
    return 'data: true' in sortie.stdout


# ------------------------------------------------------------------ fichiers
def ecrire_apercu(carte, tags, args, chemin):
    image = cv2.cvtColor(carte.pixels, cv2.COLOR_GRAY2BGR)
    echelle = max(1, int(round(600 / max(image.shape[:2]))))
    image = cv2.resize(image, None, fx=echelle, fy=echelle, interpolation=cv2.INTER_NEAREST)

    def px(x, y):
        ligne, colonne = carte.cellule(x, y)
        return int((colonne + 0.5) * echelle), int((ligne + 0.5) * echelle)
    for tag in tags:
        x, y, cap = tag['map']
        n = (math.cos(cap), math.sin(cap))
        visite = (x + n[0] * args.visite, y + n[1] * args.visite)
        cv2.arrowedLine(image, px(x, y), px(x + n[0] * 0.6, y + n[1] * 0.6), (0, 0, 255), 2,
                        tipLength=0.35)
        cv2.circle(image, px(x, y), 5, (0, 0, 255), -1)
        cv2.circle(image, px(*visite), 4, (0, 160, 0), -1)
        cv2.putText(image, str(tag['id']), px(x - 0.25, y + 0.25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (200, 0, 0), 2)
    cv2.drawMarker(image, px(carte.depart[0], carte.depart[1]), (255, 0, 255),
                   cv2.MARKER_TILTED_CROSS, 14, 2)
    cv2.imwrite(str(chemin), image)


def ecrire_tags_yaml(carte, tags, args, defauts):
    chemin = carte.dossier / defauts['fichier_tags']
    if chemin.exists():
        shutil.copy2(chemin, chemin.with_suffix(chemin.suffix + '.avant_fictifs'))
    contenu = {'repere': 'map', 'tags': {
        tag['id']: {'x': round(tag['map'][0], 4), 'y': round(tag['map'][1], 4),
                    'z': round(args.hauteur, 4), 'lacet_normale': round(tag['map'][2], 4),
                    'observations': 1}
        for tag in tags}}
    chemin.write_text('# Tags FICTIFS (tools/tags_fictifs.py) : positions exactes.\n'
                      + yaml.safe_dump(contenu, sort_keys=True), encoding='utf-8')
    print(f'{chemin} écrit (ancienne version : {chemin.name}.avant_fictifs).')


def comparer(carte, defauts):
    verite_chemin = carte.dossier / 'tags_fictifs' / 'verite.yaml'
    registre_chemin = carte.dossier / defauts['fichier_tags']
    if not verite_chemin.is_file() or not registre_chemin.is_file():
        sys.exit(f'Il faut {verite_chemin} et {registre_chemin}.')
    verite = yaml.safe_load(verite_chemin.read_text(encoding='utf-8'))['tags']
    registre = (yaml.safe_load(registre_chemin.read_text(encoding='utf-8')) or {}).get('tags') or {}
    print(f'{"tag":>4} {"écart position":>15} {"écart normale":>14} {"vues":>5}')
    for identifiant, vrai in sorted(verite.items()):
        note = registre.get(identifiant) or registre.get(str(identifiant))
        if not note:
            print(f'{identifiant:>4} {"jamais vu":>15}')
            continue
        ecart = math.hypot(note['x'] - vrai['map_x'], note['y'] - vrai['map_y'])
        angle = math.degrees(math.atan2(math.sin(note['lacet_normale'] - vrai['map_cap']),
                                        math.cos(note['lacet_normale'] - vrai['map_cap'])))
        print(f'{identifiant:>4} {ecart:>13.2f} m {angle:>12.1f} ° {note.get("observations", "?"):>5}')


# ------------------------------------------------------------------ principal
def main():
    defauts = valeurs_par_defaut()
    lecteur = argparse.ArgumentParser(
        description='Tags AprilTag fictifs dans Gazebo, placés sur une carte enregistrée.')
    lecteur.add_argument('carte', help='dossier de la carte (contient fosa_map.yaml)')
    lecteur.add_argument('--ids', default=','.join(map(str, defauts['ids'])))
    lecteur.add_argument('--positions', help='« x,y,cap_deg; ... » dans le repère map')
    lecteur.add_argument('--graine', type=int, default=0, help='autre tirage des places (0 : déterministe)')
    lecteur.add_argument('--hauteur', type=float, default=1.0, help='centre du tag au-dessus du sol (m)')
    lecteur.add_argument('--taille', type=float, default=defauts['taille'],
                         help='côté du carré noir (m), comme perception.apriltag.size_m')
    lecteur.add_argument('--recul', type=float, default=0.20, help='distance mur -> tag (m)')
    lecteur.add_argument('--separation', type=float, default=1.5, help='écart minimal entre tags (m)')
    lecteur.add_argument('--distance-depart', type=float, default=1.0,
                         help='distance minimale départ -> tag (m)')
    lecteur.add_argument('--degagement', type=float, default=0.35,
                         help='rayon du robot pour juger une pose atteignable (m)')
    lecteur.add_argument('--monde', help='nom du monde Gazebo (par défaut : le seul ouvert)')
    lecteur.add_argument('--apercu', action='store_true', help='choisir seulement, rien dans Gazebo')
    lecteur.add_argument('--retirer', action='store_true', help='enlever les tags de Gazebo')
    lecteur.add_argument('--comparer', action='store_true',
                         help='comparer <carte>/tags.yaml (registre) à la vérité')
    lecteur.add_argument('--tags-yaml', action='store_true',
                         help='écrire aussi <carte>/tags.yaml avec les positions exactes')
    args = lecteur.parse_args()
    args.visite, args.attente = defauts['visite'], defauts['attente']
    identifiants = [int(v) for v in args.ids.split(',') if v.strip()]

    carte = Carte(args.carte)
    if args.comparer:
        comparer(carte, defauts)
        return
    if args.retirer:
        monde = trouver_monde(args.monde)
        retirer(monde, identifiants)
        print(f'Tags {identifiants} retirés du monde « {monde} ».')
        return
    if carte.monde_depuis_carte is None:
        sys.exit('fosa_map.yaml n\'a pas de world_from_map : la carte n\'a pas été faite dans '
                 'Gazebo, impossible de placer les tags dans le monde.')

    if args.positions:
        places = lire_positions(args.positions)
        if len(places) != len(identifiants):
            sys.exit(f'--positions : {len(places)} places pour {len(identifiants)} tags.')
    else:
        atteignable = zone_atteignable(carte, args.degagement)
        candidats = candidats_murs(carte, atteignable, args)
        places = choisir(candidats, len(identifiants), carte, args)
        if not places:
            sys.exit(f'{len(candidats)} places possibles seulement, pas assez séparées pour '
                     f'{len(identifiants)} tags : essayer --separation 1.0, --graine 1, ou --positions.')

    dossier = carte.dossier / 'tags_fictifs'
    dossier.mkdir(exist_ok=True)
    tags = []
    for identifiant, (x, y, cap) in zip(identifiants, places):
        monde_x, monde_y, monde_cap = composer(carte.monde_depuis_carte, (x, y, cap))
        fichier = dossier / f'tag_{identifiant}.sdf'
        fichier.write_text(sdf_tag(identifiant, args), encoding='utf-8')
        tags.append({'id': identifiant, 'map': (x, y, cap),
                     'monde': (monde_x, monde_y, monde_cap), 'fichier': fichier})

    verite = {'carte': str(carte.dossier), 'hauteur_m': args.hauteur, 'taille_m': args.taille,
              'tags': {tag['id']: {
                  'map_x': round(tag['map'][0], 4), 'map_y': round(tag['map'][1], 4),
                  'map_cap': round(tag['map'][2], 4),
                  'monde_x': round(tag['monde'][0], 4), 'monde_y': round(tag['monde'][1], 4),
                  'monde_cap': round(tag['monde'][2], 4)} for tag in tags}}
    (dossier / 'verite.yaml').write_text(
        '# Positions vraies des tags fictifs. map_cap / monde_cap : direction de la face (rad).\n'
        + yaml.safe_dump(verite, sort_keys=False), encoding='utf-8')
    ecrire_apercu(carte, tags, args, dossier / 'apercu.png')

    print(f'{"tag":>4} {"map x":>7} {"map y":>7} {"face":>6}   {"monde x":>8} {"monde y":>8}')
    for tag in tags:
        print(f'{tag["id"]:>4} {tag["map"][0]:7.2f} {tag["map"][1]:7.2f} '
              f'{math.degrees(tag["map"][2]):5.0f}°   {tag["monde"][0]:8.2f} {tag["monde"][1]:8.2f}')
    print(f'Aperçu : {dossier / "apercu.png"} ; vérité : {dossier / "verite.yaml"}')
    print(f'Tag : carré noir de {args.taille:.3f} m, centre à {args.hauteur:.2f} m du sol '
          f'(taille par défaut : {defauts["source"]}).')
    if defauts['source'] != 'caytu_config' and '--taille' not in sys.argv:
        print('ATTENTION : sourcer le workspace (install/setup.bash) pour que la taille soit '
              'celle du détecteur (perception.apriltag.size_m) ; sinon les distances seront fausses.')
    if args.tags_yaml:
        ecrire_tags_yaml(carte, tags, args, defauts)
    if args.apercu:
        return

    monde = trouver_monde(args.monde)
    retirer(monde, identifiants)            # relancer le script remplace les anciens tags
    echecs = [tag['id'] for tag in tags
              if not faire_apparaitre(monde, tag['id'], tag['fichier'], tag['monde'])]
    if echecs:
        sys.exit(f'Gazebo a refusé les tags {echecs} (monde « {monde} »).')
    print(f'{len(tags)} tags posés dans le monde « {monde} ».')


if __name__ == '__main__':
    main()
