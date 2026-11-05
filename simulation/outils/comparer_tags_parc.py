#!/usr/bin/env python3
"""Compare les tags notés (<carte>/tags.yaml) à la vérité du simulateur officiel PARC.

    python3 simulation/outils/comparer_tags_parc.py ~/fosa_maps/parc_seed7 \\
        /tmp/parc_worlds/parc_random_seed7.json

Le simulateur officiel (phase2_hard.launch.py seed:=N) écrit la vérité du monde
dans <world_dir>/parc_random_seed<N>.json : tags, poses de visite, départ. Elle
sert aux essais seulement, jamais à la solution. La carte de slam_toolbox part
de la pose de départ du robot ; on la replace dans le monde avec « start ».

Pour chaque tag : écart de position (m) et écart d'orientation de la face
visible (°). Rend 0 si les quatre tags sont notés à moins de --tolerance.
"""
import argparse
import json
import math
from pathlib import Path
import sys

import yaml


def vers_monde(depart, x, y):
    c, s = math.cos(depart['yaw']), math.sin(depart['yaw'])
    return depart['x'] + c * x - s * y, depart['y'] + s * x + c * y


def ecart_angle(a, b):
    return math.degrees(math.atan2(math.sin(a - b), math.cos(a - b)))


def main():
    lecteur = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    lecteur.add_argument('carte', type=Path, help='dossier de la carte (avec tags.yaml)')
    lecteur.add_argument('monde', type=Path, help='parc_random_seed<N>.json du simulateur')
    lecteur.add_argument('--tolerance', type=float, default=0.30, help='écart admis (m)')
    args = lecteur.parse_args()

    fichier = args.carte / 'tags.yaml' if args.carte.is_dir() else args.carte
    notes = (yaml.safe_load(fichier.read_text(encoding='utf-8')) or {}).get('tags') or {}
    verite = json.loads(args.monde.read_text(encoding='utf-8'))
    depart = verite['start']
    bons = 0
    for tag in sorted(verite['tags'], key=lambda t: t['id']):
        note = notes.get(tag['id']) or notes.get(str(tag['id']))
        if not note:
            print(f'tag {tag["id"]} : jamais noté')
            continue
        x, y = vers_monde(depart, float(note['x']), float(note['y']))
        ecart = math.hypot(x - tag['x'], y - tag['y'])
        face = float(note.get('lacet_normale', 0.0)) + depart['yaw']
        juste = ecart <= args.tolerance
        bons += juste
        print(f'tag {tag["id"]} : {"OK    " if juste else "ÉCART "} {ecart:.2f} m '
              f'(noté ({x:.2f}, {y:.2f}), vrai ({tag["x"]:.2f}, {tag["y"]:.2f})), '
              f'face {ecart_angle(face, tag["facing_yaw"]):+.0f}°, '
              f'{note.get("observations", "?")} vues')
    print(f'RESULTAT : {bons}/{len(verite["tags"])} tags à moins de {args.tolerance:.2f} m')
    sys.exit(0 if bons == len(verite['tags']) else 1)


if __name__ == '__main__':
    main()
