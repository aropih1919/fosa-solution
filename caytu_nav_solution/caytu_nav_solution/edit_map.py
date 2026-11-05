#!/usr/bin/env python3
"""Retouche d'une carte générée, sans toucher à la carte brute.

    ros2 run caytu_nav_solution edit_map ~/fosa_maps/arene_dakar \\
        --erase 1.2 0.4 1.9 1.1      # efface une table qui sera déplacée
        --wall -0.3 -0.3 4.8 -0.25   # mur virtuel : bord ouvert de l'arène

Coordonnées en mètres dans le repère `map` (dans RViz : outil « Publish
Point », puis ros2 topic echo /clicked_point). Chaque rectangle est donné par
deux coins opposés x1 y1 x2 y2.

Le résultat est écrit dans nav_map.pgm / nav_map.yaml, que la navigation
utilise ensuite. La carte brute (map.pgm) et le graphe de slam_toolbox ne
changent pas : la localisation garde tous les repères qu'elle a vus. Les
retouches s'ajoutent les unes aux autres ; --reset repart de la carte brute.
"""

import argparse
import os
import sys

from caytu_nav_solution.map_io import (
    META_FILE, NAV_MAP, PGM_FREE, PGM_OCCUPIED, RAW_MAP, load_map_info, load_map_yaml,
    paint_rectangle, read_pgm, save_map_info, write_map_files)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        prog='edit_map', description='Retouche une carte Fosa (résultat : nav_map.yaml).')
    parser.add_argument('folder', help='dossier de la carte (contient map.yaml)')
    parser.add_argument('--erase', nargs=4, type=float, action='append', default=[],
                        metavar=('X1', 'Y1', 'X2', 'Y2'),
                        help='rectangle à rendre libre (meuble mobile)')
    parser.add_argument('--wall', nargs=4, type=float, action='append', default=[],
                        metavar=('X1', 'Y1', 'X2', 'Y2'),
                        help='rectangle à rendre occupé (mur virtuel)')
    parser.add_argument('--reset', action='store_true',
                        help='repartir de la carte brute map.pgm')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    folder = os.path.abspath(os.path.expanduser(args.folder))
    nav_yaml = os.path.join(folder, NAV_MAP + '.yaml')
    raw_yaml = os.path.join(folder, RAW_MAP + '.yaml')
    source = raw_yaml if args.reset or not os.path.isfile(nav_yaml) else nav_yaml
    if not os.path.isfile(source):
        print(f'{source} introuvable.', file=sys.stderr)
        return 1
    if not (args.erase or args.wall or args.reset):
        print('Rien à faire : donnez --erase, --wall ou --reset.', file=sys.stderr)
        return 1

    meta = load_map_yaml(source)
    with open(meta['image'], 'rb') as stream:
        width, height, pixels = read_pgm(stream.read())
    resolution = meta['resolution']
    origin_x, origin_y = meta['origin'][0], meta['origin'][1]

    for rect in args.erase:
        n = paint_rectangle(pixels, width, height, resolution, origin_x, origin_y, rect, PGM_FREE)
        print(f'Effacé  {rect} : {n} cellules rendues libres.')
    for rect in args.wall:
        n = paint_rectangle(pixels, width, height, resolution, origin_x, origin_y, rect,
                            PGM_OCCUPIED)
        print(f'Mur     {rect} : {n} cellules rendues occupées.')

    write_map_files(folder, NAV_MAP, width, height, pixels, resolution, meta['origin'])
    if os.path.isfile(os.path.join(folder, META_FILE)):
        info = load_map_info(folder)
        info.nav_map = NAV_MAP + '.yaml'
        save_map_info(info)
    print(f'Carte de navigation écrite : {nav_yaml} (à partir de {os.path.basename(source)}).')
    return 0


if __name__ == '__main__':
    sys.exit(main())
