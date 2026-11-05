#!/usr/bin/env python3
"""Gazebo : mesure la qualité d'une carte générée face aux murs réels du café.

    source install/setup.bash
    python3 src/fosa-solution/tools/compare_maps.py ~/fosa_maps/cafe_test

Chaque cellule occupée de la carte générée est ramenée dans le repère du monde
Gazebo (world_from_map, noté par mapping_session avec gazebo:=true), puis on
cherche un mur de la carte de référence (maps/cafe_map.yaml, tirée du monde
officiel) à moins de 10 cm, puis de 20 cm.

La référence ne contient que la structure fixe (murs, comptoir, cuisine) : les
pieds de tables et de chaises vus par le lidar comptent comme « hors mur ». Le
taux n'atteint donc jamais 100 %. Ce qui compte : il doit rester élevé et
stable d'un essai à l'autre. Une carte décalée, tournée ou dédoublée le fait
chuter nettement.
"""

import argparse
import os
import sys

from caytu_nav_solution.map_io import RAW_MAP, load_map_info, load_map_yaml, read_pgm
from caytu_nav_solution.map_io import iter_occupied_points
from caytu_nav_solution.nav_math import Pose2D, charger_carte_pgm


def default_reference() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('caytu_nav_bringup'),
                            'maps', 'cafe_map.yaml')
    except Exception:       # noqa: BLE001 — hors d'un espace ROS sourcé
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(here, '..', 'caytu_nav_bringup', 'maps', 'cafe_map.yaml')


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('folder', help='dossier de la carte générée')
    parser.add_argument('reference', nargs='?', default=None,
                        help='YAML de la carte de référence (défaut : cafe_map.yaml)')
    args = parser.parse_args(argv)

    info = load_map_info(args.folder)
    if info.world_from_map is None:
        print('Cette carte n\'a pas de lien avec le repère Gazebo '
              '(cartographie lancée sans gazebo:=true) : comparaison impossible.',
              file=sys.stderr)
        return 1

    generated = load_map_yaml(os.path.join(info.folder, RAW_MAP + '.yaml'))
    with open(generated['image'], 'rb') as stream:
        width, height, pixels = read_pgm(stream.read())

    ref_yaml = args.reference or default_reference()
    reference = load_map_yaml(ref_yaml)
    with open(reference['image'], 'rb') as stream:
        ref_grid = charger_carte_pgm(stream.read(), reference['resolution'],
                                reference['origin'][0], reference['origin'][1])

    tolerances = (2, 4)                         # cellules de 5 cm : 10 et 20 cm
    hits = [0] * len(tolerances)
    total = 0
    for x, y in iter_occupied_points(width, height, pixels, generated['resolution'],
                                     generated['origin'][0], generated['origin'][1]):
        world = info.world_from_map.composer(Pose2D(x, y, 0.0))
        total += 1
        for i, tol in enumerate(tolerances):
            if ref_grid.est_occupee_pres(world.x, world.y, tol):
                hits[i] += 1

    if total == 0:
        print('La carte générée ne contient aucune cellule occupée.', file=sys.stderr)
        return 1
    print(f'Carte « {info.name} » : zone explorée {info.explored_m[0]:.1f} m x '
          f'{info.explored_m[1]:.1f} m, {total} cellules occupées.')
    print(f'Référence : {os.path.abspath(ref_yaml)}')
    for tol, hit in zip(tolerances, hits):
        print(f'  à moins de {tol * 5} cm d\'un mur de référence : {100.0 * hit / total:5.1f} %')
    return 0


if __name__ == '__main__':
    sys.exit(main())
