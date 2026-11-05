"""Tests de la cartographie (fichiers de carte, métadonnées, repères), sans ROS."""
import datetime
import math
import os

import pytest
import yaml

from caytu_nav_solution import edit_map
from caytu_nav_solution.map_io import (
    META_FILE, PGM_FREE, PGM_OCCUPIED, PGM_UNKNOWN, MapInfo, check_map_name,
    iter_occupied_points, load_map_info, load_map_yaml, occupancy_to_pixels,
    paint_rectangle, pixel_stats, prepare_map_folder, read_pgm, save_map_info,
    world_from_map_at_start, world_point_in_map, write_map_files)
from caytu_nav_solution.nav_math import Pose2D, charger_carte_pgm

SPAWN = Pose2D(-0.200546, -7.485170, 1.571)        # task_params.yaml officiel
GOAL = (-2.293350, 2.232090)


def _grid(width, height, fill=-1):
    return [fill] * (width * height)


def test_occupancy_to_pixels_flips_rows_and_applies_map_saver_thresholds():
    width, height = 3, 2
    # Ligne 0 de l'OccupancyGrid = y minimal.
    data = [100, 0, -1,
            65, 25, 50]
    pixels = occupancy_to_pixels(width, height, data)
    # Ligne 0 du PGM = y maximal : la ligne 1 de la grille passe en premier.
    assert list(pixels) == [PGM_OCCUPIED, PGM_FREE, PGM_UNKNOWN,
                            PGM_OCCUPIED, PGM_FREE, PGM_UNKNOWN]
    with pytest.raises(ValueError):
        occupancy_to_pixels(2, 2, data)


def test_written_map_reads_back_at_the_right_place(tmp_path):
    width, height, res = 40, 30, 0.05
    origin = (-1.0, 2.0, 0.0)
    data = _grid(width, height, 0)
    data[7 * width + 12] = 100                    # cellule occupée en (12, 7)
    yaml_path = write_map_files(str(tmp_path), 'map', width, height,
                                occupancy_to_pixels(width, height, data), res, origin)
    meta = load_map_yaml(yaml_path)
    assert meta['resolution'] == pytest.approx(res)
    assert meta['origin'] == pytest.approx(list(origin))
    with open(meta['image'], 'rb') as stream:
        grid = charger_carte_pgm(stream.read(), res, origin[0], origin[1])
    x = origin[0] + 12.5 * res
    y = origin[1] + 7.5 * res
    assert grid.est_occupee_pres(x, y, 0)
    assert not grid.est_occupee_pres(x + 3 * res, y, 1)
    # Le YAML suit le format de nav2_map_server, avec un seuil qui garde 205 inconnu.
    text = yaml.safe_load(open(yaml_path, encoding='utf-8'))
    assert text['mode'] == 'trinary' and text['image'] == 'map.pgm'
    assert (255 - PGM_UNKNOWN) / 255.0 >= text['free_thresh']
    assert (255 - PGM_FREE) / 255.0 < text['free_thresh']
    # Aucun fichier temporaire ne traîne.
    assert sorted(os.listdir(tmp_path)) == ['map.pgm', 'map.yaml']


def test_read_pgm_rejects_bad_files():
    with pytest.raises(ValueError):
        read_pgm(b'P2\n2 2\n255\n0 0 0 0')
    with pytest.raises(ValueError):
        read_pgm(b'P5\n4 4\n255\n\x00\x00')


def test_pixel_stats_measures_explored_area():
    width, height, res = 100, 80, 0.05
    data = _grid(width, height)
    for row in range(10, 30):                     # 20 lignes = 1,0 m
        for col in range(5, 45):                  # 40 colonnes = 2,0 m
            data[row * width + col] = 0
    data[15 * width + 20] = 100
    stats = pixel_stats(width, height, occupancy_to_pixels(width, height, data), res)
    assert stats.known_cells == 800
    assert stats.occupied_cells == 1
    assert stats.known_width_m == pytest.approx(2.0)
    assert stats.known_height_m == pytest.approx(1.0)
    empty = pixel_stats(4, 4, bytes([PGM_UNKNOWN]) * 16, res)
    assert empty.known_cells == 0 and empty.known_width_m == 0.0


def test_prepare_map_folder_never_overwrites_a_map(tmp_path):
    folder, moved = prepare_map_folder(str(tmp_path), 'arene')
    assert moved is None and os.path.isdir(folder)
    open(os.path.join(folder, 'map.pgm'), 'wb').write(b'ancienne')
    now = datetime.datetime(2026, 11, 3, 18, 0, 0)
    folder2, moved = prepare_map_folder(str(tmp_path), 'arene', now=now)
    assert folder2 == folder and os.listdir(folder2) == []
    assert moved.endswith('arene_ancienne_20261103-180000')
    assert open(os.path.join(moved, 'map.pgm'), 'rb').read() == b'ancienne'
    # Même seconde : un suffixe évite toute collision.
    open(os.path.join(folder, 'map.pgm'), 'wb').write(b'deuxieme')
    _, moved2 = prepare_map_folder(str(tmp_path), 'arene', now=now)
    assert moved2 != moved and os.path.isdir(moved2)


@pytest.mark.parametrize('name', ['', '  ', '.', '..', 'a/b', '../x'])
def test_map_names_cannot_escape_the_maps_folder(name):
    with pytest.raises(ValueError):
        check_map_name(name)


def test_map_info_round_trip_and_missing_files(tmp_path):
    folder = str(tmp_path)
    write_map_files(folder, 'map', 2, 2, bytes([PGM_FREE]) * 4, 0.05, (0.0, 0.0, 0.0))
    info = MapInfo(folder=folder, name='cafe_test', created='2026-11-03T18:00:00+03:00',
                   saved_by='manuel', explored_m=(9.3, 21.7),
                   start_pose=Pose2D(0.01, -0.02, 0.003),
                   world_from_map=SPAWN)
    save_map_info(info)
    raw = yaml.safe_load(open(os.path.join(folder, META_FILE), encoding='utf-8'))
    assert all(isinstance(v, float) for v in raw['start_pose'])
    back = load_map_info(folder)
    assert back.name == 'cafe_test' and back.nav_map_path == os.path.join(folder, 'map.yaml')
    assert back.start_pose.x == pytest.approx(0.01)
    assert back.world_from_map.yaw == pytest.approx(SPAWN.yaw, abs=1e-4)
    assert not back.has_posegraph()
    for ext in ('.posegraph', '.data'):
        open(back.posegraph_prefix + ext, 'wb').write(b'x')
    assert back.has_posegraph()
    # Carte hors Gazebo : pas de lien avec le monde.
    info.world_from_map = None
    save_map_info(info)
    assert load_map_info(folder).world_from_map is None
    with pytest.raises(FileNotFoundError):
        load_map_info(str(tmp_path / 'absent'))


def test_gazebo_goal_conversion_follows_the_mapping_start():
    # slam_toolbox démarre avec map = odom ; dans Gazebo, odom = spawn.
    world_from_map = world_from_map_at_start(SPAWN, Pose2D(0.0, 0.0, 0.0))
    assert (world_from_map.x, world_from_map.y) == pytest.approx((SPAWN.x, SPAWN.y))
    gx, gy = world_point_in_map(world_from_map, *GOAL)
    # Le robot regarde vers +y du monde au spawn : le but, 9,7 m plus au nord et
    # 2,1 m à l'ouest, est donc devant lui (x) et à sa gauche (y).
    assert gx == pytest.approx(9.717, abs=0.01)
    assert gy == pytest.approx(2.092, abs=0.01)
    back = world_from_map.composer(Pose2D(gx, gy, 0.0))
    assert (back.x, back.y) == pytest.approx(GOAL)
    # Une première correction de slam_toolbox (map -> odom non nul) est prise
    # en compte : le but reste au même endroit physique.
    map_from_odom = Pose2D(0.05, -0.03, 0.02)
    corrected = world_from_map_at_start(SPAWN, map_from_odom)
    gx2, gy2 = world_point_in_map(corrected, *GOAL)
    world_again = corrected.composer(Pose2D(gx2, gy2, 0.0))
    assert (world_again.x, world_again.y) == pytest.approx(GOAL)
    odom_goal = map_from_odom.inverse().composer(Pose2D(gx2, gy2, 0.0))
    assert (odom_goal.x, odom_goal.y) == pytest.approx((gx, gy))


def test_paint_rectangle_uses_map_coordinates():
    width, height, res = 20, 10, 0.1
    pixels = bytearray([PGM_FREE]) * (width * height)
    n = paint_rectangle(pixels, width, height, res, -1.0, 0.0,
                        (-0.95, 0.05, -0.75, 0.25), PGM_OCCUPIED)
    assert n == 9                                  # colonnes 0-2, lignes 0-2
    points = sorted(iter_occupied_points(width, height, pixels, res, -1.0, 0.0))
    assert points[0] == pytest.approx((-0.95, 0.05))
    assert points[-1] == pytest.approx((-0.75, 0.25))
    # Coins inversés et débordement : rognés à la carte, rien ne casse.
    n = paint_rectangle(pixels, width, height, res, -1.0, 0.0,
                        (5.0, 5.0, -5.0, -5.0), PGM_FREE)
    assert n == 9 and PGM_OCCUPIED not in pixels


def test_edit_map_writes_nav_map_and_keeps_raw_map(tmp_path, capsys):
    folder = str(tmp_path)
    width, height, res = 40, 40, 0.05
    data = _grid(width, height, 0)
    for col in range(10, 14):                     # « table » de 20 cm en y = 1,0 m
        for row in range(20, 24):
            data[row * width + col] = 100
    write_map_files(folder, 'map', width, height, occupancy_to_pixels(width, height, data),
                    res, (0.0, 0.0, 0.0))
    save_map_info(MapInfo(folder=folder, name='t'))
    raw_before = open(os.path.join(folder, 'map.pgm'), 'rb').read()

    assert edit_map.main([folder, '--erase', '0.45', '0.95', '0.75', '1.25',
                          '--wall', '0.0', '1.9', '1.95', '1.95']) == 0
    assert open(os.path.join(folder, 'map.pgm'), 'rb').read() == raw_before
    info = load_map_info(folder)
    assert info.nav_map == 'nav_map.yaml'
    with open(os.path.join(folder, 'nav_map.pgm'), 'rb') as stream:
        grid = charger_carte_pgm(stream.read(), res, 0.0, 0.0)
    assert not grid.est_occupee_pres(0.6, 1.1, 1)  # table effacée
    assert grid.est_occupee_pres(1.0, 1.92, 0)     # mur virtuel ajouté

    # Les retouches s'ajoutent ; --reset repart de la carte brute.
    assert edit_map.main([folder, '--wall', '0.0', '0.0', '0.1', '0.1']) == 0
    with open(os.path.join(folder, 'nav_map.pgm'), 'rb') as stream:
        grid = charger_carte_pgm(stream.read(), res, 0.0, 0.0)
    assert grid.est_occupee_pres(1.0, 1.92, 0) and grid.est_occupee_pres(0.05, 0.05, 0)
    assert edit_map.main([folder, '--reset']) == 0
    with open(os.path.join(folder, 'nav_map.pgm'), 'rb') as stream:
        assert stream.read() == raw_before
    assert edit_map.main([folder]) == 1            # rien demandé
    capsys.readouterr()


def test_occupied_points_are_cell_centres():
    pixels = bytearray([PGM_FREE]) * 6
    pixels[0] = PGM_OCCUPIED                       # PGM ligne 0 = y maximal
    points = list(iter_occupied_points(3, 2, pixels, 1.0, 10.0, 20.0))
    assert points == [(10.5, 21.5)]
    assert all(math.isfinite(v) for p in points for v in p)
