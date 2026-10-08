"""Tests des calculs purs : s'exécutent avec `pytest`, sans ROS ni Gazebo."""
import math
import random

import pytest

import numpy as np

from caytu_nav_solution.nav_math import (
    GRAVITY, GridMap, OdomImuFusion, Pose2D, build_distance_field, cell_cost, field_lookup,
    footprint_gap,
    floor_ranges, lidar_height, load_pgm_map, nearest_free_cell, normalize_angle,
    quaternion_from_rpy, quaternion_from_yaw, scan_map_agreement, split_floor_returns,
    tilt_from_accel, yaw_from_quaternion)

SPAWN = Pose2D(-0.200546, -7.485170, 1.571)        # task_params.yaml officiel
GOAL = (-2.293350, 2.232090)


def test_pose_compose_and_inverse_are_consistent():
    a = Pose2D(1.0, -2.0, 0.7)
    b = Pose2D(0.3, 0.4, -1.2)
    identity = a.compose(a.inverse())
    assert identity.x == pytest.approx(0.0, abs=1e-12)
    assert identity.y == pytest.approx(0.0, abs=1e-12)
    assert identity.yaw == pytest.approx(0.0, abs=1e-12)
    back = a.inverse().compose(a.compose(b))
    assert (back.x, back.y, back.yaw) == pytest.approx((b.x, b.y, b.yaw))


def test_quaternion_helpers_round_trip():
    for yaw in (-3.0, -1.0, 0.0, 0.5, 3.1):
        assert yaw_from_quaternion(*quaternion_from_yaw(yaw)) == pytest.approx(yaw)
    # Le lacet reste lisible quand le robot penche (cas réel : 2,5° de tangage).
    assert yaw_from_quaternion(*quaternion_from_rpy(0.01, 0.043, 1.2)) == pytest.approx(1.2)


def test_first_update_places_map_origin_on_gazebo_frame():
    fusion = OdomImuFusion(SPAWN)
    map_to_odom = fusion.update(Pose2D(0.0, 0.0, 0.0), imu_yaw=0.0)
    # L'origine de odom est le spawn : map devient le repère Gazebo.
    assert (map_to_odom.x, map_to_odom.y, map_to_odom.yaw) == pytest.approx(
        (SPAWN.x, SPAWN.y, SPAWN.yaw))
    assert (fusion.pose.x, fusion.pose.y) == pytest.approx((SPAWN.x, SPAWN.y))


def _drive(fusion, segments, yaw_gain, use_imu, noise=0.0, seed=1, axle=0.0):
    """Simule un robot réel et son odométrie dont le cap est multiplié par yaw_gain.

    segments : liste de (distance, rotation) parcourus par petits pas.
    axle     : distance réelle entre base_footprint et l'essieu, devant lui. Les
               roues déplacent l'essieu ; base_footprint suit, `axle` derrière.
    Retourne (pose vraie de base_footprint dans map, pose estimée).
    """
    rng = random.Random(seed)
    true = SPAWN.compose(Pose2D(axle, 0.0, 0.0))   # milieu de l'essieu dans Gazebo
    odom = Pose2D()                   # ce que publie le plugin DiffDrive
    true_yaw_rel = 0.0                # ce que mesure l'IMU (relatif au départ)
    fusion.update(odom, 0.0 if use_imu else None)
    for distance, rotation in segments:
        steps = 200
        for _ in range(steps):
            ds, dth = distance / steps, rotation / steps
            true = Pose2D(true.x + ds * math.cos(true.yaw + dth / 2),
                          true.y + ds * math.sin(true.yaw + dth / 2),
                          normalize_angle(true.yaw + dth))
            true_yaw_rel += dth
            dth_odom = dth * yaw_gain
            odom = Pose2D(odom.x + ds * math.cos(odom.yaw + dth_odom / 2),
                          odom.y + ds * math.sin(odom.yaw + dth_odom / 2),
                          normalize_angle(odom.yaw + dth_odom))
            imu = normalize_angle(true_yaw_rel + rng.gauss(0.0, noise)) if use_imu else None
            map_to_odom = fusion.update(odom, imu)
            # La TF publiée doit toujours redonner la pose estimée.
            via_tf = map_to_odom.compose(odom)
            assert (via_tf.x, via_tf.y) == pytest.approx((fusion.pose.x, fusion.pose.y), abs=1e-9)
    return true.compose(Pose2D(-axle, 0.0, 0.0)), fusion.pose


SLALOM = [(2.0, 0.0), (0.0, 0.8), (1.5, 0.0), (0.0, -1.6), (2.0, 0.0),
          (0.0, 0.8), (2.5, 0.0), (1.0, 0.6), (1.5, -0.6)]


def test_imu_heading_cancels_the_odometry_rotation_error():
    # Entraxe déclaré 0.3409 m pour 0.393 m réels : cap odométrique x1,153.
    gain = 0.39298 / 0.3409
    true, est = _drive(OdomImuFusion(SPAWN), SLALOM, gain, use_imu=True, noise=2e-4)
    assert math.hypot(est.x - true.x, est.y - true.y) < 0.02
    assert abs(normalize_angle(est.yaw - true.yaw)) < 0.005


def test_odometry_heading_alone_would_miss_the_goal_circle():
    gain = 0.39298 / 0.3409
    true, est = _drive(OdomImuFusion(SPAWN), SLALOM, gain, use_imu=False)
    # Sans IMU, l'erreur dépasse la tolérance d'arrivée : d'où le choix de l'IMU.
    assert math.hypot(est.x - true.x, est.y - true.y) > 0.25


# Essieu 9,5 cm devant base_footprint (URDF officiel : roues en y = -0.095 de
# base_link, lui-même tourné de 90° par rapport à base_footprint).
AXLE = 0.095


def test_axle_offset_keeps_the_estimate_on_base_footprint_through_turns():
    gain = 0.39298 / 0.3409
    true, est = _drive(OdomImuFusion(SPAWN, axle_offset=AXLE), SLALOM, gain,
                       use_imu=True, noise=2e-4, axle=AXLE)
    assert math.hypot(est.x - true.x, est.y - true.y) < 0.02


def test_ignoring_the_axle_offset_shifts_the_estimate_after_a_turn():
    # Demi-tour sur place : l'essieu ne bouge pas, base_footprint passe de
    # l'autre côté. Sans correction, l'erreur vaut deux fois le bras de levier.
    half_turn = [(0.0, math.pi)]
    true, est = _drive(OdomImuFusion(SPAWN), half_turn, 1.0, use_imu=True, axle=AXLE)
    assert math.hypot(est.x - true.x, est.y - true.y) == pytest.approx(2 * AXLE, abs=1e-3)
    true, est = _drive(OdomImuFusion(SPAWN, axle_offset=AXLE), half_turn, 1.0,
                       use_imu=True, axle=AXLE)
    assert math.hypot(est.x - true.x, est.y - true.y) < 1e-3


def test_axle_offset_changes_nothing_while_the_heading_is_unchanged():
    straight = [(3.0, 0.0)]
    _, plain = _drive(OdomImuFusion(SPAWN), straight, 1.0, use_imu=True)
    _, with_axle = _drive(OdomImuFusion(SPAWN, axle_offset=AXLE), straight, 1.0,
                          use_imu=True, axle=AXLE)
    assert (with_axle.x, with_axle.y) == pytest.approx((plain.x, plain.y), abs=1e-9)


def test_first_update_with_axle_offset_still_starts_on_the_spawn():
    fusion = OdomImuFusion(SPAWN, axle_offset=AXLE)
    map_to_odom = fusion.update(Pose2D(0.0, 0.0, 0.0), imu_yaw=0.0)
    assert (map_to_odom.x, map_to_odom.y, map_to_odom.yaw) == pytest.approx(
        (SPAWN.x, SPAWN.y, SPAWN.yaw))
    assert (fusion.pose.x, fusion.pose.y) == pytest.approx((SPAWN.x, SPAWN.y))


def test_fusion_falls_back_then_recovers_without_jump():
    fusion = OdomImuFusion(SPAWN)
    fusion.update(Pose2D(0.0, 0.0, 0.0), 0.0)
    fusion.update(Pose2D(0.5, 0.0, 0.0), 0.0)
    assert fusion.uses_imu
    before = fusion.pose
    fusion.update(Pose2D(0.5, 0.0, 0.1), None)        # IMU muette : cap odométrique
    assert not fusion.uses_imu
    assert fusion.pose.yaw == pytest.approx(normalize_angle(before.yaw + 0.1))
    fusion.update(Pose2D(0.5, 0.0, 0.1), 2.5)         # IMU de retour, autre référence
    assert fusion.uses_imu
    assert fusion.pose.yaw == pytest.approx(normalize_angle(before.yaw + 0.1))


def test_odometry_jump_is_not_integrated_as_motion():
    fusion = OdomImuFusion(SPAWN)
    fusion.update(Pose2D(0.0, 0.0, 0.0), 0.0)
    fusion.update(Pose2D(5.0, 5.0, 0.0), 0.0)          # réinitialisation du simulateur
    assert (fusion.pose.x, fusion.pose.y) == pytest.approx((SPAWN.x, SPAWN.y))


def test_tilt_from_accelerometer_matches_robot_geometry():
    pitch = math.atan(0.0098 / 0.228)                  # roulette 9,8 mm plus basse
    ax, ay, az = -GRAVITY * math.sin(pitch), 0.0, GRAVITY * math.cos(pitch)
    roll, measured = tilt_from_accel(ax, ay, az)
    assert measured == pytest.approx(pitch)
    assert roll == pytest.approx(0.0)
    assert math.degrees(pitch) == pytest.approx(2.46, abs=0.01)


def test_floor_line_is_straight_across_the_front():
    height = lidar_height(math.radians(3.5), 0.0481, 0.116)
    plane_pitch = math.radians(3.5) - 0.04363323129985824      # 1° vers le bas
    angles = [math.radians(a) for a in (-40, -20, 0, 20, 40, 120, 180)]
    floor = floor_ranges(angles, plane_pitch, 0.0, height)
    forward = [r * math.cos(a) for r, a in zip(floor[:5], angles[:5])]
    # À l'avant, le sol est une ligne droite perpendiculaire au cap...
    assert max(forward) - min(forward) < 1e-9
    assert forward[0] == pytest.approx(height / math.sin(plane_pitch))
    # ...et les rayons arrière, qui montent, ne touchent jamais le sol.
    assert floor[5] == math.inf and floor[6] == math.inf


def test_level_lidar_never_reports_floor():
    floor = floor_ranges([0.0, 1.0, 2.0, 3.0], 0.0, 0.0, 0.05)
    assert all(r == math.inf for r in floor)


def test_floor_returns_are_removed_but_obstacles_are_kept():
    floor = [3.0, 3.0, 3.0, math.inf]
    ranges = [3.02, 1.20, math.inf, 5.0]               # sol, obstacle, vide, mur arrière
    mark, clear, count = split_floor_returns(ranges, floor, range_max=12.0)
    assert count == 1
    assert mark == [math.inf, 1.20, math.inf, 5.0]
    assert clear[0] == pytest.approx(2.1)              # libre jusqu'à 70 % du sol
    assert clear[1:] == [1.20, math.inf, 5.0]


def _square_room_pgm(size=40):
    """Salle carrée de 2 m (résolution 0.05) entourée d'un mur d'une cellule."""
    rows = []
    for r in range(size):
        rows.append(bytes(0 if r in (0, size - 1) or c in (0, size - 1) else 254
                          for c in range(size)))
    return f'P5\n# test\n{size} {size}\n255\n'.encode() + b''.join(rows)


def test_pgm_loading_and_scan_agreement():
    grid = load_pgm_map(_square_room_pgm(), 0.05, -1.0, -1.0)
    assert grid.width == 40 and grid.height == 40
    field = build_distance_field(grid)
    assert float(field_lookup(field, grid, [-0.99], [0.0])[0]) <= 0.01
    assert float(field_lookup(field, grid, [0.0], [0.0])[0]) > 0.40

    n = 180
    angle_min, inc = -math.pi, 2 * math.pi / n
    def wall_range(a):
        return min(0.975 / max(abs(math.cos(a)), 1e-9), 0.975 / max(abs(math.sin(a)), 1e-9))
    ranges = [wall_range(angle_min + i * inc) for i in range(n)]
    agreement, used = scan_map_agreement(grid, Pose2D(0.0, 0.0, 0.0), angle_min, inc, ranges)
    assert used == n and agreement > 0.95
    # Même scan, mais robot supposé ailleurs : la carte ne colle plus.
    wrong, _ = scan_map_agreement(grid, Pose2D(0.5, 0.4, 0.6), angle_min, inc, ranges)
    assert wrong < 0.5


def test_pgm_rejects_other_formats():
    with pytest.raises(ValueError):
        load_pgm_map(b'P2\n2 2\n255\n0 0 0 0\n', 0.05, 0.0, 0.0)


def _rect_room(width_m=8.0, height_m=6.0, res=0.05, corridor=False):
    """Salle rectangulaire synthetique, murs d'une cellule."""
    w, h = int(width_m / res), int(height_m / res)
    ox, oy = -width_m / 2.0, -height_m / 2.0
    rows = []
    for r in range(h):
        line = bytearray(w)
        for c in range(w):
            if corridor:
                wall = r in (0, h - 1)
            else:
                wall = r in (0, h - 1) or c in (0, w - 1)
            line[c] = 1 if wall else 0
        rows.append(line)
    from caytu_nav_solution.nav_math import GridMap
    return GridMap(w, h, res, ox, oy, rows)


def test_distance_field_values():
    grid = _rect_room()
    field = build_distance_field(grid)
    assert float(field_lookup(field, grid, [-3.975], [0.0])[0]) == 0.0
    assert abs(float(field_lookup(field, grid, [-3.925], [0.0])[0]) - 0.05) < 0.02
    assert float(field_lookup(field, grid, [0.0], [0.0])[0]) == 0.5


def _fallback_grid(blocked_radius=0.0, blocked_cost=100, ring_cost=None):
    w = h = 40
    res = 0.05
    vals = [0] * (w * h)
    for r in range(h):
        for c in range(w):
            x = (c + 0.5) * res
            y = (r + 0.5) * res
            d = math.hypot(x - 1.0, y - 1.0)
            if d <= blocked_radius:
                vals[r * w + c] = blocked_cost
            elif ring_cost is not None and d <= 0.50:
                vals[r * w + c] = ring_cost
    return vals, w, h, res


def test_free_goal_returns_goal_cell():
    vals, w, h, res = _fallback_grid()
    cell = nearest_free_cell(vals, w, h, res, 0.0, 0.0, 1.0, 1.0, 1.0, 70)
    assert cell is not None and cell[2] < 0.05


def test_blocked_goal_returns_ring_cell():
    vals, w, h, res = _fallback_grid(blocked_radius=0.30)
    cell = nearest_free_cell(vals, w, h, res, 0.0, 0.0, 1.0, 1.0, 1.0, 70)
    assert cell is not None and 0.30 <= cell[2] <= 0.40
    c = int(math.floor((cell[0] - 0.0) / res))
    r = int(math.floor((cell[1] - 0.0) / res))
    assert vals[r * w + c] == 0


def test_cost_above_threshold_is_refused():
    vals, w, h, res = _fallback_grid(blocked_radius=0.30, ring_cost=80)
    cell = nearest_free_cell(vals, w, h, res, 0.0, 0.0, 1.0, 1.0, 1.0, 70)
    assert cell is not None and cell[2] >= 0.50


def test_excluded_point_is_skipped():
    vals, w, h, res = _fallback_grid(blocked_radius=0.30)
    first = nearest_free_cell(vals, w, h, res, 0.0, 0.0, 1.0, 1.0, 1.0, 70)
    second = nearest_free_cell(vals, w, h, res, 0.0, 0.0, 1.0, 1.0, 1.0, 70,
                               excluded=[(first[0], first[1])])
    assert math.hypot(second[0] - first[0], second[1] - first[1]) > 0.15


def test_all_blocked_returns_none():
    assert nearest_free_cell([100] * 1600, 40, 40, 0.05, 0.0, 0.0, 1.0, 1.0, 1.0, 70) is None


def test_goal_outside_grid():
    vals, _, _, _ = _fallback_grid()
    assert nearest_free_cell(vals, 40, 40, 0.05, 0.0, 0.0, 5.0, 5.0, 1.0, 70) is None


# --------------------------------------------------------------------------- #
# Champ de distance : la version en deux passes doit rester exacte
# --------------------------------------------------------------------------- #
def _brute_force_field(occ, res, max_dist):
    height, width = occ.shape
    ys, xs = np.nonzero(occ)
    out = np.full(occ.shape, max_dist, dtype=np.float64)
    for r in range(height):
        for c in range(width):
            if len(ys):
                d = res * math.sqrt(float(((ys - r) ** 2 + (xs - c) ** 2).min()))
                out[r, c] = min(d, max_dist)
    return out


def test_distance_field_matches_brute_force():
    rng = np.random.default_rng(3)
    for density in (0.01, 0.05, 0.30):
        occ = rng.random((23, 31)) < density
        grid = GridMap(31, 23, 0.05, 0.0, 0.0, occ)
        for max_dist in (0.2, 0.5):
            field = build_distance_field(grid, max_dist)
            # Exact à mieux qu'une demi-cellule près de la troncature, exact ailleurs.
            reference = _brute_force_field(occ, 0.05, max_dist)
            assert np.abs(field - reference).max() < 1e-5


def test_distance_field_of_an_empty_map_is_the_cap():
    grid = GridMap(8, 5, 0.05, 0.0, 0.0, np.zeros((5, 8), dtype=bool))
    assert np.all(build_distance_field(grid, 0.5) == np.float32(0.5))


def test_gridmap_accepts_rows_and_arrays():
    rows = [bytearray([0, 1, 0]), bytearray([1, 0, 0])]
    grid = GridMap(3, 2, 0.05, 0.0, 0.0, rows)
    assert grid.occupied.dtype == bool and grid.occupied.shape == (2, 3)
    assert bool(grid.occupied[0][1]) and not bool(grid.occupied[1][1])


def test_pgm_rows_are_flipped_and_truncated_files_rejected():
    # Ligne du haut du fichier (y maximal) occupée, ligne du bas libre.
    pgm = b'P5\n3 2\n255\n' + bytes([0, 0, 0, 254, 254, 254])
    grid = load_pgm_map(pgm, 0.05, 0.0, 0.0)
    assert not grid.occupied[0].any() and grid.occupied[1].all()
    with pytest.raises(ValueError):
        load_pgm_map(b'P5\n3 2\n255\n' + bytes([0, 0]), 0.05, 0.0, 0.0)


# --------------------------------------------------------------------------- #
# Filtre du sol : valeurs non finies
# --------------------------------------------------------------------------- #
def test_floor_split_keeps_nan_and_inf_untouched():
    floor = [2.0, 2.0, math.inf, 2.0]
    ranges = [math.nan, math.inf, 1.0, 13.0]               # 13 m : au-delà de range_max
    mark, clear, count = split_floor_returns(ranges, floor, range_max=12.0)
    assert count == 0
    assert math.isnan(mark[0]) and math.isnan(clear[0])
    assert mark[1:] == [math.inf, 1.0, 13.0] and clear[1:] == [math.inf, 1.0, 13.0]


def test_cell_cost_reads_one_cell():
    values = [0, 10, 20, 30, 40, 50]                    # 3 colonnes x 2 lignes
    assert cell_cost(values, 3, 2, 0.5, -1.0, 2.0, -0.9, 2.1) == 0
    assert cell_cost(values, 3, 2, 0.5, -1.0, 2.0, 0.2, 2.6) == 50
    assert cell_cost(values, 3, 2, 0.5, -1.0, 2.0, 0.6, 2.1) is None
    assert cell_cost(values, 3, 2, 0.5, -1.0, 2.0, -1.1, 2.1) is None


def test_footprint_gap_measures_the_distance_to_the_robot_outline():
    robot = Pose2D(1.0, 2.0, math.pi / 2)              # tourné vers +y
    assert footprint_gap(robot, 1.0, 2.0) == 0.0          # sous le robot
    # Devant : l'avant est à 0,198 m du centre.
    assert footprint_gap(robot, 1.0, 3.0) == pytest.approx(1.0 - 0.198)
    # Derrière : l'arrière est à 0,288 m.
    assert footprint_gap(robot, 1.0, 1.0) == pytest.approx(1.0 - 0.288)
    # Sur le côté : demi-largeur 0,222 m.
    assert footprint_gap(robot, 0.0, 2.0) == pytest.approx(1.0 - 0.222)
    # En diagonale d'un coin.
    assert footprint_gap(robot, 1.0 - 0.222 - 0.3, 2.0 + 0.198 + 0.4) == pytest.approx(0.5)
