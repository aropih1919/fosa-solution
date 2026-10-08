"""Tests du choix « garder la route ou prendre la nouvelle » (sans ROS)."""
import math
import random

import numpy as np
import pytest

from caytu_nav_solution.path_choice import (
    BETTER, BLOCKED, FIRST, KEPT, NEW_GOAL, NO_COSTMAP, OFF_PATH, SAME_ROUTE, TOO_CLOSE,
    CostGrid, RouteKeeper, as_points, choose_path, closest_index, heading_at,
    occupancy_to_cost, route_deviation, too_close, travel_cost, turn_between)

RES = 0.05


def _grid(width_m=12.0, height_m=8.0):
    """Costmap libre de 12 m x 8 m, origine (-1, -4)."""
    values = np.zeros((int(height_m / RES), int(width_m / RES)), dtype=np.int8)
    return CostGrid(values, RES, -1.0, -4.0)


def _paint(grid, x0, x1, y0, y1, value):
    c0, c1 = int((x0 - grid.origin_x) / RES), int((x1 - grid.origin_x) / RES)
    r0, r1 = int((y0 - grid.origin_y) / RES), int((y1 - grid.origin_y) / RES)
    grid.values[r0:r1, c0:c1] = value


def _line(x0, y0, x1, y1, step=RES):
    n = max(2, int(math.hypot(x1 - x0, y1 - y0) / step) + 1)
    return as_points([(x0 + (x1 - x0) * k / (n - 1), y0 + (y1 - y0) * k / (n - 1))
                      for k in range(n)])


def _detour(side, start=(0.0, 0.0), bulge=1.5, goal_x=10.0):
    """Chemin de `start` à (goal_x, 0) qui contourne une table en (5, 0) par le côté donné."""
    first = _line(start[0], start[1], 5.0, side * bulge)
    return np.vstack([first, _line(5.0, side * bulge, goal_x, 0.0)[1:]])


def test_occupancy_values_map_back_to_nav2_costs():
    cost = occupancy_to_cost(np.array([-1, 0, 1, 98, 99, 100]))
    assert list(cost) == pytest.approx([0.0, 0.0, 1.0, 252.0, 253.0, 254.0])


def test_travel_cost_follows_the_planner_formula():
    grid = _grid()
    path = _line(0.0, 0.0, 4.0, 0.0)
    cost, blocked = travel_cost(path, grid)
    assert cost == pytest.approx(4.0) and not blocked
    _paint(grid, -1.0, 11.0, -4.0, 4.0, 50)          # coût uniforme
    cost, blocked = travel_cost(path, grid, multiplier=3.0)
    expected = 4.0 * (1.0 + 3.0 * float(occupancy_to_cost(np.array([50]))[0]) / 252.0)
    assert cost == pytest.approx(expected) and not blocked
    _paint(grid, 2.0, 2.2, -0.2, 0.2, 100)
    assert travel_cost(path, grid)[1] is True
    # Hors de la grille : compté comme libre, jamais d'erreur.
    assert travel_cost(_line(50.0, 50.0, 51.0, 50.0), grid) == (pytest.approx(1.0), False)


def test_closest_index_finds_the_point_ahead_of_the_robot():
    path = _line(0.0, 0.0, 10.0, 0.0)
    index, offset = closest_index(path, 4.02, 0.1)
    assert path[index, 0] == pytest.approx(4.0, abs=RES)
    assert offset == pytest.approx(0.1, abs=0.03)


def test_deviation_separates_an_adjusted_path_from_another_route():
    left, right = _detour(+1), _detour(-1)
    assert route_deviation(left, right) > 2.5                              # côtés opposés
    assert route_deviation(left, _detour(+1, bulge=1.2)) == pytest.approx(0.3, abs=0.1)
    assert route_deviation(left, left) == 0.0
    # Un raccourci qui coupe le détour est bien « ailleurs », dans les deux sens.
    straight = _line(0.0, 0.0, 10.0, 0.0)
    assert route_deviation(straight, left) > 1.0 and route_deviation(left, straight) > 1.0
    # Chemins très longs : la mesure reste bornée à 120 points par chemin.
    long_a, long_b = _line(0.0, 0.0, 10.0, 0.0, step=0.002), _line(0.0, 0.5, 10.0, 0.5, step=0.002)
    assert route_deviation(long_a, long_b) == pytest.approx(0.5, abs=0.05)


def test_first_path_is_always_taken():
    choice = choose_path(None, _line(0.0, 0.0, 10.0, 0.0), _grid())
    assert choice.use_new and choice.reason == FIRST


def test_a_path_on_the_same_route_is_always_taken():
    # Même côté de la table, chemin simplement ajusté : aucune retenue, même si
    # le nouveau chemin est plus cher (le planificateur a ses raisons).
    grid = _grid()
    kept = _detour(+1, bulge=1.5)
    for bulge in (1.2, 1.8):
        choice = choose_path(kept, _detour(+1, bulge=bulge), grid)
        assert choice.use_new and choice.reason == SAME_ROUTE
        assert choice.deviation < 1.0
    # ... et cela sans costmap.
    assert choose_path(kept, _detour(+1, bulge=1.3), None).reason == SAME_ROUTE


def test_an_equivalent_route_on_the_other_side_is_not_taken():
    grid = _grid()
    choice = choose_path(_detour(+1), _detour(-1), grid)
    assert not choice.use_new and choice.reason == KEPT
    assert choice.gain == pytest.approx(0.0, abs=0.01) and choice.deviation > 1.0


def test_an_obstacle_on_the_kept_route_forces_the_change():
    grid = _grid()
    _paint(grid, 4.8, 5.2, 1.3, 1.7, 100)            # obstacle sur la route suivie
    choice = choose_path(_detour(+1), _detour(-1), grid)
    assert choice.use_new and choice.reason == BLOCKED


def test_a_kept_route_that_brushes_an_obstacle_ahead_is_left():
    grid = _grid()
    _paint(grid, 4.8, 5.2, 1.3, 1.7, 92)             # pas interdit, mais le robot frôlerait
    assert too_close(_detour(+1), grid)
    choice = choose_path(_detour(+1), _detour(-1), grid)
    assert choice.use_new and choice.reason == TOO_CLOSE
    # Le même coût sous le robot lui-même n'est pas une raison de changer :
    # il longe déjà l'obstacle.
    grid = _grid()
    _paint(grid, -0.2, 0.3, -0.2, 0.3, 92)
    assert not too_close(_detour(+1), grid)
    assert not choose_path(_detour(+1), _detour(-1), grid).use_new


def test_the_other_route_is_taken_only_for_a_clear_gain():
    grid = _grid()
    kept = _detour(+1, bulge=1.5)
    # Autre route plus courte de 1 m environ : gain insuffisant, on garde.
    shorter = _detour(-1, bulge=0.6)
    gain = travel_cost(kept, grid)[0] - travel_cost(shorter, grid)[0]
    assert 0.3 < gain < 1.5
    choice = choose_path(kept, shorter, grid)
    assert not choice.use_new and choice.reason == KEPT
    # La route suivie devient chère sur 2 m (passage étroit) : on change.
    _paint(grid, 4.0, 6.0, 1.0, 2.0, 60)
    choice = choose_path(kept, shorter, grid, turn_cost=0.0)
    assert choice.use_new and choice.reason == BETTER and choice.gain > 1.5
    # Le seuil est un paramètre.
    assert choose_path(kept, shorter, _grid(), min_gain=0.2, turn_cost=0.0).use_new


def test_only_the_part_of_the_kept_path_ahead_of_the_robot_is_compared():
    grid = _grid()
    kept = _detour(+1)
    # Le robot est au sommet du détour ; l'autre route repartirait en arrière.
    new = np.vstack([_line(5.0, 1.5, 5.0, -1.5), _line(5.0, -1.5, 10.0, 0.0)[1:]])
    choice = choose_path(kept, new, grid)
    assert not choice.use_new
    assert kept[choice.start_index, 0] == pytest.approx(5.0, abs=RES)
    assert choice.kept_cost < choice.new_cost
    # Un obstacle DERRIÈRE le robot ne compte plus.
    _paint(grid, 2.0, 2.4, 0.4, 0.9, 100)
    assert not choose_path(kept, new, grid).use_new


def test_robot_far_from_the_kept_route_takes_the_new_one():
    grid = _grid()
    kept = _detour(+1)
    start = (2.0, -0.2)                              # à 0,7 m du chemin gardé
    choice = choose_path(kept, _detour(-1, start=start), grid, max_offset=0.50)
    assert choice.use_new and choice.reason == OFF_PATH


def test_new_goal_or_missing_costmap_takes_the_new_path():
    grid = _grid()
    kept = _detour(+1)
    choice = choose_path(kept, _line(0.0, 0.0, 9.0, 1.0), grid)
    assert choice.use_new and choice.reason == NEW_GOAL
    choice = choose_path(kept, _detour(-1), None)
    assert choice.use_new and choice.reason == NO_COSTMAP
    with pytest.raises(ValueError):
        choose_path(kept, as_points([]), grid)


def test_kept_path_almost_finished_gives_way_to_the_new_one():
    grid = _grid()
    kept = _line(0.0, 0.0, 10.0, 0.0)
    new = as_points([(10.0, 0.0), (10.0, 0.0)])
    assert choose_path(kept, new, grid).use_new


def _count_switches(min_gain, seed=3, rounds=40, amplitude=18, turn_cost=1.0):
    """Le robot suit son chemin vers une table contournable des deux côtés. À
    chaque calcul, le côté qu'il regarde paraît plus encombré (surcoût aléatoire
    sur ce côté, jusqu'à 1 m équivalent) : le planificateur propose l'autre.
    Compte les changements de côté."""
    rng = random.Random(seed)
    kept, kept_side, switches = None, 0, 0
    robot = (0.0, 0.0)
    for _ in range(rounds):
        grid = _grid()
        looked = kept_side or rng.choice((+1, -1))
        value = rng.randint(amplitude // 2, amplitude)
        _paint(grid, 4.0, 6.0, min(looked * 1.2, looked * 1.8), max(looked * 1.2, looked * 1.8),
               value)
        side = -looked                              # le côté non regardé paraît moins cher
        new = _detour(side, start=robot)
        choice = choose_path(kept, new, grid, min_gain=min_gain, turn_cost=turn_cost)
        if choice.use_new:
            if kept is not None and side != kept_side:
                switches += 1
            kept, kept_side = new, side
        else:
            kept = kept[choice.start_index:]
        # Le robot avance de 10 cm le long du chemin retenu, sans dépasser x = 3.
        if kept[0, 0] < 3.0:
            kept = kept[2:]
        robot = (float(kept[0, 0]), float(kept[0, 1]))
    return switches


def test_commitment_removes_the_side_flip_flop():
    assert _count_switches(min_gain=0.0, turn_cost=0.0) > 10    # sans retenue : la route change sans cesse
    assert _count_switches(min_gain=1.5) == 0       # avec retenue : une seule route, jusqu'au bout


# --------------------------------------------------------------------------- #
# RouteKeeper : règles dans le temps
# --------------------------------------------------------------------------- #
def _poses(points):
    return [f'pose{k}' for k in range(points.shape[0])]


def _feed(keeper, now, path, grid=None, goal=(10.0, 0.0)):
    grid = grid if grid is not None else _grid()
    return keeper.update(now, path, _poses(path), grid, goal)


def test_keeper_sends_the_kept_poses_trimmed_to_the_robot():
    keeper = RouteKeeper()
    left = _detour(+1)
    choice, sent = _feed(keeper, 0.0, left)
    assert choice.reason == FIRST and sent == _poses(left)
    # L'autre côté est proposé alors que le robot est au sommet du détour gauche.
    right = np.vstack([_line(5.0, 1.5, 5.0, -1.5), _line(5.0, -1.5, 10.0, 0.0)[1:]])
    choice, sent = _feed(keeper, 0.5, right)
    assert not choice.use_new and choice.reason == KEPT
    assert sent == _poses(left)[choice.start_index:]
    assert keeper.holds == 1 and keeper.switches == 0


def test_a_route_change_for_gain_waits_for_the_dwell_time():
    keeper = RouteKeeper(min_gain=0.2, min_dwell=5.0, turn_cost=0.0)
    _feed(keeper, 0.0, _detour(+1, bulge=1.5))
    shorter = _detour(-1, bulge=0.6)
    choice, _ = _feed(keeper, 0.5, shorter)
    assert choice.use_new and choice.reason == BETTER         # premier changement : permis
    assert keeper.switches == 1
    # Le côté choisi devient cher (sans être coupé) : l'autre côté redevient
    # nettement meilleur, mais le retour est refusé pendant 5 s.
    grid = _grid()
    _paint(grid, 3.0, 7.0, -1.0, -0.3, 40)
    back = _detour(+1, bulge=1.2)
    for now in (1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0):
        choice, _ = _feed(keeper, now, back, grid)
        assert not choice.use_new and choice.gain > 0.2, now
    choice, _ = _feed(keeper, 6.0, back, grid)
    assert choice.use_new and choice.reason == BETTER
    assert keeper.switches == 2


def test_a_blocked_route_is_left_at_once_even_during_the_dwell_time():
    keeper = RouteKeeper(min_gain=0.2, min_dwell=5.0, turn_cost=0.0)
    _feed(keeper, 0.0, _detour(+1, bulge=1.5))
    _feed(keeper, 0.5, _detour(-1, bulge=0.6))               # changement pour un gain
    grid = _grid()
    _paint(grid, 4.8, 5.2, -0.8, -0.4, 100)                   # la nouvelle route est coupée
    choice, _ = _feed(keeper, 1.0, _detour(+1, bulge=1.5), grid)
    assert choice.use_new and choice.reason == BLOCKED


def test_too_close_must_be_seen_twice_in_a_row():
    keeper = RouteKeeper()
    _feed(keeper, 0.0, _detour(+1))
    grid = _grid()
    _paint(grid, 4.8, 5.2, 1.3, 1.7, 92)
    choice, _ = _feed(keeper, 0.5, _detour(-1), grid)
    assert not choice.use_new and choice.reason == TOO_CLOSE   # une fois : peut-être du bruit
    choice, _ = _feed(keeper, 1.0, _detour(+1), _grid())       # disparu : on oublie
    assert choice.reason == SAME_ROUTE
    choice, _ = _feed(keeper, 1.5, _detour(-1), grid)
    assert not choice.use_new
    choice, _ = _feed(keeper, 2.0, _detour(-1), grid)          # deux fois de suite : on change
    assert choice.use_new and choice.reason == TOO_CLOSE


def test_keeper_forgets_after_a_pause_or_a_new_goal():
    keeper = RouteKeeper()
    _feed(keeper, 0.0, _detour(+1))
    # Plus de 2 s sans demande : une récupération a eu lieu, on repart de zéro.
    choice, _ = _feed(keeper, 3.0, _detour(-1))
    assert choice.use_new and choice.reason == FIRST
    choice, _ = _feed(keeper, 3.5, _detour(+1), goal=(9.0, 1.0))
    assert choice.use_new and choice.reason == FIRST
    keeper.forget()
    assert keeper.kept is None and keeper.payload == []
    with pytest.raises(ValueError):
        keeper.update(4.0, _detour(+1), ['une seule pose'], _grid(), (10.0, 0.0))


# --------------------------------------------------------------------------- #
# Prix du demi-tour
# --------------------------------------------------------------------------- #
def test_heading_and_turn_between_two_routes():
    east = _line(0.0, 0.0, 3.0, 0.0)
    north = _line(0.0, 0.0, 0.0, 3.0)
    west = _line(0.0, 0.0, -3.0, 0.0)
    assert heading_at(east) == pytest.approx(0.0)
    assert heading_at(north) == pytest.approx(math.pi / 2)
    assert turn_between(east, north) == pytest.approx(math.pi / 2)
    assert turn_between(east, west) == pytest.approx(math.pi)
    assert turn_between(east, east) == 0.0
    assert heading_at(as_points([(1.0, 1.0)])) is None
    assert turn_between(as_points([(1.0, 1.0)]), east) == 0.0


def test_a_route_behind_the_robot_must_pay_for_the_half_turn():
    grid = _grid()
    # Le robot (en 5, 0) roule vers l'est ; le but est en (10, 0).
    kept = np.vstack([_line(5.0, 0.0, 6.0, 0.0), _line(6.0, 0.0, 6.0, 3.0)[1:],
                      _line(6.0, 3.0, 10.0, 3.0)[1:], _line(10.0, 3.0, 10.0, 0.0)[1:]])
    # Autre route : repartir vers l'ouest puis faire le tour (plus courte d'environ 2 m).
    behind = np.vstack([_line(5.0, 0.0, 4.0, 0.0), _line(4.0, 0.0, 4.0, -1.0)[1:],
                        _line(4.0, -1.0, 10.0, -1.0)[1:], _line(10.0, -1.0, 10.0, 0.0)[1:]])
    gain = travel_cost(kept, grid)[0] - travel_cost(behind, grid)[0]
    assert 1.5 < gain < 1.5 + math.pi
    choice = choose_path(kept, behind, grid, min_gain=1.5, turn_cost=1.0)
    assert not choice.use_new and choice.turn == pytest.approx(math.pi)
    # Sans prix du pivot, ce gain suffirait.
    assert choose_path(kept, behind, grid, min_gain=1.5, turn_cost=0.0).use_new
