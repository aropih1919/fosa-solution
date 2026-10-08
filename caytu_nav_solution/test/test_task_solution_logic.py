"""Scénarios complets de task_solution.py contre un faux Nav2 (voir fake_ros.py).

Chaque test déroule la vraie boucle `run()` : envoi du but, interruption,
choix du nettoyage, repli, limite de temps. Aucun simulateur n'est lancé et le
temps est simulé : la suite entière dure moins d'une seconde.
"""
import json
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))
import fake_ros                                             # noqa: E402

SPAWN = (-0.2, -7.5, 1.571)
GOAL = (-2.3, 2.2)
LOCAL = '/local_costmap/clear_entirely_local_costmap'
GLOBAL_ALL = '/global_costmap/clear_entirely_global_costmap'
GLOBAL_FAR = '/global_costmap/clear_except_global_costmap'
GLOBAL_NEAR = '/global_costmap/clear_around_global_costmap'


def _task_file(tmp_path):
    path = tmp_path / 'task_params.yaml'
    path.write_text(
        '/**:\n  ros__parameters:\n'
        f'    x: {SPAWN[0]}\n    y: {SPAWN[1]}\n    z: 0.22\n    yaw: {SPAWN[2]}\n'
        f'    goal_x: {GOAL[0]}\n    goal_y: {GOAL[1]}\n    goal_z: 0.22\n', encoding='utf-8')
    return str(path)


@pytest.fixture
def setup(tmp_path):
    world, module = fake_ros.load_task_solution()
    world.robot = list(SPAWN)

    def make(**parameters):
        values = {
            'task_params_file': _task_file(tmp_path),
            'launch_bringup': False,            # pas de ros2 launch dans un test
            'static_map': 'always',             # pas d'attente de scan
            'map_yaml': str(tmp_path / 'carte.yaml'),
            'report_dir': str(tmp_path / 'rapports'),
        }
        values.update(parameters)
        overrides = [fake_ros.Parameter(name, value=value) for name, value in values.items()]
        return module.TaskSolution(parameter_overrides=overrides)

    return world, module, make, tmp_path


def _costmap_with_blocked_goal(world, radius):
    """Costmap globale 8 x 8 m autour du but, disque occupé de `radius` sur le but."""
    res, size = 0.05, 160
    origin_x, origin_y = GOAL[0] - 4.0, GOAL[1] - 4.0
    values = [0] * (size * size)
    for row in range(size):
        for col in range(size):
            x = origin_x + (col + 0.5) * res
            y = origin_y + (row + 0.5) * res
            distance = math.hypot(x - GOAL[0], y - GOAL[1])
            if radius <= 0.0:
                continue                                 # aucun obstacle
            if distance <= radius:
                values[row * size + col] = 100
            elif distance <= radius + 0.30:
                values[row * size + col] = 90            # inflation : trop près
    world.set_costmap(values, size, size, res, origin_x, origin_y)


def _global_clears(world):
    return [name for name, _ in world.clears if name != LOCAL]


def test_normal_run_reaches_the_goal_on_the_first_attempt(setup):
    world, module, make, tmp_path = setup
    node = make()
    assert node.run() == module.EXIT_SUCCESS
    assert world.sent_goals == [pytest.approx(GOAL)]
    assert world.clears == [] and world.cancels == 0
    assert 'BUT ATTEINT' in world.log_text()
    node._write_report(module.EXIT_SUCCESS)
    reports = list((tmp_path / 'rapports').glob('run_*.json'))
    assert len(reports) == 1
    data = json.loads(reports[0].read_text(encoding='utf-8'))
    assert data['result'] == 'success' and data['attempts'] == 1 and data['failures'] == 0
    assert (tmp_path / 'rapports' / 'trajets.csv').exists()
    assert data['settings'] == 'default' and data['nav2_ready_s'] is not None
    assert world.sent_trees == ['']                     # path_keeper absent : arbre par défaut
    node.shutdown_sequence()
    # Le robot reçoit des vitesses nulles à l'arrêt.
    assert world.published and all(m.linear.x == 0.0 for _, m in world.published)


def test_the_cause_of_an_interruption_is_logged_and_progress_keeps_memory(setup):
    world, module, make, _ = setup
    # Le robot avance de 5 m puis Nav2 abandonne faute de commande sûre.
    world.nav_script = [
        {'status': 'ABORTED', 'error_code': 106, 'move_to': (-1.0, -2.5)},
        {},
    ]
    node = make()
    assert node.run() == module.EXIT_SUCCESS
    text = world.log_text()
    assert 'NO_VALID_CONTROL' in text and 'contrôleur' in text and 'code 106' in text
    # Progrès réel : on ne garde que le nettoyage doux (zone lointaine).
    assert _global_clears(world) == [GLOBAL_FAR]
    assert dict(world.clears)[GLOBAL_FAR] == pytest.approx(5.0)       # 2 x 2,5 m gardés
    assert len(world.sent_goals) == 2
    assert node._metrics.failures == ['NO_VALID_CONTROL']


def test_repeated_failures_without_progress_escalate_to_a_full_clear(setup):
    world, module, make, _ = setup
    _costmap_with_blocked_goal(world, radius=0.0)          # le but est libre
    world.nav_script = [
        {'status': 'ABORTED', 'error_code': 208, 'move_to': None},
        {'status': 'ABORTED', 'error_code': 208, 'move_to': None},
        {},
    ]
    node = make()
    assert node.run() == module.EXIT_SUCCESS
    # 1er échec sans progrès : zone lointaine. 2e : le repli est examiné, le but
    # est libre et atteignable, donc dernier recours : tout effacer.
    assert _global_clears(world) == [GLOBAL_FAR, GLOBAL_ALL]
    assert not node._on_fallback and len(world.sent_goals) == 3
    assert 'NO_VALID_PATH' in world.log_text()


def test_goal_occupied_triggers_the_fallback_immediately(setup):
    world, module, make, _ = setup
    _costmap_with_blocked_goal(world, radius=0.20)         # une boîte posée sur le but
    world.nav_script = [{'status': 'ABORTED', 'error_code': 206, 'move_to': (-2.0, 0.5)}, {}]
    node = make()
    assert node.run() == module.EXIT_SUCCESS
    assert node._on_fallback and len(world.sent_goals) == 2
    target = world.sent_goals[1]
    offset = math.hypot(target[0] - GOAL[0], target[1] - GOAL[1])
    assert 0.50 <= offset <= 0.60                          # hors boîte et hors inflation
    assert world.clears == []                              # costmap intacte pour le repli
    text = world.log_text()
    assert 'GOAL_OCCUPIED' in text and 'BUT ATTEINT par repli' in text
    assert node._metrics.fallback_used and node._metrics.fallback_offset_m == pytest.approx(offset)


def test_fallback_asks_the_planner_of_the_behavior_tree(setup):
    world, module, make, _ = setup
    _costmap_with_blocked_goal(world, radius=0.20)
    world.nav_script = [{'status': 'ABORTED', 'error_code': 208, 'move_to': None}, {}]
    node = make()
    assert node.run() == module.EXIT_SUCCESS
    assert node._on_fallback
    assert {planner for planner, _, _ in world.plan_requests} == {'GridBased'}


def test_fallback_outside_the_circle_holds_then_returns_to_the_official_goal(setup):
    world, module, make, _ = setup
    _costmap_with_blocked_goal(world, radius=0.60)         # gros obstacle sur le but
    blocked = {'value': True}
    world.plan_ok = lambda planner, x, y: not (
        blocked['value'] and math.hypot(x - GOAL[0], y - GOAL[1]) < 0.05)
    world.nav_script = [{'status': 'ABORTED', 'error_code': 206, 'move_to': None}, {}, {}]
    # L'obstacle disparaît 40 s après le départ.
    world.at(40.0, lambda: blocked.update(value=False))
    node = make()
    assert node.run() == module.EXIT_SUCCESS
    text = world.log_text()
    assert 'Repli atteint hors du cercle' in text
    assert 'Le but officiel est de nouveau atteignable' in text
    assert not node._on_fallback
    assert world.sent_goals[-1] == pytest.approx(GOAL)
    assert math.hypot(world.robot[0] - GOAL[0], world.robot[1] - GOAL[1]) < 0.01


def test_start_occupied_clears_only_around_the_robot(setup):
    world, module, make, _ = setup
    world.nav_script = [{'status': 'ABORTED', 'error_code': 205, 'move_to': None}, {}]
    node = make()
    assert node.run() == module.EXIT_SUCCESS
    assert _global_clears(world) == [GLOBAL_NEAR]
    assert dict(world.clears)[GLOBAL_NEAR] == pytest.approx(1.2)
    assert 'START_OCCUPIED' in world.log_text()


def test_a_silent_nav2_is_cancelled_and_the_goal_is_sent_again(setup):
    world, module, make, _ = setup
    world.nav_script = [{'never': True, 'feedback': False}, {}]
    node = make(nav2_silence_timeout_sec=30.0)
    start = world.t
    assert node.run() == module.EXIT_SUCCESS
    assert world.cancels == 1 and len(world.sent_goals) == 2
    assert 30.0 <= world.t - start <= 40.0
    assert 'Aucune nouvelle de Nav2' in world.log_text()
    assert node._metrics.failures == ['NAV2_SILENT']


def test_feedback_keeps_a_long_navigation_alive(setup):
    world, module, make, _ = setup
    world.nav_script = [{'duration': 120.0}]               # long, mais Nav2 donne des nouvelles
    node = make(nav2_silence_timeout_sec=30.0)
    assert node.run() == module.EXIT_SUCCESS
    assert world.cancels == 0 and len(world.sent_goals) == 1


def test_arrival_announced_too_far_is_sent_again(setup):
    world, module, make, _ = setup
    world.nav_script = [{'move_to': (GOAL[0] + 0.5, GOAL[1])}, {}]
    node = make()
    assert node.run() == module.EXIT_SUCCESS
    assert len(world.sent_goals) == 2
    assert node._metrics.failures == ['ARRIVAL_TOO_FAR']


def test_rejected_goals_are_retried_without_touching_the_costmaps(setup):
    world, module, make, _ = setup
    world.nav_script = [{'reject': True}, {'reject': True}, {}]
    node = make()
    assert node.run() == module.EXIT_SUCCESS
    assert world.clears == [] and node._metrics.attempts == 1 and node._metrics.rejections == 2


def test_time_limit_stops_a_goal_that_never_succeeds(setup):
    world, module, make, tmp_path = setup
    world.nav_script = [{'status': 'ABORTED', 'error_code': 208, 'move_to': None,
                         'duration': 5.0} for _ in range(400)]
    node = make(time_limit_sec=120.0, goal_fallback=False)
    start = world.t
    assert node.run() == module.EXIT_TIMEOUT
    assert 115.0 <= world.t - start <= 125.0               # limite respectée
    assert 'Limite de temps atteinte' in world.log_text()
    # Sans repli : zone lointaine, puis tout, en alternance.
    assert _global_clears(world)[:4] == [GLOBAL_FAR, GLOBAL_ALL, GLOBAL_FAR, GLOBAL_ALL]
    node._write_report(module.EXIT_TIMEOUT)
    data = json.loads(next((tmp_path / 'rapports').glob('run_*.json')).read_text(encoding='utf-8'))
    assert data['result'] == 'timeout' and 'NO_VALID_PATH' in data['failure_causes']


def test_lost_pose_at_the_end_does_not_crash_the_success_message(setup):
    world, module, make, _ = setup
    _costmap_with_blocked_goal(world, radius=0.20)
    world.nav_script = [{'status': 'ABORTED', 'error_code': 206, 'move_to': None}, {}]
    node = make()
    original = node._navigate_once

    def navigate_then_lose_tf():
        outcome = original()
        if outcome == 'success':
            world.tf_ok = False            # la position disparaît juste après l'arrivée
        return outcome
    node._navigate_once = navigate_then_lose_tf
    assert node.run() == module.EXIT_SUCCESS


def test_error_numbers_come_from_the_installed_package(setup, monkeypatch):
    world, module, make, _ = setup
    # Une version de Nav2 qui numéroterait autrement : 308 au lieu de 208.
    monkeypatch.setattr(fake_ros.ComputePathToPose.Result, 'NO_VALID_PATH', 308)
    world.nav_script = [{'status': 'ABORTED', 'error_code': 308, 'move_to': (-1.0, -2.5)}, {}]
    node = make()
    assert node._errors[308] == ('planificateur', 'NO_VALID_PATH')
    assert 208 not in node._errors
    assert node.run() == module.EXIT_SUCCESS
    assert node._metrics.failures == ['NO_VALID_PATH']
    assert 'code 308' in world.log_text()


def test_no_valid_path_with_a_blocked_goal_goes_straight_to_the_fallback(setup):
    world, module, make, _ = setup
    # Ce que Nav2 répond réellement pour une boîte posée sur le but : « aucun
    # chemin » (208), jamais « but occupé ». C'est la costmap qui tranche.
    _costmap_with_blocked_goal(world, radius=0.20)
    world.nav_script = [{'status': 'ABORTED', 'error_code': 208, 'move_to': (-2.0, 0.5)}, {}]
    node = make()
    assert node.run() == module.EXIT_SUCCESS
    assert node._on_fallback and len(world.sent_goals) == 2      # repli dès le 1er échec
    assert world.clears == []
    text = world.log_text()
    assert 'Le but est occupé dans la costmap globale' in text and 'BUT ATTEINT par repli' in text


def test_repeated_start_occupied_does_not_loop_on_the_same_cleanup(setup):
    world, module, make, _ = setup
    _costmap_with_blocked_goal(world, radius=0.0)
    world.nav_script = [{'status': 'ABORTED', 'error_code': 205, 'move_to': None}] * 3 + [{}]
    node = make()
    assert node.run() == module.EXIT_SUCCESS
    # Autour du robot une fois, puis l'échelle normale (ici : tout, puis loin).
    assert _global_clears(world) == [GLOBAL_NEAR, GLOBAL_ALL, GLOBAL_FAR]


def test_time_limit_cancels_a_goal_in_flight(setup):
    world, module, make, _ = setup
    world.nav_script = [{'never': True, 'duration': 1000.0}]      # Nav2 vivant, jamais fini
    node = make(time_limit_sec=60.0, nav2_silence_timeout_sec=0.0)
    start = world.t
    assert node.run() == module.EXIT_TIMEOUT
    assert world.cancels == 1 and len(world.sent_goals) == 1
    assert 55.0 <= world.t - start <= 65.0
    assert 'Limite de temps atteinte : annulation du but' in world.log_text()


def test_time_limit_ends_the_wait_at_a_fallback_point(setup):
    world, module, make, _ = setup
    _costmap_with_blocked_goal(world, radius=0.60)
    # Le but officiel reste inaccessible jusqu'au bout.
    world.plan_ok = lambda planner, x, y: math.hypot(x - GOAL[0], y - GOAL[1]) > 0.05
    world.nav_script = [{'status': 'ABORTED', 'error_code': 208, 'move_to': None}, {}]
    node = make(time_limit_sec=90.0)
    start = world.t
    assert node.run() == module.EXIT_TIMEOUT
    assert 85.0 <= world.t - start <= 95.0
    assert 'Repli atteint hors du cercle' in world.log_text()
    assert len(world.sent_goals) == 2                              # le robot ne bouge plus


# --------------------------------------------------------------------------- #
# Réglages de conduite et chemin stabilisé
# --------------------------------------------------------------------------- #
def _keeper_tree(tmp_path):
    path = tmp_path / 'navigate_keep_path.xml'
    path.write_text('<root/>', encoding='utf-8')
    return str(path)


def test_keeper_tree_is_requested_only_when_the_keeper_answers(setup):
    world, module, make, tmp_path = setup
    tree = _keeper_tree(tmp_path)
    world.keeper_ready = True
    node = make(keeper_tree=tree)
    assert node.run() == module.EXIT_SUCCESS
    assert world.sent_trees == [tree]
    assert 'Chemin stabilisé actif' in world.log_text()


def test_missing_keeper_falls_back_to_the_default_tree_after_a_bounded_wait(setup):
    world, module, make, tmp_path = setup
    node = make(keeper_tree=_keeper_tree(tmp_path), keeper_wait_sec=3.0)
    start = world.t
    assert node.run() == module.EXIT_SUCCESS
    assert world.sent_trees == ['']
    assert 'Chemin stabilisé indisponible' in world.log_text()
    assert world.sent_times[0] - start < 3.0 + 1.0      # attente bornée, puis départ


def test_keeper_is_not_waited_for_when_it_is_switched_off(setup):
    world, module, make, tmp_path = setup
    world.keeper_ready = True                           # même présent, il n'est pas utilisé
    node = make(keeper_tree=_keeper_tree(tmp_path), path_keeper=False)
    assert node.run() == module.EXIT_SUCCESS
    assert world.sent_trees == ['']
    assert 'Chemin stabilisé' not in world.log_text()
    assert node._metrics.settings == 'default + path_keeper=false'


def test_two_unexplained_interruptions_switch_the_keeper_off(setup):
    world, module, make, tmp_path = setup
    tree = _keeper_tree(tmp_path)
    world.keeper_ready = True
    world.nav_script = [
        {'status': 'ABORTED', 'move_to': None},         # sans code d'erreur
        {'status': 'ABORTED', 'move_to': None},
        {},
    ]
    node = make(keeper_tree=tree)
    assert node.run() == module.EXIT_SUCCESS
    assert world.sent_trees == [tree, tree, '']
    assert 'Chemin stabilisé désactivé après deux interruptions' in world.log_text()


def test_an_explained_interruption_keeps_the_keeper(setup):
    world, module, make, tmp_path = setup
    tree = _keeper_tree(tmp_path)
    world.keeper_ready = True
    world.nav_script = [
        {'status': 'ABORTED', 'error_code': 106, 'move_to': (-1.0, -2.5)},
        {'status': 'ABORTED', 'error_code': 106, 'move_to': (-1.5, 0.0)},
        {},
    ]
    node = make(keeper_tree=tree)
    assert node.run() == module.EXIT_SUCCESS
    assert world.sent_trees == [tree, tree, tree]


def test_rejected_goals_do_not_count_against_the_keeper(setup):
    # Un but rejeté veut dire « Nav2 pas encore actif », pas « chemin stabilisé en panne ».
    world, module, make, tmp_path = setup
    tree = _keeper_tree(tmp_path)
    world.keeper_ready = True
    world.nav_script = [{'reject': True}] * 6 + [{}]
    node = make(keeper_tree=tree)
    assert node.run() == module.EXIT_SUCCESS
    assert world.sent_trees == [tree] * 7
    assert node._metrics.rejections == 6 and node._metrics.attempts == 1


def test_missing_keeper_tree_file_disables_the_keeper(setup):
    world, module, make, tmp_path = setup
    world.keeper_ready = True
    node = make(keeper_tree=str(tmp_path / 'absent.xml'))
    assert node.run() == module.EXIT_SUCCESS
    assert world.sent_trees == ['']
    assert 'arbre de comportement introuvable' in world.log_text()


def test_reference_profile_and_single_settings_reach_the_launch_command(setup, monkeypatch):
    world, module, make, _ = setup
    commands = []

    class FakeProcess:
        pid = 4242

        def poll(self):
            return None

    monkeypatch.setattr(module.subprocess, 'Popen',
                        lambda command, **options: commands.append(command) or FakeProcess())
    node = make(launch_bringup=True, drive_profile='reference', cruise_speed=0.7)
    monkeypatch.setattr(node, '_stop_stale_bringups', lambda: None)
    assert node.run() == module.EXIT_SUCCESS
    command = commands[0]
    assert command[:4] == ['ros2', 'launch', 'caytu_nav_bringup', 'solution_bringup.launch.py']
    assert 'cruise_speed:=0.7' in command and 'turn_speed:=0.8' in command
    assert 'camera_range:=2.5' in command and 'axle_offset:=0' in command
    assert 'path_keeper:=false' in command
    assert node._metrics.settings == 'reference + cruise_speed=0.7'
    assert 'Réglages de conduite : reference + cruise_speed=0.7' in world.log_text()


def test_default_run_only_states_the_keeper_switch_to_the_launch(setup, monkeypatch):
    world, module, make, _ = setup
    commands = []

    class FakeProcess:
        pid = 4242

        def poll(self):
            return None

    monkeypatch.setattr(module.subprocess, 'Popen',
                        lambda command, **options: commands.append(command) or FakeProcess())
    node = make(launch_bringup=True, keeper_wait_sec=0.5)
    monkeypatch.setattr(node, '_stop_stale_bringups', lambda: None)
    monkeypatch.setattr(node, '_resolve_keeper_tree', lambda: None)
    assert node.run() == module.EXIT_SUCCESS
    extra = [argument for argument in commands[0][4:]
             if not argument.startswith(('use_sim_time:=', 'map:=', 'task_params_file:='))]
    assert extra == ['path_keeper:=true']


def test_an_invalid_setting_stops_before_anything_is_started(setup):
    world, module, make, _ = setup
    with pytest.raises(ValueError) as error:
        make(cruise_speed=5.0)
    assert 'cruise_speed' in str(error.value)
    with pytest.raises(ValueError):
        make(drive_profile='turbo')


# --------------------------------------------------------------------------- #
# Démarrage de Nav2 bloqué
# --------------------------------------------------------------------------- #
def _fake_bringup(module, monkeypatch, world, ready_after_starts=None):
    """Remplace ros2 launch : compte les lancements et les arrêts.

    ready_after_starts : Nav2 devient prêt 2 s après ce numéro de lancement
    (None = jamais).
    """
    events = {'starts': 0, 'stops': 0}

    class FakeProcess:
        pid = 4242
        alive = True

        def poll(self):
            return None if self.alive else 0

    def popen(command, **options):
        events['starts'] += 1
        if ready_after_starts is not None and events['starts'] >= ready_after_starts:
            world.at(2.0, lambda: setattr(world, 'nav_ready', True))
        return FakeProcess()

    monkeypatch.setattr(module.subprocess, 'Popen', popen)
    return events


def test_a_stuck_nav2_startup_is_restarted_and_the_run_succeeds(setup, monkeypatch):
    world, module, make, _ = setup
    world.nav_ready = False
    events = _fake_bringup(module, monkeypatch, world, ready_after_starts=2)
    node = make(launch_bringup=True, nav2_startup_timeout_sec=20.0, path_keeper=False)
    monkeypatch.setattr(node, '_stop_stale_bringups', lambda: None)
    monkeypatch.setattr(node, 'stop_bringup', lambda: events.__setitem__('stops', events['stops'] + 1))
    start = world.t
    assert node.run() == module.EXIT_SUCCESS
    assert events == {'starts': 2, 'stops': 1}
    assert 20.0 < world.t - start < 20.0 + 2.0 + 5.0
    assert node._metrics.nav2_restarts == 1
    assert 'arrêt puis relance du bringup' in world.log_text()
    assert 'après 1 relance(s)' in node._metrics.to_text()


def test_startup_gives_up_after_the_allowed_restarts_with_growing_patience(setup, monkeypatch):
    world, module, make, _ = setup
    world.nav_ready = False
    events = _fake_bringup(module, monkeypatch, world)
    node = make(launch_bringup=True, nav2_startup_timeout_sec=10.0, nav2_startup_restarts=2,
                path_keeper=False)
    monkeypatch.setattr(node, '_stop_stale_bringups', lambda: None)
    monkeypatch.setattr(node, 'stop_bringup', lambda: events.__setitem__('stops', events['stops'] + 1))
    start = world.t
    assert node.run() == module.EXIT_FAILURE
    assert events == {'starts': 3, 'stops': 2}
    # 10 s, puis 20 s, puis 40 s : une machine lente a de plus en plus de temps.
    assert world.t - start == pytest.approx(70.0, abs=2.0)
    assert 'Démarrage impossible' in world.log_text()
    assert world.sent_goals == []


def test_a_bringup_that_dies_during_startup_is_restarted_at_once(setup, monkeypatch):
    world, module, make, _ = setup
    world.nav_ready = False
    events = _fake_bringup(module, monkeypatch, world, ready_after_starts=2)
    node = make(launch_bringup=True, nav2_startup_timeout_sec=60.0, path_keeper=False)
    monkeypatch.setattr(node, '_stop_stale_bringups', lambda: None)
    monkeypatch.setattr(node, 'stop_bringup', lambda: events.__setitem__('stops', events['stops'] + 1))
    world.at(3.0, lambda: setattr(node._bringup, 'alive', False))
    start = world.t
    assert node.run() == module.EXIT_SUCCESS
    assert events['starts'] == 2
    assert world.t - start < 15.0
    assert 'bringup s\'est arrêté pendant le démarrage' in world.log_text()


def test_startup_is_not_restarted_when_the_bringup_is_not_ours(setup):
    world, module, make, _ = setup
    world.nav_ready = False
    node = make(nav2_startup_timeout_sec=5.0)           # launch_bringup=False
    start = world.t
    assert node.run() == module.EXIT_FAILURE
    assert world.t - start == pytest.approx(5.0, abs=1.0)
    assert node._metrics.nav2_restarts == 0


def test_a_position_error_right_after_startup_is_resent_without_cleaning(setup):
    # Cas observé sur le banc d'essai : Nav2 vient de s'activer, son contrôleur
    # n'a pas encore reçu map -> odom, le premier but échoue en 60 ms.
    world, module, make, _ = setup
    world.nav_script = [
        {'status': 'ABORTED', 'error_code': 102, 'move_to': None, 'duration': 0.1},
        {},
    ]
    node = make()
    assert node.run() == module.EXIT_SUCCESS
    assert world.clears == []                           # aucune costmap effacée
    # Renvoi en moins d'une seconde : ni pause d'une seconde, ni attente d'une
    # costmap fraîche (plus de 2 s en tout).
    assert world.sent_times[1] - world.sent_times[0] < 0.1 + 0.3 + 0.4
    text = world.log_text()
    assert 'TF_ERROR' in text and 'rien à nettoyer' in text
    assert node._metrics.failures == ['TF_ERROR']


def test_repeated_position_errors_fall_back_to_the_normal_ladder(setup):
    world, module, make, _ = setup
    world.nav_script = [{'status': 'ABORTED', 'error_code': 202, 'move_to': None,
                         'duration': 0.1}] * 6 + [{}]
    node = make()
    assert node.run() == module.EXIT_SUCCESS
    # Cinq renvois simples, puis le sixième échec déclenche un nettoyage.
    assert len([name for name, _ in world.clears if name == LOCAL]) == 1


def test_an_immediate_unexplained_failure_is_resent_and_keeps_the_keeper(setup):
    # Observé sur le banc : au premier but, l'arbre crée son client vers
    # path_keeper ; la première demande n'est pas acquittée à temps.
    world, module, make, tmp_path = setup
    tree = _keeper_tree(tmp_path)
    world.keeper_ready = True
    world.nav_script = [{'status': 'ABORTED', 'move_to': None, 'duration': 0.5}, {}]
    node = make(keeper_tree=tree)
    assert node.run() == module.EXIT_SUCCESS
    assert world.clears == []
    assert world.sent_trees == [tree, tree]
    assert node._keeper_strikes == 0
    assert 'rien à nettoyer' in world.log_text()


def test_a_slow_unexplained_failure_is_still_cleaned_and_counted(setup):
    world, module, make, tmp_path = setup
    tree = _keeper_tree(tmp_path)
    world.keeper_ready = True
    world.nav_script = [{'status': 'ABORTED', 'move_to': None, 'duration': 3.0}, {}]
    node = make(keeper_tree=tree)
    assert node.run() == module.EXIT_SUCCESS
    assert world.clears != []
    assert node._keeper_strikes == 1


def test_blocked_near_an_occupied_goal_but_inside_the_circle_is_a_success(setup):
    # Une boîte est posée sur le but : le robot rejoint le point de repli,
    # puis bute sur la boîte. Son avant est déjà dans le cercle de 0,6 m.
    world, module, make, _ = setup
    _costmap_with_blocked_goal(world, radius=0.20)
    near = (GOAL[0], GOAL[1] - 0.62)                   # centre à 0,62 m, face au but
    world.robot = [SPAWN[0], SPAWN[1], math.pi / 2]
    world.nav_script = [
        {'status': 'ABORTED', 'error_code': 208, 'move_to': (GOAL[0], GOAL[1] - 0.8)},
        {'status': 'ABORTED', 'error_code': 104, 'move_to': near},
    ]
    node = make()
    assert node.run() == module.EXIT_SUCCESS
    assert node._on_fallback and len(world.sent_goals) == 2
    assert 'BUT ATTEINT par repli' in world.log_text()


def test_blocked_far_from_an_occupied_goal_keeps_trying(setup):
    world, module, make, _ = setup
    _costmap_with_blocked_goal(world, radius=0.20)
    world.robot = [SPAWN[0], SPAWN[1], math.pi / 2]
    world.nav_script = [
        {'status': 'ABORTED', 'error_code': 208, 'move_to': (GOAL[0], GOAL[1] - 1.5)},
        {'status': 'ABORTED', 'error_code': 104, 'move_to': (GOAL[0], GOAL[1] - 1.2)},
        {},
    ]
    node = make()
    assert node.run() == module.EXIT_SUCCESS
    assert len(world.sent_goals) == 3                     # le robot hors du cercle insiste


def test_a_slow_startup_that_keeps_progressing_is_not_restarted(setup, monkeypatch):
    # Démarrage lent : un nœud change d'état toutes les 15 s, 75 s en tout,
    # avec un délai de 20 s. Rien n'est relancé.
    world, module, make, _ = setup
    world.nav_ready = False
    events = _fake_bringup(module, monkeypatch, world)
    for k, delay in enumerate((15.0, 30.0, 45.0, 60.0)):
        world.at(delay, lambda k=k: setattr(world, 'lifecycle_state', 2 + k % 2))
    world.at(75.0, lambda: setattr(world, 'nav_ready', True))
    node = make(launch_bringup=True, nav2_startup_timeout_sec=20.0, path_keeper=False)
    monkeypatch.setattr(node, '_stop_stale_bringups', lambda: None)
    monkeypatch.setattr(node, 'stop_bringup', lambda: events.__setitem__('stops', events['stops'] + 1))
    assert node.run() == module.EXIT_SUCCESS
    assert events == {'starts': 1, 'stops': 0}
    assert world.state_requests > 0


def test_a_startup_stuck_after_some_progress_is_restarted(setup, monkeypatch):
    world, module, make, _ = setup
    world.nav_ready = False
    events = _fake_bringup(module, monkeypatch, world, ready_after_starts=2)
    world.at(5.0, lambda: setattr(world, 'lifecycle_state', 2))     # puis plus rien
    node = make(launch_bringup=True, nav2_startup_timeout_sec=20.0, path_keeper=False)
    monkeypatch.setattr(node, '_stop_stale_bringups', lambda: None)
    monkeypatch.setattr(node, 'stop_bringup', lambda: events.__setitem__('stops', events['stops'] + 1))
    start = world.t
    assert node.run() == module.EXIT_SUCCESS
    assert events == {'starts': 2, 'stops': 1}
    # Relance 20 s après le dernier changement d'état, pas 20 s après le lancement.
    assert world.sent_times[0] - start == pytest.approx(5.0 + 20.0 + 2.0, abs=2.0)
    assert 'Démarrage de Nav2 bloqué' in world.log_text()


def test_goals_refused_for_too_long_restart_the_bringup(setup, monkeypatch):
    # Nav2 se dit actif mais refuse tous les buts (activation à moitié faite).
    world, module, make, _ = setup
    events = _fake_bringup(module, monkeypatch, world)
    world.nav_script = [{'reject': True}] * 200 + [{}]
    node = make(launch_bringup=True, nav2_startup_timeout_sec=10.0, path_keeper=False)
    monkeypatch.setattr(node, '_stop_stale_bringups', lambda: None)

    def stop():
        events['stops'] += 1
        world.nav_script = [{}]                         # après la relance, tout va bien
    monkeypatch.setattr(node, 'stop_bringup', stop)
    assert node.run() == module.EXIT_SUCCESS
    assert events == {'starts': 2, 'stops': 1}
    assert 'Nav2 refuse le but depuis' in world.log_text()


def test_two_immediate_failures_with_the_keeper_switch_it_off(setup):
    # L'arbre du chemin stabilisé n'arrive pas à démarrer (échec sans cause,
    # aussitôt) : après deux fois, l'arbre par défaut prend le relais.
    world, module, make, tmp_path = setup
    tree = _keeper_tree(tmp_path)
    world.keeper_ready = True
    world.nav_script = [{'status': 'ABORTED', 'move_to': None, 'duration': 0.2}] * 2 + [{}]
    node = make(keeper_tree=tree)
    assert node.run() == module.EXIT_SUCCESS
    assert world.sent_trees == [tree, tree, '']


def test_planner_timeouts_with_the_keeper_count_against_it(setup):
    world, module, make, tmp_path = setup
    tree = _keeper_tree(tmp_path)
    world.keeper_ready = True
    world.nav_script = [{'status': 'ABORTED', 'error_code': 207, 'move_to': None}] * 2 + [{}]
    node = make(keeper_tree=tree)
    assert node.run() == module.EXIT_SUCCESS
    assert world.sent_trees == [tree, tree, '']
