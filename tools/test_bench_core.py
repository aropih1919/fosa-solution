#!/usr/bin/env python3
"""Tests de bench_core.py. Lancer : python3 tools/test_bench_core.py"""

import math

import bench_core as core


def close(a, b, tol=1e-6):
    return abs(a - b) < tol


def test_yaw():
    # Quaternion d'une rotation de 90 degrés autour de Z
    yaw = core.yaw_from_quaternion(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    assert close(yaw, math.pi / 2)
    assert close(core.yaw_from_quaternion(0, 0, 0, 1), 0.0)


def test_distance_but_devant():
    # Robot en (0,0) tourné vers +x, but à 1 m devant : l'avant du robot est à 0.198 m
    d = core.distance_to_robot_body(1.0, 0.0, 0.0, 0.0, 0.0)
    assert close(d, 1.0 - core.ROBOT_X_MAX)


def test_distance_but_derriere():
    d = core.distance_to_robot_body(-1.0, 0.0, 0.0, 0.0, 0.0)
    assert close(d, 1.0 + core.ROBOT_X_MIN)


def test_distance_but_sous_le_robot():
    assert close(core.distance_to_robot_body(0.05, 0.05, 0.0, 0.0, 0.0), 0.0)


def test_distance_robot_tourne():
    # Robot tourné de 90 degrés : son avant pointe vers +y
    d = core.distance_to_robot_body(0.0, 1.0, 0.0, 0.0, math.pi / 2)
    assert close(d, 1.0 - core.ROBOT_X_MAX)


def test_distance_but_sur_le_cote():
    d = core.distance_to_robot_body(0.0, 1.0, 0.0, 0.0, 0.0)
    assert close(d, 1.0 - core.ROBOT_Y_HALF)


def test_contacts_ignores():
    ignore = ['ground', 'floor', 'plane']
    assert not core.is_real_contact('sitoe_robot::right_wheel_link::col', 'ground_plane::link::col', ignore)
    assert not core.is_real_contact('sitoe_robot::base_link::col', 'sitoe_robot::top_chassis::col', ignore)
    assert core.is_real_contact('sitoe_robot::base_link::col', 'WoodenChair::link::col', ignore)
    assert core.is_real_contact('sitoe_robot::base_link::col', 'stadium::walls::col', ignore)


def test_episodes_de_contact():
    counter = core.ContactCounter(gap_sec=1.0)
    # 10 messages à 5 Hz = un seul épisode
    for i in range(10):
        counter.add(10.0 + i * 0.2)
    assert counter.episodes == 1
    assert counter.messages == 10
    # 5 secondes plus tard : nouvel épisode
    assert counter.add(20.0) is True
    assert counter.episodes == 2


def test_synthese():
    runs = [
        {'label': 'a', 'result': 'SUCCESS', 'time_sec': 100.0, 'final_center_distance': 0.3,
         'contacts_episodes': 0, 'recoveries': 0, 'git_commit': 'abc'},
        {'label': 'a', 'result': 'TIMEOUT', 'time_sec': 600.0, 'final_center_distance': 4.0,
         'contacts_episodes': 2, 'recoveries': 4, 'git_commit': 'abc'},
        {'label': 'b', 'result': 'SUCCESS', 'time_sec': 80.0, 'final_center_distance': 0.2,
         'contacts_episodes': None, 'recoveries': 1, 'git_commit': 'def'},
    ]
    rows = core.summarize_runs(runs)
    assert len(rows) == 2
    row_a = rows[0]
    assert row_a['runs'] == 2 and row_a['success'] == 1
    assert row_a['success_rate'] == 0.5
    assert row_a['time_median_s'] == 100.0          # seulement les runs réussis
    assert row_a['final_dist_mean_m'] == 2.15
    assert row_a['contacts_mean'] == 1.0
    assert rows[1]['contacts_mean'] is None          # contacts non mesurés
    assert 'label' in core.format_table(rows)


if __name__ == '__main__':
    tests = [value for name, value in sorted(globals().items()) if name.startswith('test_')]
    for test in tests:
        test()
        print(f'OK  {test.__name__}')
    print(f'\n{len(tests)} tests passés.')
