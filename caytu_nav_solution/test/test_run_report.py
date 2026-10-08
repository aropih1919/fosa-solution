"""Tests du rapport de trajet : calculs purs, sans ROS."""
import csv
import json
import math

import pytest

from caytu_nav_solution.run_report import RunMetrics, append_csv_row

GOAL = (10.0, 10.0)


def _feed(metrics, samples, t0=0.0, dt=0.2):
    """samples : liste de (x, y, cap) prises toutes les dt secondes."""
    t = t0
    for x, y, yaw in samples:
        metrics.update_pose(t, x, y, *GOAL, yaw)
        t += dt
    return t


def test_square_perimeter_is_four_meters():
    m = RunMetrics()
    m.start(0.0, 0.0)
    _feed(m, [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (0.0, 1.0, 0.0),
              (0.0, 0.0, 0.0)], dt=1.0)
    assert m.path_length == pytest.approx(4.0)
    assert m.first_motion_t is not None


def test_time_is_split_between_startup_driving_pivots_and_stops():
    m = RunMetrics()
    m.start(0.0, 0.0)
    samples = [(0.0, 0.0, 0.0)] * 26                                   # 5 s immobile : démarrage
    samples += [(0.1 * k, 0.0, 0.0) for k in range(1, 21)]            # 4 s à 0,5 m/s
    samples += [(2.0, 0.0, 0.16 * k) for k in range(1, 11)]           # 2 s de pivot à 0,8 rad/s
    samples += [(2.0, 0.0, 1.6)] * 15                                  # 3 s à l'arrêt
    samples += [(2.0, 0.1 * k, 1.6) for k in range(1, 11)]            # 2 s à 0,5 m/s
    end = _feed(m, samples)
    m.finish('success', end, end, 0.05)
    data = m.to_dict()
    assert data['first_motion_s'] == pytest.approx(5.0, abs=0.21)
    assert data['moving_time_s'] == pytest.approx(6.0, abs=0.21)
    assert data['pivot_time_s'] == pytest.approx(2.0, abs=0.21)
    assert data['stopped_time_s'] == pytest.approx(3.0, abs=0.21)
    assert (data['pivots'], data['stops']) == (1, 1)
    assert data['path_length_m'] == pytest.approx(3.0, abs=0.01)


def test_a_pivot_around_the_axle_is_not_counted_as_driving():
    # base_footprint est à 9,5 cm de l'essieu : il décrit un petit arc pendant
    # un pivot. Ce déplacement ne doit pas compter comme du roulage.
    m = RunMetrics()
    m.start(0.0, 0.0)
    samples = [(-0.095 * math.cos(0.16 * k), -0.095 * math.sin(0.16 * k), 0.16 * k)
               for k in range(0, 11)]
    _feed(m, samples)
    assert m.pivot_time == pytest.approx(2.0, abs=0.01)
    assert m.moving_time == 0.0 and m.path_length == 0.0


def test_short_pauses_are_not_counted_as_stops():
    m = RunMetrics()
    m.start(0.0, 0.0)
    samples = [(0.1 * k, 0.0, 0.0) for k in range(0, 6)] + [(0.5, 0.0, 0.0)] * 2
    samples += [(0.5 + 0.1 * k, 0.0, 0.0) for k in range(1, 6)]
    _feed(m, samples)
    assert m.stops == 0 and m.stopped_time == pytest.approx(0.4)


def test_nav2_ready_time_is_reported_once():
    m = RunMetrics()
    m.start(100.0, 0.0)
    m.mark_nav2_ready(112.5)
    m.mark_nav2_ready(130.0)
    assert m.to_dict()['nav2_ready_s'] == pytest.approx(12.5)


def test_floor_contact_ignored_table_counted():
    m = RunMetrics()
    m.add_contact('wheel_link', 'cafe_floor')
    assert m.contacts == 0
    m.add_contact('base_link', 'table_leg')
    assert m.contacts == 1
    assert 'table_leg' in m.contact_names


def test_to_dict_is_json_serializable_and_records_the_settings():
    m = RunMetrics()
    m.start(0.0, 0.0)
    m.settings = 'reference + cruise_speed=0.7'
    m.update_pose(0.0, 0.0, 0.0, 1.0, 0.0)
    m.update_pose(1.0, 0.1, 0.0, 1.0, 0.0)
    m.add_attempt()
    m.add_rejection()
    m.finish('success', 10.0, 12.0, 0.9)
    data = m.to_dict()
    json.dumps(data)
    assert data['attempts'] == 1 and data['rejections'] == 1
    assert data['settings'] == 'reference + cruise_speed=0.7'
    assert 'reference + cruise_speed=0.7' in m.to_text()


def test_text_report_survives_missing_values():
    m = RunMetrics()
    m.contacts_available = False
    text = m.to_text()
    assert 'contacts : non mesurés' in text and 'réglages : par défaut' in text


def test_fallback_defaults_and_serialization():
    m = RunMetrics()
    data = m.to_dict()
    assert data['fallback_used'] is False and data['fallback_offset_m'] is None
    m.fallback_used = True
    m.fallback_offset_m = 0.42
    json.dumps(m.to_dict())
    assert 'repli : oui (0.42 m du centre)' in m.to_text()


def test_failure_causes_are_reported():
    m = RunMetrics()
    data = m.to_dict()
    assert data['failures'] == 0 and data['failure_causes'] == ''
    assert 'interruptions : aucune' in m.to_text()
    for cause in ('NO_VALID_PATH', 'NONE', 'NO_VALID_PATH'):
        m.add_failure(cause)
    data = m.to_dict()
    assert data['failures'] == 3
    assert data['failure_causes'] == 'NONE x1; NO_VALID_PATH x2'
    assert 'NO_VALID_PATH x2' in m.to_text()
    json.dumps(data)


def test_csv_rows_never_shift_when_columns_change(tmp_path):
    path = str(tmp_path / 'trajets.csv')
    append_csv_row(path, {'result': 'success', 'attempts': 1})
    append_csv_row(path, {'result': 'timeout', 'attempts': 4})
    with open(path, newline='', encoding='utf-8') as stream:
        assert list(csv.reader(stream)) == [
            ['result', 'attempts'], ['success', '1'], ['timeout', '4']]
    # Nouvelle version du rapport : une colonne de plus. L'ancien fichier est
    # mis de côté, le nouveau repart avec le bon en-tête.
    append_csv_row(path, {'result': 'success', 'attempts': 1, 'failures': 0})
    with open(path, newline='', encoding='utf-8') as stream:
        assert list(csv.reader(stream)) == [
            ['result', 'attempts', 'failures'], ['success', '1', '0']]
    with open(str(tmp_path / 'trajets.ancien1.csv'), newline='', encoding='utf-8') as stream:
        assert len(list(csv.reader(stream))) == 3


def test_durations_are_measured_from_the_start_of_the_solution():
    # L'horloge simulée tournait déjà depuis 120 s quand la solution démarre.
    m = RunMetrics()
    m.start(120.0, 0.0)
    m.mark_nav2_ready(124.0)
    end = _feed(m, [(0.0, 0.0, 0.0)] * 30 + [(0.1 * k, 0.0, 0.0) for k in range(1, 11)], t0=120.0)
    m.finish('success', end, 9.0, 0.1)
    data = m.to_dict()
    assert data['nav2_ready_s'] == pytest.approx(4.0)
    assert data['first_motion_s'] == pytest.approx(5.8, abs=0.21)
    assert data['sim_duration_s'] == pytest.approx(end - 120.0)
