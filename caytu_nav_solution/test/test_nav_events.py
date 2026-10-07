"""Tests du detecteur pur : temps fournis a la main, sans ROS."""
from caytu_nav_solution.nav_events import NavEventDetector


def test_status_goal_done_stop_fail():
    det = NavEventDetector()
    assert det.on_status(1, 0.0)[0][1] == "GOAL"
    assert det.goal_active is True
    assert det.on_status(1, 0.5) == []
    assert det.on_status(4, 1.0)[0][1] == "DONE"
    assert det.goal_active is False
    assert det.on_status(5, 2.0)[0][1] == "STOP"
    assert det.on_status(6, 3.0)[0][1] == "FAIL"


def test_plan_empty_is_fail():
    det = NavEventDetector()
    events = det.on_plan(0, 0.0, 0.0)
    assert events and events[0][1] == "FAIL"


def test_plan_throttle_then_replan():
    det = NavEventDetector()
    assert det.on_plan(100, 5.00, 0.0)[0][1] == "PLAN"
    assert det.replans == 1
    assert det.on_plan(100, 5.02, 0.5) == []
    assert det.on_plan(100, 5.40, 2.0)[0][1] == "PLAN"
    assert det.replans == 2


def test_feedback_time_and_distance():
    det = NavEventDetector()
    assert det.on_feedback(5.00, 0.0)[0][1] == "MOVE"
    assert det.on_feedback(4.95, 1.0) == []
    assert det.on_feedback(4.95, 3.5)[0][1] == "MOVE"
    assert det.on_feedback(4.70, 4.0)[0][1] == "MOVE"


def test_spin_start_remind_and_end():
    det = NavEventDetector()
    assert det.on_cmd(0.0, 0.5, 0.0)[0][1] == "SPIN"
    assert det.on_cmd(0.0, 0.5, 7.0)[0][1] == "SPIN"
    assert det.on_cmd(0.0, 0.0, 8.0)[0][1] == "SPIN"
    assert det.spin_time > 0.8


def test_stop_after_four_seconds():
    det = NavEventDetector()
    det.on_status(1, 0.0)
    assert det.on_cmd(0.0, 0.0, 0.0) == []
    assert det.on_cmd(0.0, 0.0, 4.5)[0][1] == "STOP"
    assert det.on_cmd(0.0, 0.0, 8.0)[0][1] == "STOP"


def test_move_every_two_seconds():
    det = NavEventDetector()
    assert det.on_cmd(0.3, 0.0, 0.0)[0][1] == "MOVE"
    assert det.on_cmd(0.3, 0.0, 1.0) == []
    assert det.on_cmd(0.3, 0.0, 2.5)[0][1] == "MOVE"
