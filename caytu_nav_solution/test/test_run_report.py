"""Tests du rapport de trajet : calculs purs, sans ROS."""
import json

from caytu_nav_solution.run_report import RunMetrics


def test_square_perimeter_is_four_meters():
    m = RunMetrics()
    m.start(0.0, 0.0)
    goal = (10.0, 10.0)
    pts = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]
    t = 0.0
    for x, y in pts:
        m.update_pose(t, x, y, *goal)
        t += 1.0
    assert m.path_length == json.loads(json.dumps({"v": m.path_length}))["v"]
    assert abs(m.path_length - 4.0) < 1e-9
    assert m.first_motion_t is not None


def test_floor_contact_ignored_table_counted():
    m = RunMetrics()
    m.add_contact("wheel_link", "cafe_floor")
    assert m.contacts == 0
    m.add_contact("base_link", "table_leg")
    assert m.contacts == 1
    assert "table_leg" in m.contact_names


def test_to_dict_is_json_serializable():
    m = RunMetrics()
    m.start(0.0, 0.0)
    m.update_pose(0.0, 0.0, 0.0, 1.0, 0.0)
    m.update_pose(1.0, 0.1, 0.0, 1.0, 0.0)
    m.add_attempt()
    m.add_rejection()
    m.finish("success", 10.0, 12.0, 0.9)
    data = m.to_dict()
    json.dumps(data)
    assert data["attempts"] == 1 and data["rejections"] == 1
