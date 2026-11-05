"""Tests de lecture de task_params.yaml (sans ROS)."""
import pytest

from caytu_nav_solution.task_params import load_task_params

CONTENU_VALIDE = """\
/**:
  ros__parameters:
    x: -0.200546
    y: -7.485170
    z: 0.220794
    yaw: 1.571
    goal_x: -2.293350
    goal_y: 2.232090
    goal_z: 0.220042
"""


def test_load_task_params(tmp_path):
    chemin = tmp_path / "task_params.yaml"
    chemin.write_text(CONTENU_VALIDE, encoding="utf-8")

    params = load_task_params(str(chemin))

    assert params.spawn.x == pytest.approx(-0.200546)
    assert params.spawn.y == pytest.approx(-7.485170)
    assert params.spawn.yaw == pytest.approx(1.571)
    assert params.goal_x == pytest.approx(-2.293350)
    assert params.goal_y == pytest.approx(2.232090)
    assert params.path == str(chemin)


def test_fichier_invalide_leve_une_erreur_claire(tmp_path):
    # YAML valide, mais il manque des clés obligatoires (y, yaw, goal_x...).
    chemin = tmp_path / "task_params.yaml"
    chemin.write_text("/**:\n  ros__parameters:\n    x: 1.0\n", encoding="utf-8")

    with pytest.raises(ValueError, match="task_params.yaml invalide"):
        load_task_params(str(chemin))
