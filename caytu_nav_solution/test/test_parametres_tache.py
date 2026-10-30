"""Tests de lecture de task_params.yaml (sans ROS)."""
import pytest

from caytu_nav_solution.parametres_tache import charger_parametres_tache

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


def test_charger_parametres_tache(tmp_path):
    chemin = tmp_path / "task_params.yaml"
    chemin.write_text(CONTENU_VALIDE, encoding="utf-8")

    parametres = charger_parametres_tache(str(chemin))

    assert parametres.depart.x == pytest.approx(-0.200546)
    assert parametres.depart.y == pytest.approx(-7.485170)
    assert parametres.depart.yaw == pytest.approx(1.571)
    assert parametres.but_x == pytest.approx(-2.293350)
    assert parametres.but_y == pytest.approx(2.232090)
    assert parametres.chemin == str(chemin)


def test_fichier_invalide_leve_une_erreur_claire(tmp_path):
    # YAML valide, mais il manque des clés obligatoires (y, yaw, goal_x...).
    chemin = tmp_path / "task_params.yaml"
    chemin.write_text("/**:\n  ros__parameters:\n    x: 1.0\n", encoding="utf-8")

    with pytest.raises(ValueError, match="task_params.yaml invalide"):
        charger_parametres_tache(str(chemin))
