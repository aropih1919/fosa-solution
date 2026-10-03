"""Test offline de la politique d'inflation (sans ROS)."""

import math

from caytu_nav_surcouche.inflation_policy import target_inflation


def test_table_politique():
    assert target_inflation(0.0, 0.5) == 0.35  # lent -> prudent
    assert target_inflation(0.5, 0.5) == 0.35  # encombre -> prudent
    assert target_inflation(0.5, 5.0) == 0.20  # vite + degage -> ouvert
    assert target_inflation(0.2, 1.5) == 0.30  # entre-deux -> compromis
    assert target_inflation(0.5, math.inf) == 0.20  # rien devant -> ouvert
