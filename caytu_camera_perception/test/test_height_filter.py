"""Test offline du filtrage geometrique (numpy only, sans ROS ni Gazebo)."""

import numpy as np

from caytu_camera_perception.point_filter import filter_obstacle_points


def test_garde_la_bande_de_hauteur():
    pts = np.array([
        [1.0, 0.0, 0.05],   # trop bas (sol / roues, vu par le LiDAR)
        [1.0, 0.0, 0.5],    # plateau de table -> garde
        [1.0, 0.0, 1.1],    # buste -> garde
        [1.0, 0.0, 1.8],    # plafond -> jette
        [5.0, 0.0, 0.5],    # trop loin (bruit D435) -> jette
        [1.0, 0.0, np.nan],  # bruit capteur -> jette
    ])
    kept, stats = filter_obstacle_points(pts)
    assert stats == {'total': 6, 'kept': 2}
    assert kept[:, 2].tolist() == [float(np.float32(0.5)), float(np.float32(1.1))]


def test_nuage_vide_ou_bruite():
    kept, stats = filter_obstacle_points(np.zeros((0, 3)))
    assert stats['kept'] == 0
    kept, stats = filter_obstacle_points(np.full((10, 3), np.inf))
    assert stats['kept'] == 0
