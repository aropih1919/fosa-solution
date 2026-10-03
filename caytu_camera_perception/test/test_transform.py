"""Test offline de la matrice TF (numpy only)."""

import math
import numpy as np
from types import SimpleNamespace as NS

from caytu_camera_perception.height_filter_node import transform_to_matrix


def _tf(px, py, pz, qx, qy, qz, qw):
    return NS(transform=NS(rotation=NS(x=qx, y=qy, z=qz, w=qw),
                           translation=NS(x=px, y=py, z=pz)))


def test_identite():
    m = transform_to_matrix(_tf(0, 0, 0, 0, 0, 0, 1))
    assert np.allclose(m, np.eye(4))


def test_translation_plus_rotation_90deg_z():
    a = math.pi / 2
    m = transform_to_matrix(_tf(1, 0, 0, 0, 0, math.sin(a / 2), math.cos(a / 2)))
    p = (m[:3, :3] @ np.array([1.0, 0.0, 0.0])) + m[:3, 3]
    assert np.allclose(p, [1.0, 1.0, 0.0])
