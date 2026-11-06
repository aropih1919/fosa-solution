"""Tests des calculs purs : s'exécutent avec `pytest`, sans ROS ni Gazebo."""

import math

import pytest

from caytu_nav_solution.nav_math import (
    Pose2D,
    lacet_depuis_quaternion,
    normaliser_angle,
    quaternion_depuis_lacet,
)


def test_pose_composer_et_inverse_coherents():
    a = Pose2D(1.0, -2.0, 0.7)
    b = Pose2D(0.3, 0.4, -1.2)
    identite = a.composer(a.inverse())
    assert identite.x == pytest.approx(0.0, abs=1e-12)
    assert identite.y == pytest.approx(0.0, abs=1e-12)
    assert identite.yaw == pytest.approx(0.0, abs=1e-12)
    retour = a.inverse().composer(a.composer(b))
    assert (retour.x, retour.y, retour.yaw) == pytest.approx((b.x, b.y, b.yaw))


def test_quaternions_aller_retour():
    for lacet in (-3.0, -1.0, 0.0, 0.5, 3.1):
        assert lacet_depuis_quaternion(
            *quaternion_depuis_lacet(lacet)) == pytest.approx(lacet)
    # Quaternion non trivial (z = 1, w = 0) : demi-tour autour de z.
    assert lacet_depuis_quaternion(0.0, 0.0, 1.0, 0.0) == pytest.approx(math.pi)


def test_les_angles_sont_normalises():
    assert normaliser_angle(3.0 * math.pi) == pytest.approx(math.pi)
    assert normaliser_angle(-3.0 * math.pi) == pytest.approx(-math.pi)
    assert normaliser_angle(0.5) == pytest.approx(0.5)


def test_la_distance_est_euclidienne():
    pose = Pose2D(1.0, 2.0, 0.0)
    assert pose.distance_vers(4.0, 6.0) == pytest.approx(5.0)
