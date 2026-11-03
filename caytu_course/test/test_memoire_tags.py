"""Mémoire des tags observés dans le repère map."""

import math

import pytest

from caytu_course.memoire_tags import MemoireTags


def test_un_tag_observe_est_connu():
    memoire = MemoireTags(distance_max=4.0)
    assert not memoire.connait(7)
    assert memoire.ajouter(7, 1.0, 2.0, math.pi, 1.5)
    assert memoire.connait(7)
    pose = memoire.pose(7)
    assert (pose.x, pose.y) == pytest.approx((1.0, 2.0))
    assert pose.nombre == 1
    assert pose.distance_min == pytest.approx(1.5)


def test_les_observations_proches_comptent_plus():
    memoire = MemoireTags(distance_max=4.0)
    memoire.ajouter(4, 0.0, 0.0, 0.0, 1.2)   # poids 1/1,44
    memoire.ajouter(4, 1.0, 0.0, 0.0, 1.0)   # poids 1
    assert memoire.pose(4).x == pytest.approx(1.0 / (1.0 + 1.0 / 1.44))
    assert memoire.pose(4).distance_min == pytest.approx(1.0)


def test_la_normale_est_une_moyenne_circulaire():
    memoire = MemoireTags(distance_max=4.0)
    memoire.ajouter(1, 0.0, 0.0, math.radians(179.0), 1.0)
    memoire.ajouter(1, 0.0, 0.0, math.radians(-179.0), 1.0)
    assert abs(math.degrees(memoire.pose(1).cap)) == pytest.approx(180.0, abs=1e-6)


def test_une_observation_trop_lointaine_est_ignoree():
    memoire = MemoireTags(distance_max=4.0)
    assert not memoire.ajouter(9, 0.0, 0.0, 0.0, 4.5)
    assert not memoire.connait(9)


def test_oublier_un_tag():
    memoire = MemoireTags(distance_max=4.0)
    memoire.ajouter(9, 0.0, 0.0, 0.0, 1.0)
    memoire.oublier(9)
    assert not memoire.connait(9)
    assert memoire.ids() == []


def test_export_et_import():
    memoire = MemoireTags(distance_max=4.0)
    memoire.ajouter(7, 1.0, 2.0, 0.5, 1.0)
    memoire.ajouter(4, -1.0, 0.5, -2.0, 2.0)
    copie = MemoireTags.importer(memoire.exporter(), distance_max=4.0)
    assert copie.ids() == [4, 7]
    assert copie.pose(7).x == pytest.approx(1.0)
    assert copie.pose(4).cap == pytest.approx(-2.0)
    # Une observation ajoutée après import se fond dans la mémoire importée.
    copie.ajouter(7, 1.0, 2.0, 0.5, 1.0)
    assert copie.pose(7).nombre == 2


def test_une_vue_bien_plus_proche_remplace_les_vues_lointaines():
    """Mesuré en simulation : les vues à 1,5 m dispersaient la pose de 0,6 m."""
    memoire = MemoireTags(distance_max=4.0)
    memoire.ajouter(4, 0.0, 0.0, 0.0, 2.0)
    memoire.ajouter(4, 0.6, 0.0, 0.5, 1.9)
    memoire.ajouter(4, 1.0, 1.0, 1.0, 0.8)
    pose = memoire.pose(4)
    assert (pose.x, pose.y, pose.cap) == pytest.approx((1.0, 1.0, 1.0))
    assert pose.nombre == 1


def test_une_vue_lointaine_n_abime_pas_une_pose_vue_de_pres():
    memoire = MemoireTags(distance_max=4.0)
    memoire.ajouter(4, 1.0, 1.0, 1.0, 0.8)
    assert not memoire.ajouter(4, 3.0, 3.0, 0.0, 2.0)
    assert memoire.pose(4).x == pytest.approx(1.0)
    assert memoire.ajouter(4, 1.1, 1.0, 1.0, 1.0)
    assert memoire.pose(4).nombre == 2
