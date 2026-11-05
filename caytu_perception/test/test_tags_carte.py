"""Registre des tags : normale côté caméra, filtrage, fichier."""

import math

from caytu_perception.tags_carte import (
    ObservationTag, RegistreTags, lacet_normale)


def _quaternion_axe_z_vers(lacet):
    """Rotation qui envoie l'axe z sur l'horizontale de cap `lacet`."""
    # Rotation de +90° autour de y (z -> x), puis du cap autour de z.
    qy = (0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4))
    qz = (0.0, 0.0, math.sin(lacet / 2), math.cos(lacet / 2))
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qy
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def test_normal_points_towards_the_camera_whatever_the_detector_convention():
    tag, camera = (2.0, 0.0, 1.0), (0.5, 0.2)
    vers_mur = _quaternion_axe_z_vers(0.0)          # axe z vers le mur (+x)
    vers_camera = _quaternion_axe_z_vers(math.pi)   # axe z vers la caméra (-x)
    for quaternion in (vers_mur, vers_camera):
        assert math.isclose(abs(lacet_normale(quaternion, tag, camera)), math.pi, abs_tol=1e-6)
    # Sans position de caméra, on garde l'axe z tel quel.
    assert math.isclose(lacet_normale(vers_mur, tag), 0.0, abs_tol=1e-6)


def test_registry_filters_outliers_and_round_trips_through_yaml(tmp_path):
    registre = RegistreTags(identifiants_attendus=(1, 4, 7, 9), distance_max=4.0)
    for i in range(8):
        bruit = 0.01 * ((-1) ** i)
        assert registre.ajouter(ObservationTag(7, 2.0 + bruit, 1.0 - bruit, 1.0,
                                               math.pi + bruit, distance=1.5))
    # Vue lointaine aberrante : refusée. Vue proche : toujours gardée (la
    # médiane des vues proches l'absorbe si elle est isolée).
    assert not registre.ajouter(ObservationTag(7, 4.0, 3.0, 1.0, 0.0, distance=3.0))
    assert registre.ajouter(ObservationTag(7, 2.4, 1.3, 1.0, math.pi, distance=1.0))
    # Tag vu trop de biais : refusé.
    assert not registre.ajouter(ObservationTag(9, 0.0, 0.0, 1.0, 0.0, distance=1.0,
                                               angle_vue=math.radians(80)))
    assert not registre.ajouter(ObservationTag(3, 0.0, 0.0, 1.0, 0.0, distance=1.0))
    assert not registre.ajouter(ObservationTag(1, 0.0, 0.0, 1.0, 0.0, distance=6.0))
    estimation = registre.estimation(7)
    assert estimation.observations == 9
    assert abs(estimation.x - 2.0) < 0.02 and abs(estimation.y - 1.0) < 0.02
    x, y, cap = estimation.pose_visite(0.5)
    assert abs(x - 1.5) < 0.03 and abs(y - 1.0) < 0.03 and abs(cap) < 0.03

    fichier = tmp_path / 'tags.yaml'
    registre.enregistrer(fichier)
    relu = RegistreTags()
    assert relu.charger(fichier) == 1
    copie = relu.estimation(7)
    assert abs(copie.x - estimation.x) < 1e-3 and copie.observations == 9
    assert RegistreTags().charger(tmp_path / 'absent.yaml') == 0


def test_nan_observations_never_poison_the_estimate():
    registre = RegistreTags()
    assert registre.ajouter(ObservationTag(4, 1.0, 2.0, 1.0, 0.5, distance=1.2))
    assert not registre.ajouter(ObservationTag(4, float('nan'), 2.0, 1.0, 0.5, distance=1.2))
    assert not registre.ajouter(ObservationTag(4, 1.0, 2.0, 1.0, float('nan'), distance=1.2))
    estimation = registre.estimation(4)
    assert math.isfinite(estimation.lacet_normale) and estimation.observations == 1
