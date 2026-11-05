"""Course finale : ordre des tags, géométrie de visite, approche, garde-fou."""

import math

from caytu_nav_solution.course_logique import (
    choisir_point_exploration, commande_alignement, commande_approche, distance_libre, ecarts_visite,
    extraire_ordre, points_exploration, pose_devant_tag)
from caytu_nav_solution.nav_math import Pose2D

ATTENDUS = (1, 4, 7, 9)


class FauxString:
    def __init__(self, data):
        self.data = data

    @staticmethod
    def get_fields_and_field_types():
        return {'data': 'string'}


class FauxTableau:
    def __init__(self, data):
        self.layout = object()
        self.data = data

    @staticmethod
    def get_fields_and_field_types():
        return {'layout': 'std_msgs/MultiArrayLayout', 'data': 'sequence<int32>'}


def test_order_is_read_from_any_message_type():
    assert extraire_ordre(FauxString('7,1,9,4'), ATTENDUS) == [7, 1, 9, 4]
    assert extraire_ordre(FauxString('[9, 4, 1, 7]'), ATTENDUS) == [9, 4, 1, 7]
    assert extraire_ordre(FauxString('ordre : 4 7 1 9'), ATTENDUS) == [4, 7, 1, 9]
    assert extraire_ordre(FauxTableau([1, 9, 7, 4]), ATTENDUS) == [1, 9, 7, 4]
    assert extraire_ordre(FauxTableau(bytes([7, 4, 9, 1])), ATTENDUS) == [7, 4, 9, 1]
    # Identifiants inconnus et doublons ignorés.
    assert extraire_ordre(FauxString('7, 3, 7, 1, 12, 9, 4'), ATTENDUS) == [7, 1, 9, 4]
    assert extraire_ordre(FauxString(''), ATTENDUS) == []


def test_visit_pose_is_half_a_metre_in_front_facing_the_tag():
    # Tag au mur x = 2, visible depuis -x (normale vers -x).
    pose = pose_devant_tag(2.0, 1.0, math.pi, 0.5)
    assert math.isclose(pose.x, 1.5) and math.isclose(pose.y, 1.0, abs_tol=1e-9)
    assert math.isclose(abs(pose.yaw), 0.0, abs_tol=1e-9)        # regarde vers +x
    distance, angle, lateral = ecarts_visite(pose, 2.0, 1.0, math.pi, 0.5)
    assert abs(distance) < 1e-9 and abs(angle) < 1e-9 and abs(lateral) < 1e-9
    # Robot 3 cm trop loin, tourné de 5°, décalé de 2 cm.
    robot = Pose2D(1.47, 1.02, math.radians(5.0))
    distance, angle, lateral = ecarts_visite(robot, 2.0, 1.0, math.pi, 0.5)
    assert math.isclose(distance, math.hypot(0.53, 0.02) - 0.5, abs_tol=1e-6)
    assert math.isclose(angle, 5.0, abs_tol=1e-6)
    assert math.isclose(abs(lateral), 0.02, abs_tol=1e-6)


def _simuler(robot, cible, pas=0.1, iterations=600):
    for _ in range(iterations):
        v, w, fini = commande_approche(robot, cible, 0.10, 0.35)
        if fini:
            return robot, True
        robot = Pose2D(robot.x + v * pas * math.cos(robot.yaw),
                       robot.y + v * pas * math.sin(robot.yaw), robot.yaw + w * pas)
    return robot, False


def test_final_approach_reaches_the_pose_within_the_tolerances():
    cible = pose_devant_tag(2.0, 1.0, math.pi, 0.5)
    for depart in (Pose2D(1.0, 1.0, 0.0), Pose2D(1.1, 0.7, 0.6), Pose2D(1.6, 1.1, -0.4),
                   Pose2D(1.45, 0.95, 2.5)):
        robot, fini = _simuler(depart, cible)
        assert fini, depart
        distance, angle, _ = ecarts_visite(robot, 2.0, 1.0, math.pi, 0.5)
        assert abs(distance) < 0.03 and abs(angle) < 4.0


def test_close_corrections_back_up_instead_of_turning_around():
    cible = Pose2D(1.5, 1.0, 0.0)
    v, w, _ = commande_approche(Pose2D(1.6, 1.0, 0.0), cible, 0.10, 0.35)
    assert v < 0.0 and abs(w) < 1e-6


def test_lidar_guard_measures_the_corridor_in_front_and_behind():
    points = [(0.40, 0.0), (0.30, 0.5), (-0.50, 0.1), (1.0, -0.2)]
    assert math.isclose(distance_libre(points, 0.27), 0.18)
    assert math.isclose(distance_libre(points, 0.27, avant=False), 0.19)
    assert distance_libre([], 0.27) == float('inf')


def test_exploration_points_stay_clear_of_obstacles():
    largeur = hauteur = 60                           # 3 m x 3 m à 5 cm
    grille = [0] * (largeur * hauteur)
    for ligne in range(hauteur):
        grille[ligne * largeur + 30] = 100           # mur vertical au milieu
    points = points_exploration(grille, largeur, hauteur, 0.05, 0.0, 0.0)
    assert points
    assert all(abs(x - 1.525) > 0.44 for x, _ in points)
    choisi = choisir_point_exploration(points, Pose2D(0.2, 0.2, 0.0), [(0.4, 0.4)])
    assert choisi is not None and math.hypot(choisi[0] - 0.4, choisi[1] - 0.4) > 1.2
    assert choisir_point_exploration(points, Pose2D(0, 0, 0), points, rayon_vu=0.1) is None


def test_final_alignment_faces_the_tag_then_fixes_the_distance_in_a_straight_line():
    # Tag en (2, 1), face vers -x : pose visée (1.5, 1), cap 0.
    v, w, fini = commande_alignement(Pose2D(1.46, 1.02, math.radians(8.0)), 2.0, 1.0, math.pi,
                                     0.5, 0.05, 0.35, 0.018, math.radians(3.0))
    assert v == 0.0 and w < 0.0 and not fini           # d'abord le cap, sur place
    v, w, fini = commande_alignement(Pose2D(1.46, 1.02, 0.0), 2.0, 1.0, math.pi,
                                     0.5, 0.05, 0.35, 0.018, math.radians(3.0))
    assert v > 0.0 and not fini                         # trop loin : avance tout droit
    v, w, fini = commande_alignement(Pose2D(1.50, 1.03, 0.0), 2.0, 1.0, math.pi,
                                     0.5, 0.05, 0.35, 0.018, math.radians(3.0))
    assert fini                                         # 3 cm de côté : seule la distance compte


def test_final_alignment_converges_despite_noisy_tag_views():
    import random
    hasard = random.Random(4)
    robot = Pose2D(1.47, 0.97, math.radians(-10.0))
    for _ in range(400):
        tag_x, tag_y = 2.0 + hasard.gauss(0, 0.01), 1.0 + hasard.gauss(0, 0.01)
        lacet = math.pi + hasard.gauss(0, math.radians(1.0))
        v, w, fini = commande_alignement(robot, tag_x, tag_y, lacet, 0.5, 0.05, 0.35, 0.018,
                                         math.radians(3.0))
        if fini:
            break
        robot = Pose2D(robot.x + v * 0.1 * math.cos(robot.yaw),
                       robot.y + v * 0.1 * math.sin(robot.yaw), robot.yaw + w * 0.1)
    assert fini
    distance, angle, _ = ecarts_visite(robot, 2.0, 1.0, math.pi, 0.5)
    assert abs(distance) < 0.04 and abs(angle) < 5.0
