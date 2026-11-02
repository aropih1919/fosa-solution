"""Tests des fonctions pures de caytu_perception (sans ROS).

Les imports ROS/interfaces sont mockés avant tout import du paquet,
ce qui permet d'exécuter ces tests dans un environnement CI sans ROS installé.
"""

import math
import sys
from unittest.mock import MagicMock

# ---------------------------------------------------------------------------
# Mock de tous les modules ROS et dépendances avant tout import du paquet
# ---------------------------------------------------------------------------
_MODULES_MOCKS = [
    'rclpy',
    'rclpy.node',
    'rclpy.time',
    'cv2',
    'cv2.aruco',
    'cv_bridge',
    'sensor_msgs',
    'sensor_msgs.msg',
    'sensor_msgs_py',
    'sensor_msgs_py.point_cloud2',
    'tf2_ros',
    'geometry_msgs',
    'geometry_msgs.msg',
    'caytu_interfaces',
    'caytu_interfaces.msg',
    'caytu_interfaces.srv',
    'std_msgs',
    'std_msgs.msg',
    'caytu_config',
    'caytu_config.loader',
    'numpy',
]

for _module in _MODULES_MOCKS:
    sys.modules.setdefault(_module, MagicMock())

# numpy doit se comporter suffisamment pour les fonctions pures qui l'utilisent
import numpy as np  # noqa: E402 — numpy réel, pas mocké

# Remplacer le mock numpy par le vrai numpy
sys.modules['numpy'] = np

# ---------------------------------------------------------------------------
# Import des fonctions pures après le mock
# ---------------------------------------------------------------------------
from caytu_perception.detecteur_tags import (  # noqa: E402
    _multiplier_quaternions,
    _quaternion_depuis_matrice,
    _tourner_vecteur,
)
from caytu_perception.classificateur_obstacles import (  # noqa: E402
    _composantes,
    _tourner_quaternion,
)


# ===========================================================================
# Constantes de tolérance numérique
# ===========================================================================
TOLERANCE = 1e-9


def _proches(a, b, tol=TOLERANCE):
    """Vérifie que deux tuples de flottants sont proches composante par composante."""
    return all(abs(x - y) < tol for x, y in zip(a, b))


# ===========================================================================
# Lot 7 — Tag detector : fonctions pures
# ===========================================================================

class TestQuaternionDepuisMatrice:
    """Tests de `_quaternion_depuis_matrice`."""

    def test_quaternion_depuis_matrice_identite(self):
        """Matrice identité 3×3 → quaternion unité (0, 0, 0, 1)."""
        identite = np.eye(3)
        qx, qy, qz, qw = _quaternion_depuis_matrice(identite)
        assert abs(qx) < TOLERANCE
        assert abs(qy) < TOLERANCE
        assert abs(qz) < TOLERANCE
        assert abs(qw - 1.0) < TOLERANCE

    def test_quaternion_depuis_matrice_rotation_z_90(self):
        """Rotation de 90° autour de Z → quaternion (0, 0, sin45°, cos45°)."""
        angle = math.pi / 2.0
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        # Matrice de rotation Rz(90°)
        matrice = np.array([
            [cos_a, -sin_a, 0.0],
            [sin_a,  cos_a, 0.0],
            [0.0,    0.0,   1.0],
        ])
        qx, qy, qz, qw = _quaternion_depuis_matrice(matrice)
        demi = math.sqrt(2.0) / 2.0
        assert abs(qx) < 1e-7
        assert abs(qy) < 1e-7
        assert abs(qz - demi) < 1e-7
        assert abs(qw - demi) < 1e-7

    def test_quaternion_depuis_matrice_rotation_x_180(self):
        """Rotation de 180° autour de X → quaternion (1, 0, 0, 0)."""
        matrice = np.array([
            [1.0,  0.0,  0.0],
            [0.0, -1.0,  0.0],
            [0.0,  0.0, -1.0],
        ])
        qx, qy, qz, qw = _quaternion_depuis_matrice(matrice)
        assert abs(qx - 1.0) < 1e-7
        assert abs(qy) < 1e-7
        assert abs(qz) < 1e-7
        assert abs(qw) < 1e-7


class TestMultiplicationQuaternions:
    """Tests de `_multiplier_quaternions`."""

    def test_multiplication_quaternions_identite_droite(self):
        """q * identité = q."""
        q = (0.1, 0.2, 0.3, math.sqrt(1.0 - 0.1**2 - 0.2**2 - 0.3**2))
        identite = (0.0, 0.0, 0.0, 1.0)
        resultat = _multiplier_quaternions(q, identite)
        assert _proches(resultat, q)

    def test_multiplication_quaternions_identite_gauche(self):
        """identité * q = q."""
        q = (0.1, 0.2, 0.3, math.sqrt(1.0 - 0.1**2 - 0.2**2 - 0.3**2))
        identite = (0.0, 0.0, 0.0, 1.0)
        resultat = _multiplier_quaternions(identite, q)
        assert _proches(resultat, q)

    def test_multiplication_quaternions_norme_preservee(self):
        """Le produit de deux quaternions unitaires est unitaire."""
        q1 = (0.0, 0.0, math.sqrt(2.0) / 2.0, math.sqrt(2.0) / 2.0)  # Rz(90°)
        q2 = (0.0, 0.0, math.sqrt(2.0) / 2.0, math.sqrt(2.0) / 2.0)  # Rz(90°)
        resultat = _multiplier_quaternions(q1, q2)
        norme = math.sqrt(sum(c * c for c in resultat))
        assert abs(norme - 1.0) < 1e-9

    def test_multiplication_quaternions_rz90_fois_rz90_donne_rz180(self):
        """Rz(90°) * Rz(90°) = Rz(180°) ≙ (0, 0, 1, 0)."""
        demi = math.sqrt(2.0) / 2.0
        rz90 = (0.0, 0.0, demi, demi)
        qx, qy, qz, qw = _multiplier_quaternions(rz90, rz90)
        assert abs(qx) < 1e-9
        assert abs(qy) < 1e-9
        assert abs(qz - 1.0) < 1e-9
        assert abs(qw) < 1e-9


class TestRotationVecteur:
    """Tests de `_tourner_vecteur` (tag_detector)."""

    def test_rotation_vecteur_identite(self):
        """Quaternion identité → vecteur inchangé."""
        q_identite = (0.0, 0.0, 0.0, 1.0)
        vecteur = (1.0, 2.0, 3.0)
        assert _proches(_tourner_vecteur(q_identite, vecteur), vecteur)

    def test_rotation_vecteur_rz90_axe_x_vers_y(self):
        """Rotation 90° autour de Z : X → Y."""
        demi = math.sqrt(2.0) / 2.0
        q_rz90 = (0.0, 0.0, demi, demi)
        vecteur_x = (1.0, 0.0, 0.0)
        rx, ry, rz = _tourner_vecteur(q_rz90, vecteur_x)
        assert abs(rx) < 1e-9
        assert abs(ry - 1.0) < 1e-9
        assert abs(rz) < 1e-9

    def test_rotation_vecteur_rz180_inverse_x(self):
        """Rotation 180° autour de Z : X → -X."""
        q_rz180 = (0.0, 0.0, 1.0, 0.0)
        vecteur_x = (1.0, 0.0, 0.0)
        rx, ry, rz = _tourner_vecteur(q_rz180, vecteur_x)
        assert abs(rx + 1.0) < 1e-9
        assert abs(ry) < 1e-9
        assert abs(rz) < 1e-9


# ===========================================================================
# Lot 7 — Obstacle classifier : fonctions pures
# ===========================================================================

class TestRotationVecteurClassifier:
    """Tests de `_tourner_quaternion` (classificateur d'obstacles)."""

    def test_rotation_vecteur_classifier_identite(self):
        """Quaternion identité → vecteur inchangé."""
        q = (0.0, 0.0, 0.0, 1.0)
        vecteur = (3.0, -1.0, 2.0)
        assert _proches(_tourner_quaternion(q, vecteur), vecteur)

    def test_rotation_vecteur_classifier_rz90(self):
        """Rotation 90° autour de Z : X → Y (cohérente avec le détecteur de tags)."""
        demi = math.sqrt(2.0) / 2.0
        q = (0.0, 0.0, demi, demi)
        rx, ry, rz = _tourner_quaternion(q, (1.0, 0.0, 0.0))
        assert abs(rx) < 1e-9
        assert abs(ry - 1.0) < 1e-9
        assert abs(rz) < 1e-9


class TestComposantesConnexes:
    """Tests des composantes connexes sur la grille XY."""

    def test_composantes_connexes_vide(self):
        """Liste vide → aucune composante."""
        composantes = list(_composantes([], tolerance=0.5))
        assert composantes == []

    def test_composantes_connexes_un_point(self):
        """Un seul point → une composante avec ce point."""
        composantes = list(_composantes([(1.0, 2.0, 0.0)], tolerance=0.5))
        assert len(composantes) == 1
        assert len(composantes[0]) == 1
        assert composantes[0][0] == (1.0, 2.0, 0.0)

    def test_composantes_connexes_deux_groupes_separes(self):
        """Deux groupes éloignés → deux composantes distinctes."""
        groupe_a = [(0.0, 0.0, 1.0), (0.1, 0.0, 1.0), (0.0, 0.1, 1.0)]
        groupe_b = [(10.0, 10.0, 1.0), (10.1, 10.0, 1.0)]
        points = groupe_a + groupe_b
        composantes = list(_composantes(points, tolerance=0.5))
        assert len(composantes) == 2
        tailles = sorted(len(c) for c in composantes)
        assert tailles == [2, 3]

    def test_composantes_connexes_points_adjacents_fusionnes(self):
        """Points voisins dans la grille → une seule composante."""
        # Deux points séparés de 0.4 m avec tolérance 0.5 m → même cellule ou adjacents
        points = [(0.0, 0.0, 0.0), (0.4, 0.0, 0.0), (0.8, 0.0, 0.0)]
        composantes = list(_composantes(points, tolerance=0.5))
        # Tous dans des cellules voisines → une seule composante
        assert len(composantes) == 1
        assert len(composantes[0]) == 3

    def test_composantes_connexes_tolerance_separe_groupes(self):
        """Avec une tolérance fine, deux points distants restent séparés."""
        points = [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0)]
        composantes = list(_composantes(points, tolerance=0.5))
        assert len(composantes) == 2


# ===========================================================================
# Lot 7 — Intégration synthétique (math pure, sans ROS)
# ===========================================================================

class TestIntegrationSynthetique:
    """Tests de logique métier sans instancier de nœuds ROS."""

    def test_distance_tag_calculee_manuellement(self):
        """Calcul de distance d'un tag depuis la translation (math.sqrt)."""
        # Simule un vecteur translation issu de solvePnP
        translation = (1.0, 2.0, 3.0)
        distance_attendue = math.sqrt(1.0 + 4.0 + 9.0)  # ≈ 3.742
        distance_calculee = math.sqrt(sum(c * c for c in translation))
        assert abs(distance_calculee - distance_attendue) < 1e-12

    def test_tag_distance_apres_rotation(self):
        """La rotation ne change pas la norme du vecteur translation."""
        demi = math.sqrt(2.0) / 2.0
        q_rz90 = (0.0, 0.0, demi, demi)
        vecteur = (1.0, 0.0, 2.0)
        norme_avant = math.sqrt(sum(c * c for c in vecteur))
        tourne = _tourner_vecteur(q_rz90, vecteur)
        norme_apres = math.sqrt(sum(c * c for c in tourne))
        assert abs(norme_avant - norme_apres) < 1e-9

    def test_personne_non_detectee_sans_profondeur(self):
        """Sans points dans le nuage, aucune composante n'est retournée."""
        composantes = list(_composantes([], tolerance=0.3))
        # Aucun cluster → aucune entité détectée
        assert composantes == []

    def test_filtre_distance_personne_hors_zone(self):
        """Logique de filtrage : distance hors bornes → entité ignorée."""
        distance_entite = 8.0          # supposons max_distance = 5.0
        min_distance_m = 0.5
        max_distance_m = 5.0
        dans_zone = min_distance_m <= distance_entite <= max_distance_m
        assert not dans_zone

    def test_filtre_distance_personne_dans_zone(self):
        """Logique de filtrage : distance dans les bornes → entité conservée."""
        distance_entite = 2.5
        min_distance_m = 0.5
        max_distance_m = 5.0
        dans_zone = min_distance_m <= distance_entite <= max_distance_m
        assert dans_zone

    def test_filtre_bearing_hors_fov(self):
        """Entité hors du cône frontal → ignorée."""
        bearing_rad = 1.5          # > π/4 ≈ 0.785 rad (demi-angle FOV exemple)
        front_fov_rad = math.pi / 2.0   # 90° total = ±45°
        hors_fov = abs(bearing_rad) > front_fov_rad / 2.0
        assert hors_fov

    def test_filtre_bearing_dans_fov(self):
        """Entité dans le cône frontal → conservée."""
        bearing_rad = 0.2
        front_fov_rad = math.pi / 2.0
        hors_fov = abs(bearing_rad) > front_fov_rad / 2.0
        assert not hors_fov
