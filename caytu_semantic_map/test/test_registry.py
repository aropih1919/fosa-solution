"""Tests unitaires du module caytu_semantic_map.registry (sans dépendance ROS).

Couverture :
  - normaliser : accents, casse, tirets, espaces multiples
  - RegistreLieux.resoudre : nom exact, alias avec accent, insensible à la casse, inconnu
  - RegistreLieux.par_tag : tag connu, tag inconnu
  - RegistreLieux.tous : nombre correct de lieux
  - pose_approche_depuis_tag : distance correcte et orientation vers le tag
  - quaternion_yaw : quaternion pur Z
  - Lieu sans approach_pose mais avec tag_ids : valide
  - Lieu avec approach_pose statique : champ accessible
"""

import math
import types

import pytest

from caytu_config.models import SemanticMapConfig
from caytu_semantic_map.registry import (
    LieuRegistre,
    RegistreLieux,
    normaliser,
    pose_approche_depuis_tag,
    quaternion_yaw,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

def _construire_carte_minimale():
    """Construit un SemanticMapConfig minimal pour les tests."""
    return SemanticMapConfig.from_mapping({
        'world': 'test',
        'frame': 'map',
        'locations_topic': '/semantic_map/locations',
        'resolve_service': '/semantic_map/resolve_location',
        'by_tag_service': '/semantic_map/get_location_by_tag',
        'locations': [
            {
                'name': 'accueil',
                'aliases': ['réception'],
                'tag_ids': [10],
                'description': "Point d'accueil",
                'category': 'service',
                'approach_offset_m': 0.5,
            },
            {
                'name': 'toilettes',
                'aliases': ['wc'],
                'tag_ids': [11],
                'description': 'Toilettes',
                'category': 'service',
                'approach_offset_m': 0.8,
            },
            {
                'name': 'infirmerie',
                'aliases': ['secours'],
                'tag_ids': [13],
                'description': 'Poste de secours',
                'category': 'urgence',
                'approach_offset_m': 0.8,
                'approach_pose': {'x': 1.0, 'y': 2.0, 'yaw': 0.5},
            },
        ],
    })


@pytest.fixture(name='registre')
def fixture_registre():
    """Registre construit depuis une carte sémantique minimale de test."""
    carte = _construire_carte_minimale()
    return RegistreLieux(carte)


def _pose_tag(x=1.0, y=1.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    """Simule un objet pose_tag avec les attributs attendus par pose_approche_depuis_tag."""
    position = types.SimpleNamespace(x=x, y=y)
    orientation = types.SimpleNamespace(x=qx, y=qy, z=qz, w=qw)
    return types.SimpleNamespace(position=position, orientation=orientation)


# ---------------------------------------------------------------------------
# Tests de normaliser
# ---------------------------------------------------------------------------

def test_normaliser_supprime_accents():
    """normaliser retire les accents diacritiques."""
    assert normaliser('réception') == 'reception'
    assert normaliser('infirmière') == 'infirmiere'
    assert normaliser('éàü') == 'eau'


def test_normaliser_met_en_minuscules():
    """normaliser met le texte en minuscules."""
    assert normaliser('ACCUEIL') == 'accueil'
    assert normaliser('Toilettes') == 'toilettes'


def test_normaliser_remplace_tirets_et_underscores():
    """normaliser transforme tirets et underscores en espaces."""
    assert normaliser('salle-de-réunion') == 'salle de reunion'
    assert normaliser('premier_etage') == 'premier etage'


def test_normaliser_normalise_espaces_multiples():
    """normaliser compresse les espaces multiples en un seul."""
    assert normaliser('  bureau   principal  ') == 'bureau principal'
    assert normaliser('hall   d   entrée') == 'hall d entree'


# ---------------------------------------------------------------------------
# Tests de RegistreLieux.resoudre
# ---------------------------------------------------------------------------

def test_resoudre_nom_exact(registre):
    """resoudre retrouve un lieu par son nom exact."""
    lieu = registre.resoudre('accueil')
    assert lieu is not None
    assert lieu.name == 'accueil'


def test_resoudre_alias_avec_accent(registre):
    """resoudre retrouve un lieu via un alias comportant des accents."""
    # 'réception' est alias de 'accueil'
    lieu = registre.resoudre('réception')
    assert lieu is not None
    assert lieu.name == 'accueil'


def test_resoudre_insensible_casse(registre):
    """resoudre est insensible à la casse et aux accents."""
    lieu = registre.resoudre('ACCUEIL')
    assert lieu is not None
    assert lieu.name == 'accueil'

    lieu_alias = registre.resoudre('WC')
    assert lieu_alias is not None
    assert lieu_alias.name == 'toilettes'


def test_resoudre_inconnu_retourne_none(registre):
    """resoudre retourne None si le nom ou alias est inconnu."""
    assert registre.resoudre('gymnase') is None
    assert registre.resoudre('') is None


# ---------------------------------------------------------------------------
# Tests de RegistreLieux.par_tag
# ---------------------------------------------------------------------------

def test_par_tag_connu(registre):
    """par_tag retrouve un lieu par son tag_id connu."""
    lieu = registre.par_tag(10)
    assert lieu is not None
    assert lieu.name == 'accueil'


def test_par_tag_second_lieu(registre):
    """par_tag retrouve correctement le second lieu du registre."""
    lieu = registre.par_tag(11)
    assert lieu is not None
    assert lieu.name == 'toilettes'


def test_par_tag_inconnu_retourne_none(registre):
    """par_tag retourne None pour un tag_id absent du registre."""
    assert registre.par_tag(99) is None


# ---------------------------------------------------------------------------
# Tests de RegistreLieux.tous
# ---------------------------------------------------------------------------

def test_tous_retourne_nombre_correct(registre):
    """tous() retourne un tuple avec exactement autant de lieux que dans la carte."""
    lieux = registre.tous()
    assert isinstance(lieux, tuple)
    assert len(lieux) == 3


def test_tous_contient_tous_les_noms(registre):
    """tous() contient bien chaque lieu déclaré dans la carte."""
    noms = {lieu.name for lieu in registre.tous()}
    assert noms == {'accueil', 'toilettes', 'infirmerie'}


# ---------------------------------------------------------------------------
# Tests de pose_approche_depuis_tag
# ---------------------------------------------------------------------------

def test_pose_approche_distance_correcte():
    """Le point d'approche est à exactement `offset` mètres du tag."""
    offset = 0.5
    # Tag orienté vers l'est (yaw = 0 → quaternion identité)
    tag = _pose_tag(x=2.0, y=3.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0)
    x, y, _ = pose_approche_depuis_tag(tag, offset)
    distance = math.hypot(x - tag.position.x, y - tag.position.y)
    assert math.isclose(distance, offset, rel_tol=1e-6)


def test_pose_approche_orientee_vers_tag():
    """Le robot en pose d'approche est orienté vers le tag (yaw_retour = yaw_tag + π)."""
    offset = 1.0
    # Tag orienté vers le nord (yaw = π/2)
    yaw_tag = math.pi / 2
    qz = math.sin(yaw_tag / 2)
    qw = math.cos(yaw_tag / 2)
    tag = _pose_tag(x=0.0, y=0.0, qx=0.0, qy=0.0, qz=qz, qw=qw)
    _, _, yaw_retour = pose_approche_depuis_tag(tag, offset)
    # Le yaw retour doit pointer en direction du tag (= yaw_tag + π)
    attendu = yaw_tag + math.pi
    assert math.isclose(yaw_retour, attendu, rel_tol=1e-6)


def test_pose_approche_position_correcte_direction_est():
    """Vérification numérique : tag à (5, 0) orienté est, offset 1 m → approche à (4, 0)."""
    offset = 1.0
    tag = _pose_tag(x=5.0, y=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0)
    x, y, _ = pose_approche_depuis_tag(tag, offset)
    assert math.isclose(x, 4.0, abs_tol=1e-6)
    assert math.isclose(y, 0.0, abs_tol=1e-6)


# ---------------------------------------------------------------------------
# Tests de quaternion_yaw
# ---------------------------------------------------------------------------

def test_quaternion_yaw_identite():
    """Un quaternion identité donne un yaw de 0."""
    assert math.isclose(quaternion_yaw(0.0, 0.0, 0.0, 1.0), 0.0, abs_tol=1e-9)


def test_quaternion_yaw_rotation_pi_sur_deux():
    """Rotation de π/2 autour de Z donne yaw = π/2."""
    yaw = math.pi / 2
    qz = math.sin(yaw / 2)
    qw = math.cos(yaw / 2)
    resultat = quaternion_yaw(0.0, 0.0, qz, qw)
    assert math.isclose(resultat, yaw, rel_tol=1e-6)


def test_quaternion_yaw_rotation_pi():
    """Rotation de π autour de Z donne yaw ≈ π."""
    yaw = math.pi
    qz = math.sin(yaw / 2)
    qw = math.cos(yaw / 2)
    resultat = quaternion_yaw(0.0, 0.0, qz, qw)
    assert math.isclose(abs(resultat), math.pi, rel_tol=1e-6)


# ---------------------------------------------------------------------------
# Tests sur des lieux sans approach_pose
# ---------------------------------------------------------------------------

def test_lieu_sans_approach_pose_valide(registre):
    """Un lieu dont approach_pose est absent (None) est correctement intégré au registre."""
    lieu = registre.resoudre('accueil')
    assert lieu is not None
    assert isinstance(lieu, LieuRegistre)
    # Pas de pose statique : doit être None
    assert lieu.approach_pose is None
    # Mais les tag_ids sont présents
    assert 10 in lieu.tag_ids


# ---------------------------------------------------------------------------
# Tests sur un lieu avec approach_pose statique
# ---------------------------------------------------------------------------

def test_lieu_avec_approach_pose_statique_accessible(registre):
    """Un lieu déclaré avec approach_pose a le champ Pose2DConfig accessible."""
    lieu = registre.resoudre('infirmerie')
    assert lieu is not None
    assert lieu.approach_pose is not None
    # Vérifie que les coordonnées sont accessibles (objets Pose2DConfig)
    assert hasattr(lieu.approach_pose, 'x')
    assert hasattr(lieu.approach_pose, 'y')
    assert hasattr(lieu.approach_pose, 'yaw')


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
