"""Géométrie d'une visite de tag (règlement : 0,5 m, face au tag à ±12°).

Repère du tag (détecteur OpenCV) : x à droite, y en haut, z sortant de la
surface vers la caméra. La direction utile au sol est donc l'axe z projeté,
pas le lacet du quaternion (qui donne l'axe x, parallèle au mur).
"""

from dataclasses import dataclass
import math

# En dessous, le tag est presque horizontal : pas de normale exploitable.
NORME_HORIZONTALE_MIN = 0.3


def normaliser_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def cap_normale(qx, qy, qz, qw):
    """Cap (rad) de la normale du tag projetée au sol, ou None si à plat."""
    normale_x = 2.0 * (qx * qz + qw * qy)
    normale_y = 2.0 * (qy * qz - qw * qx)
    if math.hypot(normale_x, normale_y) < NORME_HORIZONTALE_MIN:
        return None
    return math.atan2(normale_y, normale_x)


def pose_approche(x_tag, y_tag, cap_normale_tag, distance):
    """Pose (x, y, cap) du centre du robot à `distance` devant le tag, face à lui."""
    return (x_tag + distance * math.cos(cap_normale_tag),
            y_tag + distance * math.sin(cap_normale_tag),
            normaliser_angle(cap_normale_tag + math.pi))


@dataclass(frozen=True)
class ErreursVisite:
    distance: float   # > 0 : trop loin du tag
    laterale: float   # écart du centre à l'axe de la normale du tag
    cap: float        # rotation à faire pour faire face au tag (> 0 : à gauche)
    # Vecteur du centre du robot vers le point de visite (repère du robot).
    vers_point_x: float = 0.0
    vers_point_y: float = 0.0


def erreurs_visite(x_tag, y_tag, cap_normale_tag, decalage_centre_x, distance_cible):
    """Erreurs de placement, le tag étant exprimé dans le repère du robot.

    decalage_centre_x : position du centre géométrique sur l'axe x du robot.
    """
    vers_centre_x = decalage_centre_x - x_tag
    vers_centre_y = -y_tag
    return ErreursVisite(
        distance=math.hypot(vers_centre_x, vers_centre_y) - distance_cible,
        laterale=(-vers_centre_x * math.sin(cap_normale_tag)
                  + vers_centre_y * math.cos(cap_normale_tag)),
        cap=normaliser_angle(cap_normale_tag + math.pi),
        vers_point_x=x_tag + distance_cible * math.cos(cap_normale_tag) - decalage_centre_x,
        vers_point_y=y_tag + distance_cible * math.sin(cap_normale_tag),
    )


def visite_valide(erreurs, tolerance_distance, tolerance_laterale, tolerance_cap):
    return (abs(erreurs.distance) <= tolerance_distance
            and abs(erreurs.laterale) <= tolerance_laterale
            and abs(erreurs.cap) <= tolerance_cap)
