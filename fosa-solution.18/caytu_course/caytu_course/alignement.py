"""Asservissement visuel final : amener le robot à 0,5 m face au tag.

Nav2 s'arrête au point de pré-approche (0,9 m) : plus près, son contrôle de
collision refusait d'avancer vers le support du tag. L'asservissement finit
le trajet en deux temps :

1. aller au point de visite (0,5 m sur la normale du tag) : tourner vers lui,
   puis rouler en corrigeant le cap, en marche arrière s'il est derrière ;
2. une fois au point, tourner sur place pour faire face au tag.

« Au point » : bonne distance, et écart latéral dans la tolérance. Vouloir
rattraper quelques centimètres de côté faisait pivoter le robot vers le point
(jusqu'à 80°), le tag sortait du champ de la caméra et le robot oscillait au
bord du champ jusqu'au délai (simulation).

Une fois au point, le robot y reste tant que l'écart tient dans la tolérance
d'acceptation : il corrige la distance par de petits mouvements avant ou
arrière, face au tag. Avec des seuils d'entrée et de sortie trop proches, la
distance mesurée (qui varie pendant la rotation, la caméra n'étant pas au
centre de rotation) faisait alterner les deux modes : 4 minutes perdues.

Un écart latéral se corrige donc en roulant ; seul un écart très grand (tag
vu de biais, ou trop loin) redemande une navigation Nav2.
"""

from dataclasses import dataclass
import math

# Au point, la distance est jugée bonne à HYSTERESIS_POINT fois la précision.
HYSTERESIS_POINT = 2.0


@dataclass(frozen=True)
class ReglagesAlignement:
    tolerance_cap: float         # au-delà, tourner sur place avant de rouler
    tolerance_laterale: float    # écart latéral accepté une fois à bonne distance
    tolerance_distance: float    # au point, on n'en sort qu'au-delà
    gain_lineaire: float
    gain_angulaire: float
    max_lineaire: float
    max_angulaire: float
    min_lineaire: float
    min_angulaire: float
    precision_point: float       # distance au point de visite pour s'arrêter
    precision_cap: float
    erreur_distance_max: float   # au-delà : Nav2, pas l'asservissement visuel
    erreur_laterale_max: float


@dataclass(frozen=True)
class CommandeAlignement:
    lineaire: float
    angulaire: float
    etat: str                    # 'en_cours' | 'aligne' | 'renaviguer'
    au_point: bool = False       # rotation finale en cours


def _saturer(valeur, minimum, maximum):
    """Garde le signe, borne l'amplitude entre minimum et maximum."""
    if valeur == 0.0:
        return 0.0
    return math.copysign(min(max(abs(valeur), minimum), maximum), valeur)


def _borner(valeur, maximum):
    return max(-maximum, min(maximum, valeur))


def _tourner(angle, reglages, au_point=False):
    return CommandeAlignement(0.0, _saturer(
        reglages.gain_angulaire * angle, reglages.min_angulaire,
        reglages.max_angulaire), 'en_cours', au_point)


def _face_au_tag(erreurs, reglages):
    """Au point : tourner face au tag, puis ajuster la distance en ligne droite."""
    distance_bonne = abs(erreurs.distance) <= reglages.precision_point * HYSTERESIS_POINT
    if abs(erreurs.cap) <= reglages.precision_cap and distance_bonne:
        return CommandeAlignement(0.0, 0.0, 'aligne', True)
    if abs(erreurs.cap) > reglages.tolerance_cap or distance_bonne:
        return _tourner(erreurs.cap, reglages, au_point=True)
    return CommandeAlignement(
        _saturer(reglages.gain_lineaire * erreurs.distance,
                 reglages.min_lineaire, reglages.max_lineaire),
        _borner(reglages.gain_angulaire * erreurs.cap, reglages.max_angulaire),
        'en_cours', True)


def commande_alignement(erreurs, reglages, au_point=False):
    """Commande de vitesse ; au_point : valeur rendue au cycle précédent."""
    if (abs(erreurs.laterale) > reglages.erreur_laterale_max
            or abs(erreurs.distance) > reglages.erreur_distance_max):
        return CommandeAlignement(0.0, 0.0, 'renaviguer')
    if au_point and (abs(erreurs.distance) <= reglages.tolerance_distance
                     and abs(erreurs.laterale) <= reglages.tolerance_laterale):
        return _face_au_tag(erreurs, reglages)
    ecart = math.hypot(erreurs.vers_point_x, erreurs.vers_point_y)
    bonne_distance = (abs(erreurs.distance) <= reglages.precision_point
                      and abs(erreurs.laterale) <= reglages.tolerance_laterale)
    if ecart <= reglages.precision_point or bonne_distance:
        return _face_au_tag(erreurs, reglages)
    angle = math.atan2(erreurs.vers_point_y, erreurs.vers_point_x)
    sens = 1.0
    if abs(angle) > math.pi / 2.0:
        # Point derrière (robot trop près) : reculer, l'arrière vers le point.
        angle = math.atan2(math.sin(angle + math.pi), math.cos(angle + math.pi))
        sens = -1.0
    if abs(angle) > reglages.tolerance_cap:
        return _tourner(angle, reglages)
    return CommandeAlignement(
        sens * _saturer(reglages.gain_lineaire * ecart,
                        reglages.min_lineaire, reglages.max_lineaire),
        _borner(reglages.gain_angulaire * angle, reglages.max_angulaire),
        'en_cours')


def obstacle_devant(points, x_avant, demi_largeur, marge):
    """Vrai si un point (repère robot) occupe le couloir devant l'empreinte."""
    limite = x_avant + marge
    return any(
        0.0 < x <= limite and abs(y) <= demi_largeur for x, y in points)


def obstacle_derriere(points, x_arriere, demi_largeur, marge):
    """Vrai si un point occupe le couloir derrière l'empreinte (recul)."""
    limite = x_arriere - marge
    return any(
        limite <= x < 0.0 and abs(y) <= demi_largeur for x, y in points)
