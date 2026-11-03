"""Asservissement visuel final : amener le robot à 0,5 m face au tag.

D'abord tourner pour faire face au tag, puis avancer ou reculer en ligne
droite en corrigeant le cap. Un écart latéral trop grand ne se corrige pas
sur place : on redemande une navigation Nav2 vers la pose d'approche.

Deux niveaux : le robot ne s'arrête qu'à l'objectif de précision ; les
tolérances (plus larges) servent à accepter la position si le temps manque.
S'arrêter dès l'entrée dans la tolérance laissait le robot à son bord.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ReglagesAlignement:
    tolerance_distance: float
    tolerance_laterale: float
    tolerance_cap: float
    gain_lineaire: float
    gain_angulaire: float
    max_lineaire: float
    max_angulaire: float
    min_lineaire: float
    min_angulaire: float
    precision_distance: float
    precision_cap: float


@dataclass(frozen=True)
class CommandeAlignement:
    lineaire: float
    angulaire: float
    etat: str   # 'en_cours' | 'aligne' | 'renaviguer'


def _saturer(valeur, minimum, maximum):
    """Garde le signe, borne l'amplitude entre minimum et maximum."""
    if valeur == 0.0:
        return 0.0
    return math.copysign(min(max(abs(valeur), minimum), maximum), valeur)


def _borner(valeur, maximum):
    return max(-maximum, min(maximum, valeur))


def commande_alignement(erreurs, reglages):
    if abs(erreurs.laterale) > 2.0 * reglages.tolerance_laterale:
        return CommandeAlignement(0.0, 0.0, 'renaviguer')
    distance_atteinte = abs(erreurs.distance) <= reglages.precision_distance
    if distance_atteinte and abs(erreurs.cap) <= reglages.precision_cap:
        return CommandeAlignement(0.0, 0.0, 'aligne')
    if abs(erreurs.cap) > reglages.tolerance_cap or distance_atteinte:
        return CommandeAlignement(0.0, _saturer(
            reglages.gain_angulaire * erreurs.cap,
            reglages.min_angulaire, reglages.max_angulaire), 'en_cours')
    return CommandeAlignement(
        _saturer(reglages.gain_lineaire * erreurs.distance,
                 reglages.min_lineaire, reglages.max_lineaire),
        _borner(reglages.gain_angulaire * erreurs.cap, reglages.max_angulaire),
        'en_cours')


def obstacle_devant(points, x_avant, demi_largeur, marge):
    """Vrai si un point (repère robot) occupe le couloir devant l'empreinte."""
    limite = x_avant + marge
    return any(
        0.0 < x <= limite and abs(y) <= demi_largeur for x, y in points)
