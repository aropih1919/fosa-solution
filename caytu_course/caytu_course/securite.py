"""Collision prédite sur l'empreinte balayée, sans dépendance ROS."""

import math

import numpy as np


def _bornes(empreinte, marge):
    return (min(x for x, _ in empreinte) - marge,
            max(x for x, _ in empreinte) + marge,
            min(y for _, y in empreinte) - marge,
            max(y for _, y in empreinte) + marge)


def _distance_signee(local_x, local_y, bornes):
    """Distance au rectangle élargi : positive dehors, négative dedans."""
    xmin, xmax, ymin, ymax = bornes
    dehors_x = np.maximum(np.maximum(xmin - local_x, local_x - xmax), 0.0)
    dehors_y = np.maximum(np.maximum(ymin - local_y, local_y - ymax), 0.0)
    dehors = np.hypot(dehors_x, dehors_y)
    dedans = np.minimum(
        np.minimum(local_x - xmin, xmax - local_x),
        np.minimum(local_y - ymin, ymax - local_y))
    return np.where(dehors > 0.0, dehors, -dedans)


def trajectoire_libre(points, lineaire, angulaire, empreinte, marge, horizon, pas,
                      tolerance_penetration=0.0):
    """Vérifie avance, recul et rotation, y compris les coins du châssis.

    Le rectangle englobant est conservateur pour une empreinte non rectangulaire.
    La marge couvre aussi l'intervalle entre deux échantillons de la trajectoire.

    Une pénétration déjà présente n'immobilise plus le robot : la trajectoire
    est acceptée tant qu'elle n'augmente pas cette pénétration de plus de
    `tolerance_penetration`. Le recul ou la rotation lente peuvent ainsi
    dégager le robot d'un obstacle latéral.
    """
    if not math.isfinite(lineaire) or not math.isfinite(angulaire):
        return False
    if not points:
        return True
    obstacles = np.asarray(points, dtype=float)
    if not np.isfinite(obstacles).all():
        return False
    rayon = max(math.hypot(x, y) for x, y in empreinte)
    marge_dynamique = marge + (abs(lineaire) + abs(angulaire) * rayon) * pas
    bornes = _bornes(empreinte, marge_dynamique)
    # La marge d'échantillonnage s'applique aussi au départ : en comparant
    # deux distances prises avec le même rectangle, seule l'aggravation
    # réelle de la pénétration compte.
    distances_depart = _distance_signee(
        obstacles[:, 0], obstacles[:, 1], bornes)
    penetration = max(0.0, -float(np.min(distances_depart)))
    seuil = penetration + tolerance_penetration
    for instant in np.linspace(0.0, horizon, math.ceil(horizon / pas) + 1):
        cap = angulaire * instant
        cosinus, sinus = math.cos(cap), math.sin(cap)
        if abs(angulaire) < 1e-9:
            x, y = lineaire * instant, 0.0
        else:
            x = lineaire / angulaire * sinus
            y = lineaire / angulaire * (1.0 - cosinus)
        dx, dy = obstacles[:, 0] - x, obstacles[:, 1] - y
        local_x, local_y = cosinus * dx + sinus * dy, -sinus * dx + cosinus * dy
        if -float(np.min(_distance_signee(local_x, local_y, bornes))) > seuil + 1e-9:
            return False
    return True


def donnees_fraiches(instants, maintenant, age_max):
    """Aucune source absente, périmée ou provenant d'une horloge future."""
    return bool(instants) and all(
        instant is not None and 0.0 <= maintenant - instant <= age_max
        for instant in instants)
