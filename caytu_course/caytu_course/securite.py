"""Collision prédite sur l'empreinte balayée, sans dépendance ROS."""

import math

import numpy as np


def trajectoire_libre(points, lineaire, angulaire, empreinte, marge, horizon, pas):
    """Vérifie avance, recul et rotation, y compris les coins du châssis.

    Le rectangle englobant est conservateur pour une empreinte non rectangulaire.
    La marge couvre aussi l'intervalle entre deux échantillons de la trajectoire.
    """
    if not math.isfinite(lineaire) or not math.isfinite(angulaire):
        return False
    if not points:
        return True
    obstacles = np.asarray(points, dtype=float)
    if not np.isfinite(obstacles).all():
        return False
    rayon = max(math.hypot(x, y) for x, y in empreinte)
    marge += (abs(lineaire) + abs(angulaire) * rayon) * pas
    xmin = min(x for x, _ in empreinte) - marge
    xmax = max(x for x, _ in empreinte) + marge
    ymin = min(y for _, y in empreinte) - marge
    ymax = max(y for _, y in empreinte) + marge
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
        if np.any((local_x >= xmin) & (local_x <= xmax)
                  & (local_y >= ymin) & (local_y <= ymax)):
            return False
    return True


def donnees_fraiches(instants, maintenant, age_max):
    """Aucune source absente, périmée ou provenant d'une horloge future."""
    return bool(instants) and all(
        instant is not None and 0.0 <= maintenant - instant <= age_max
        for instant in instants)
