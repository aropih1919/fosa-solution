"""Rapport de fin de trajet (aucune dépendance ROS).

Répond à la question « où part le temps ? » : démarrage de Nav2, roulage,
pivots sur place, arrêts. Seules des sommes courantes sont gardées : O(1) en
temps et en mémoire par échantillon, la trajectoire n'est pas stockée.

N'a aucun effet sur la conduite : le rapport lit la pose déjà calculée et
s'écrit une seule fois, à la fin.
"""

import csv
import json
import math
import os
from typing import Dict, List, Optional

from caytu_nav_solution.retry_policy import count_causes

# En dessous de cette vitesse (m/s), le robot est considéré à l'arrêt.
_MOVING_MIN_SPEED = 0.03
# Pivot sur place : le robot tourne (rad/s) autour d'un rayon inférieur à
# _PIVOT_MAX_RADIUS. base_footprint est à 9,5 cm de l'essieu : il se déplace
# un peu pendant un pivot, d'où un rayon plutôt qu'une vitesse nulle.
_PIVOT_MIN_RATE = 0.15
_PIVOT_MAX_RADIUS = 0.20
# Distance cumulée à partir de laquelle le robot est déclaré parti.
_START_DIST = 0.05
# Un arrêt ou un pivot plus court que cela n'est pas compté comme un événement.
_EVENT_MIN_SEC = 0.5

MOVING, PIVOT, STOPPED = 'roulage', 'pivot', 'arrêt'


def _angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


class RunMetrics:
    """Accumule les compteurs d'un trajet complet."""

    def __init__(self) -> None:
        self._t0_sim: Optional[float] = None
        self._t0_wall: Optional[float] = None
        self._last: Optional[tuple] = None      # (t, x, y, cap)
        self._state: Optional[str] = None
        self._state_since = 0.0
        self._counted = True                    # événement en cours déjà compté ?
        self.path_length = 0.0
        self.moving_time = 0.0
        self.pivot_time = 0.0
        self.stopped_time = 0.0                 # arrêts après le départ du robot
        self.stops = 0
        self.pivots = 0
        self.first_motion_t: Optional[float] = None
        self.nav2_ready_t: Optional[float] = None
        self.nav2_restarts = 0                  # relances du bringup au démarrage
        self.goal_accepted_t: Optional[float] = None
        self.min_goal_distance: Optional[float] = None
        self.attempts = 0
        self.rejections = 0
        self.contacts = 0
        self.contact_names = set()
        self.contacts_available = True
        self.map_used = ''
        self.settings = ''                      # réglages de conduite utilisés
        self.failures: List[str] = []           # cause de chaque interruption
        self.fallback_used = False
        self.fallback_offset_m: Optional[float] = None
        self._result = 'unknown'
        self._sim_duration = 0.0
        self._wall_duration = 0.0
        self._final_distance: Optional[float] = None

    def start(self, t_sim: float, t_wall: float) -> None:
        """Mémorise les deux temps de départ (simulé et réel)."""
        self._t0_sim = t_sim
        self._t0_wall = t_wall

    def mark_nav2_ready(self, t_sim: float) -> None:
        """Instant (temps simulé) où Nav2 est actif et le robot localisé."""
        if self.nav2_ready_t is None:
            self.nav2_ready_t = t_sim

    def mark_goal_accepted(self, t_sim: float) -> None:
        """Instant (temps simulé) où Nav2 accepte le premier but : il est actif."""
        if self.goal_accepted_t is None:
            self.goal_accepted_t = t_sim

    def update_pose(self, t: float, x: float, y: float, goal_x: float, goal_y: float,
                    yaw: float = 0.0) -> None:
        """Intègre un échantillon de pose (O(1), sans historique)."""
        goal_d = math.hypot(goal_x - x, goal_y - y)
        if self.min_goal_distance is None or goal_d < self.min_goal_distance:
            self.min_goal_distance = goal_d
        if self._last is None:
            self._last = (t, x, y, yaw)
            return
        last_t, last_x, last_y, last_yaw = self._last
        dt = t - last_t
        if dt <= 0.0:
            self._last = (t, x, y, yaw)
            return
        d = math.hypot(x - last_x, y - last_y)
        speed = d / dt
        rate = abs(_angle_diff(yaw, last_yaw)) / dt
        self._last = (t, x, y, yaw)

        if rate >= _PIVOT_MIN_RATE and speed < _PIVOT_MAX_RADIUS * rate:
            state = PIVOT
        elif speed >= _MOVING_MIN_SPEED:
            state = MOVING
        else:
            state = STOPPED

        if state == MOVING:
            self.path_length += d
        if self.first_motion_t is None:
            if state == STOPPED:
                return                          # pas encore parti : c'est le démarrage
            if state == PIVOT or self.path_length >= _START_DIST:
                self.first_motion_t = last_t
        if state == MOVING:
            self.moving_time += dt
        elif state == PIVOT:
            self.pivot_time += dt
        else:
            self.stopped_time += dt

        if state != self._state:
            self._state, self._state_since = state, last_t
            self._counted = False
        elif not self._counted and t - self._state_since >= _EVENT_MIN_SEC:
            self._counted = True
            if state == STOPPED:
                self.stops += 1
            elif state == PIVOT:
                self.pivots += 1

    def add_attempt(self) -> None:
        self.attempts += 1

    def add_rejection(self) -> None:
        self.rejections += 1

    def add_failure(self, cause: str) -> None:
        """Mémorise la cause d'une interruption de la navigation."""
        self.failures.append(cause)

    def add_contact(self, name_a: str, name_b: str) -> None:
        """Compte un contact sauf s'il implique le sol."""
        low_a, low_b = name_a.lower(), name_b.lower()
        if 'floor' in low_a or 'ground' in low_a or 'floor' in low_b or 'ground' in low_b:
            return
        self.contacts += 1
        self.contact_names.add(name_a)
        self.contact_names.add(name_b)

    def finish(self, result: str, t_sim: float, t_wall: float,
               final_distance: Optional[float]) -> None:
        """Fige les durées à partir des temps de fin."""
        self._result = result
        self._final_distance = final_distance
        if self._t0_sim is not None:
            self._sim_duration = max(0.0, t_sim - self._t0_sim)
        if self._t0_wall is not None:
            self._wall_duration = max(0.0, t_wall - self._t0_wall)

    def _since_start(self, t: Optional[float]) -> Optional[float]:
        if self._t0_sim is None or t is None:
            return None
        return max(0.0, t - self._t0_sim)

    def to_dict(self) -> Dict:
        data = {
            'result': self._result,
            'sim_duration_s': self._sim_duration,
            'wall_duration_s': self._wall_duration,
            'nav2_ready_s': self._since_start(self.nav2_ready_t),
            'nav2_restarts': self.nav2_restarts,
            'goal_accepted_s': self._since_start(self.goal_accepted_t),
            'first_motion_s': self._since_start(self.first_motion_t),
            'moving_time_s': self.moving_time,
            'pivot_time_s': self.pivot_time,
            'stopped_time_s': self.stopped_time,
            'pivots': self.pivots,
            'stops': self.stops,
            'path_length_m': self.path_length,
            'final_distance_m': self._final_distance,
            'min_goal_distance_m': self.min_goal_distance,
            'attempts': self.attempts,
            'rejections': self.rejections,
            'failures': len(self.failures),
            'failure_causes': count_causes(self.failures),
            'fallback_used': self.fallback_used,
            'fallback_offset_m': self.fallback_offset_m,
            'contacts': self.contacts,
            'contact_names': sorted(self.contact_names),
            'contacts_available': self.contacts_available,
            'settings': self.settings,
            'map_used': self.map_used,
        }
        json.dumps(data)                # échoue tout de suite si une valeur n'est pas sérialisable
        return data

    def to_text(self) -> str:
        """Résumé lisible pour le journal."""
        def sec(value: Optional[float]) -> str:
            return '?' if value is None else f'{value:.1f} s'

        def metres(value: Optional[float]) -> str:
            return '?' if value is None else f'{value:.2f} m'

        first = self._since_start(self.first_motion_t)
        mean = self.path_length / self.moving_time if self.moving_time > 0.0 else 0.0
        fallback = (f'oui ({metres(self.fallback_offset_m)} du centre)'
                    if self.fallback_used else 'non')
        contacts = str(self.contacts) if self.contacts_available else 'non mesurés'
        lines = [
            '===== Rapport du trajet =====',
            f'résultat : {self._result}    réglages : {self.settings or "par défaut"}',
            f'durée totale : {self._sim_duration:.1f} s simulées '
            f'({self._wall_duration:.1f} s réelles)',
            f'  démarrage (avant le premier mouvement) : {sec(first)}, '
            f'dont Nav2 lancé à {sec(self._since_start(self.nav2_ready_t))}'
            + (f' après {self.nav2_restarts} relance(s)' if self.nav2_restarts else '')
            + f', premier but accepté à {sec(self._since_start(self.goal_accepted_t))}',
            f'  roulage : {self.moving_time:.1f} s    pivots : {self.pivot_time:.1f} s '
            f'({self.pivots})    arrêts : {self.stopped_time:.1f} s ({self.stops})',
            f'chemin : {self.path_length:.2f} m    vitesse moyenne en roulage : {mean:.2f} m/s',
            f'distance finale au centre du but : {metres(self._final_distance)}',
            f'tentatives : {self.attempts}    interruptions : '
            f'{count_causes(self.failures) or "aucune"}    repli : {fallback}',
            f'contacts : {contacts}',
        ]
        return '\n'.join(lines)


def append_csv_row(path: str, data: Dict) -> str:
    """Ajoute une ligne au CSV des trajets et retourne le fichier écrit.

    Si le fichier existe avec d'autres colonnes (ancienne version du rapport),
    il est mis de côté sous un nouveau nom au lieu de recevoir une ligne
    décalée.
    """
    fields = list(data.keys())
    if os.path.exists(path):
        with open(path, newline='', encoding='utf-8') as stream:
            header = next(csv.reader(stream), None)
        if header != fields:
            base, ext = os.path.splitext(path)
            k = 1
            while os.path.exists(f'{base}.ancien{k}{ext}'):
                k += 1
            os.replace(path, f'{base}.ancien{k}{ext}')
    new = not os.path.exists(path)
    with open(path, 'a', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        if new:
            writer.writeheader()
        writer.writerow(data)
    return path
