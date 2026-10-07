"""Metriques de fin de trajet, sans dependance ROS.

Seules des sommes courantes sont conservees : O(1) en temps et memoire
par echantillon, sans stocker la trajectoire.
"""

import json
import math
from typing import Dict, List, Optional

# Seuil de bruit odometrique : en dessous, le robot est considere a l'arret.
_MOTION_MIN_D = 0.001
# Distance cumulee a partir de laquelle le demarrage est declare.
_START_DIST = 0.05


class RunMetrics:
    """Accumule les compteurs d'un trajet complet."""

    def __init__(self) -> None:
        self._t0_sim: Optional[float] = None
        self._t0_wall: Optional[float] = None
        self._last_t: Optional[float] = None
        self._last_x: float = 0.0
        self._last_y: float = 0.0
        self.path_length = 0.0
        self.max_speed = 0.0
        self.stopped_time = 0.0
        self.first_motion_t: Optional[float] = None
        self.min_goal_distance: Optional[float] = None
        self.attempts = 0
        self.rejections = 0
        self.contacts = 0
        self.contact_names = set()
        self.contacts_available = True
        self.map_used = ""
        self.behavior_tree = ""
        self.map_correction = ""
        self.collision_monitor = ""
        self._result = "unknown"
        self._sim_duration = 0.0
        self._wall_duration = 0.0
        self._final_distance: Optional[float] = None

    def start(self, t_sim: float, t_wall: float) -> None:
        """Memorise les deux temps de depart (simule et mur)."""
        self._t0_sim = t_sim
        self._t0_wall = t_wall

    def update_pose(self, t: float, x: float, y: float,
                    goal_x: float, goal_y: float) -> None:
        """Integre un echantillon de pose (O(1), sans historique)."""
        goal_d = math.hypot(goal_x - x, goal_y - y)
        if self.min_goal_distance is None or goal_d < self.min_goal_distance:
            self.min_goal_distance = goal_d
        if self._last_t is None:
            self._last_t, self._last_x, self._last_y = t, x, y
            return
        dt = t - self._last_t
        d = math.hypot(x - self._last_x, y - self._last_y)
        self._last_t, self._last_x, self._last_y = t, x, y
        if dt <= 0.0:
            return
        if d >= _MOTION_MIN_D:
            self.path_length += d
            speed = d / dt
            if speed > self.max_speed:
                self.max_speed = speed
            if self.first_motion_t is None and self.path_length >= _START_DIST:
                self.first_motion_t = t
        else:
            self.stopped_time += dt

    def add_attempt(self) -> None:
        self.attempts += 1

    def add_rejection(self) -> None:
        self.rejections += 1

    def add_contact(self, name_a: str, name_b: str) -> None:
        """Compte un contact sauf s'il implique le sol."""
        low_a, low_b = name_a.lower(), name_b.lower()
        if "floor" in low_a or "ground" in low_a or "floor" in low_b or "ground" in low_b:
            return
        self.contacts += 1
        self.contact_names.add(name_a)
        self.contact_names.add(name_b)

    def finish(self, result: str, t_sim: float, t_wall: float,
               final_distance: Optional[float]) -> None:
        """Fige les durees a partir des temps de fin."""
        self._result = result
        self._final_distance = final_distance
        if self._t0_sim is not None:
            self._sim_duration = max(0.0, t_sim - self._t0_sim)
        if self._t0_wall is not None:
            self._wall_duration = max(0.0, t_wall - self._t0_wall)

    def _startup_delay(self) -> float:
        if self._t0_sim is not None and self.first_motion_t is not None:
            return max(0.0, self.first_motion_t - self._t0_sim)
        return 0.0

    def to_dict(self) -> Dict:
        data = {
            "result": self._result,
            "sim_duration_s": self._sim_duration,
            "wall_duration_s": self._wall_duration,
            "startup_delay_s": self._startup_delay(),
            "path_length_m": self.path_length,
            "final_distance_m": self._final_distance,
            "min_goal_distance_m": self.min_goal_distance,
            "attempts": self.attempts,
            "rejections": self.rejections,
            "max_speed_mps": self.max_speed,
            "stopped_time_s": self.stopped_time,
            "contacts": self.contacts,
            "contact_names": sorted(self.contact_names),
            "contacts_available": self.contacts_available,
            "map_used": self.map_used,
            "behavior_tree": self.behavior_tree,
            "map_correction": self.map_correction,
            "collision_monitor": self.collision_monitor,
        }
        json.dumps(data)
        return data

    def to_text(self) -> str:
        """Resume de 6 lignes pour le journal."""
        lines = [
            f"resultat: {self._result}",
            f"duree sim: {self._sim_duration:.1f} s / mur: {self._wall_duration:.1f} s",
            f"chemin: {self.path_length:.2f} m / but final: {self._final_distance}",
            f"tentatives: {self.attempts} rejets: {self.rejections}",
            f"vitesse max: {self.max_speed:.2f} m/s / arret: {self.stopped_time:.1f} s",
            f"contacts: {self.contacts}",
        ]
        return "\n".join(lines)
