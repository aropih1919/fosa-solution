"""Detecteur d'evenements de navigation, sans dependance ROS.

Chaque methode recoit le temps `t` (secondes, horloge du noeud) et renvoie
une liste de tuples `(niveau, etiquette, texte)` avec niveau dans
{`info`, `warn`, `error`}. Etat interne : uniquement des scalaires
(temps, compteurs, derniers codes), soit O(1) en temps et memoire par appel.
"""

from typing import List, Tuple

Event = Tuple[str, str, str]

# Seuils issus du comportement observe dans Gazebo (ne pas modifier sans le signaler).
_SPIN_LIN_MAX = 0.02
_SPIN_ANG_MIN = 0.15
_SPIN_REMIND_AFTER = 6.0
_SPIN_REMIND_EVERY = 4.0
_SPIN_MIN_DUR = 0.8
_STOP_LIN_MAX = 0.01
_STOP_ANG_MAX = 0.01
_STOP_AFTER = 4.0
_STOP_EVERY = 3.0
_MOVE_LIN_MIN = 0.02
_MOVE_EVERY = 2.0
_FEEDBACK_EVERY = 3.0
_FEEDBACK_DIST = 0.10
_PLAN_MIN_GAP = 1.5
_PLAN_LEN_EPS = 0.05


class NavEventDetector:
    """Agrege les messages Nav2 en evenements lisibles dans un terminal."""

    def __init__(self) -> None:
        self._last_code = None
        self.goal_active = False
        self._last_plan_n = None
        self._last_plan_len = 0.0
        self._last_plan_t = 0.0
        self.replans = 0
        self._last_fb_t = 0.0
        self._last_fb_dist = 0.0
        self._fb_init = False
        self._spin_start = None
        self._spin_last_remind = 0.0
        self.spin_time = 0.0
        self._stop_start = None
        self._stop_last_emit = 0.0
        self._stop_prev_t = 0.0
        self.stop_time = 0.0
        self._last_move_t = 0.0
        self._move_init = False

    def on_status(self, code: int, t: float) -> List[Event]:
        """Statut de l'action NavigateToPose (1 actif, 4 reussi, 5 annule, 6 echoue)."""
        if code == self._last_code:
            return []
        self._last_code = code
        self.goal_active = (code == 1)
        if code == 1:
            return [("info", "GOAL", "navigation active")]
        if code == 4:
            return [("info", "DONE", "but atteint")]
        if code == 5:
            return [("warn", "STOP", "navigation annulee")]
        if code == 6:
            return [("error", "FAIL", "navigation interrompue par Nav2")]
        return []

    def on_plan(self, n_points: int, length: float, t: float) -> List[Event]:
        """Chemin recu sur /plan (longueur deja calculee par l'appelant)."""
        if n_points == 0:
            return [("error", "FAIL", "aucun chemin")]
        if (self._last_plan_n == n_points
                and abs(length - self._last_plan_len) < _PLAN_LEN_EPS
                and t - self._last_plan_t < _PLAN_MIN_GAP):
            return []
        self._last_plan_n = n_points
        self._last_plan_len = length
        self._last_plan_t = t
        self.replans += 1
        return [("info", "PLAN", f"chemin : {n_points} points, {length:.2f} m")]

    def on_feedback(self, distance: float, t: float) -> List[Event]:
        """Retour d'action : distance restante au but."""
        if not self._fb_init:
            self._fb_init = True
            self._last_fb_t = t
            self._last_fb_dist = distance
            return [("info", "MOVE", f"reste {distance:.2f} m")]
        if t - self._last_fb_t >= _FEEDBACK_EVERY or abs(distance - self._last_fb_dist) >= _FEEDBACK_DIST:
            self._last_fb_t = t
            self._last_fb_dist = distance
            return [("info", "MOVE", f"reste {distance:.2f} m")]
        return []

    def on_cmd(self, lin: float, ang: float, t: float) -> List[Event]:
        """Commande de vitesse : rotation, arret prolonge ou avance."""
        events: List[Event] = []
        is_spin = abs(lin) < _SPIN_LIN_MAX and abs(ang) > _SPIN_ANG_MIN
        is_stopped = (abs(lin) < _STOP_LIN_MAX and abs(ang) < _STOP_ANG_MAX
                      and self.goal_active)
        is_move = abs(lin) > _MOVE_LIN_MIN and not is_spin

        if is_spin:
            if self._spin_start is None:
                self._spin_start = t
                self._spin_last_remind = t
                events.append(("info", "SPIN", f"rotation sur place ang={ang:+.2f} rad/s"))
            elif (t - self._spin_start > _SPIN_REMIND_AFTER
                    and t - self._spin_last_remind >= _SPIN_REMIND_EVERY):
                self._spin_last_remind = t
                events.append(("warn", "SPIN",
                               f"rotation depuis {t - self._spin_start:.0f} s"))
            if self._stop_start is not None:
                self._stop_start = None
        else:
            if self._spin_start is not None:
                dur = t - self._spin_start
                self._spin_start = None
                if dur > _SPIN_MIN_DUR:
                    self.spin_time += dur
                    events.append(("info", "SPIN", f"fin de rotation ({dur:.1f} s)"))

        if is_stopped and not is_spin:
            if self._stop_start is None:
                self._stop_start = t
                self._stop_last_emit = t
                self._stop_prev_t = t
            else:
                self.stop_time += t - self._stop_prev_t
                self._stop_prev_t = t
                dur = t - self._stop_start
                if dur >= _STOP_AFTER and t - self._stop_last_emit >= _STOP_EVERY:
                    self._stop_last_emit = t
                    events.append(("warn", "STOP", f"a l'arret depuis {dur:.0f} s"))
        else:
            if self._stop_start is not None and not is_spin:
                self.stop_time += max(0.0, t - self._stop_prev_t)
            self._stop_start = None

        if is_move:
            if not self._move_init or t - self._last_move_t >= _MOVE_EVERY:
                self._move_init = True
                self._last_move_t = t
                events.append(("info", "MOVE", f"v={lin:+.2f} w={ang:+.2f}"))

        return events
