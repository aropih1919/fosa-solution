"""Garder la route choisie ou prendre celle qui vient d'être calculée ? (sans ROS)

Le planificateur recalcule un chemin deux fois par seconde. Devant une table
contournable par la gauche ou par la droite, les deux routes se valent presque.
Or les caméras ne voient que ±45° vers l'avant et leur mesure est bruitée : la
route que le robot regarde paraît toujours un peu plus encombrée que l'autre.
Sans précaution, le chemin passe à gauche, le robot pivote, la gauche paraît
alors plus chère, le chemin passe à droite, le robot pivote...

Règle appliquée ici, à chaque nouveau chemin :

  1. MÊME ROUTE que le chemin suivi (les deux ne s'écartent nulle part de plus
     de `same_route_dist`) : on prend le nouveau. Le chemin continue donc de
     s'ajuster en douceur à ce que voient les capteurs, comme sans ce module.
  2. AUTRE ROUTE : on ne change que si
       - la route suivie est coupée par un obstacle, ou
       - elle frôle un obstacle, deux calculs de suite (une demi-seconde), ou
       - le robot s'en est écarté, ou
       - la nouvelle route fait gagner plus de `min_gain`, plus le prix du
         demi-tour qu'elle impose (`turn_cost` mètres par radian : changer de
         route fait pivoter le robot sur place), et la route suivie a été
         choisie il y a plus de `min_dwell` secondes.

Pourquoi ces deux délais : vus sous un autre angle, les pieds de chaise et les
bords de table apparaissent et disparaissent de la costmap (le lidar ne touche
un pied de 3 cm qu'avec un ou deux rayons, la caméra est bruitée). Le coût d'un
passage étroit peut alors varier de plusieurs mètres équivalents d'un calcul à
l'autre. Une route coupée par un vrai obstacle reste, elle, abandonnée tout de
suite.

Les deux chemins sont notés avec la formule du planificateur lui-même (Smac 2D) :

    coût = somme, sur chaque petit segment, de
           longueur x (1 + cost_travel_multiplier x coût_costmap / 252)

Le coût s'exprime donc en mètres : `min_gain` = 1,5 veut dire « on ne change de
route que pour gagner l'équivalent de 1,5 m ». C'est moins que ce que coûte un
aller-retour de pivots.

Coût du calcul, deux fois par seconde : O(n) pour noter un chemin de n points
(quelques centaines) ; l'écart entre deux chemins est mesuré sur 120 points au
plus par chemin.
"""

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

# Valeurs d'une OccupancyGrid publiée par Nav2 : 0 libre, 1..98 coût croissant,
# 99 = le centre du robot toucherait un obstacle, 100 = obstacle, -1 = inconnu.
OCC_INSCRIBED = 99
# À partir de cette valeur, le centre du robot est à moins de 3 cm de la
# distance où son contour touche l'obstacle : le chemin « frôle ».
OCC_TOO_CLOSE = 90
# Nombre maximal de points par chemin pour mesurer l'écart entre deux chemins.
_DEVIATION_POINTS = 120
# Raisons rendues par choose_path().
FIRST = 'premier chemin'
NEW_GOAL = 'nouveau but'
SAME_ROUTE = 'même route, chemin ajusté'
OFF_PATH = 'robot écarté de la route suivie'
BLOCKED = 'route suivie coupée par un obstacle'
TOO_CLOSE = 'route suivie trop près d\'un obstacle'
BETTER = 'autre route nettement meilleure'
KEPT = 'route suivie gardée'
NO_COSTMAP = 'costmap indisponible'


@dataclass
class CostGrid:
    """Costmap globale telle que publiée par Nav2 (ligne 0 = y minimal)."""

    values: np.ndarray            # H x W, int8 : valeurs d'OccupancyGrid
    resolution: float
    origin_x: float
    origin_y: float

    @classmethod
    def from_flat(cls, data, width: int, height: int, resolution: float,
                  origin_x: float, origin_y: float) -> 'CostGrid':
        values = np.asarray(data, dtype=np.int8).reshape(height, width)
        return cls(values, float(resolution), float(origin_x), float(origin_y))

    def lookup(self, points: np.ndarray) -> np.ndarray:
        """Valeur d'OccupancyGrid sous chaque point (0 hors de la grille)."""
        height, width = self.values.shape
        cols = np.floor((points[:, 0] - self.origin_x) / self.resolution).astype(np.int64)
        rows = np.floor((points[:, 1] - self.origin_y) / self.resolution).astype(np.int64)
        inside = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
        out = np.zeros(points.shape[0], dtype=np.int16)
        out[inside] = self.values[rows[inside], cols[inside]]
        return out


def occupancy_to_cost(occupancy: np.ndarray) -> np.ndarray:
    """Coût de costmap Nav2 (0..254) à partir d'une valeur d'OccupancyGrid.

    Inverse de la table de Nav2 (Costmap2DPublisher) : 0 -> 0, 99 -> 253,
    100 -> 254, et 1..98 -> 1..252 linéairement. L'inconnu (-1) compte comme
    libre : le planificateur est réglé pour le traverser (allow_unknown).
    """
    occ = np.asarray(occupancy, dtype=np.float64)
    cost = np.where(occ >= 1, 1.0 + (occ - 1.0) * 251.0 / 97.0, 0.0)
    cost = np.where(occ >= OCC_INSCRIBED, 253.0, cost)
    cost = np.where(occ >= 100, 254.0, cost)
    return cost


def as_points(points: Sequence[Tuple[float, float]]) -> np.ndarray:
    array = np.asarray(points, dtype=np.float64)
    return array.reshape(-1, 2)


def travel_cost(points: np.ndarray, grid: CostGrid, multiplier: float = 3.0
                ) -> Tuple[float, bool]:
    """(coût du chemin en mètres équivalents, traverse une cellule interdite ?)."""
    if points.shape[0] == 0:
        return 0.0, False
    occupancy = grid.lookup(points)
    blocked = bool(np.any(occupancy >= OCC_INSCRIBED))
    if points.shape[0] == 1:
        return 0.0, blocked
    weight = 1.0 + multiplier * occupancy_to_cost(occupancy) / 252.0
    lengths = np.hypot(np.diff(points[:, 0]), np.diff(points[:, 1]))
    # Chaque segment prend le poids moyen de ses deux extrémités.
    return float(np.sum(lengths * 0.5 * (weight[:-1] + weight[1:]))), blocked


def too_close(points: np.ndarray, grid: CostGrid, skip: float = 0.5) -> bool:
    """Le chemin frôle-t-il un obstacle au-delà de ses `skip` premiers mètres ?

    Le début du chemin est ignoré : si le robot longe déjà une table, ce n'est
    pas une raison de changer de route.
    """
    if points.shape[0] < 2:
        return False
    along = np.concatenate(([0.0], np.cumsum(
        np.hypot(np.diff(points[:, 0]), np.diff(points[:, 1])))))
    ahead = points[along >= skip]
    return bool(ahead.shape[0]) and bool(np.any(grid.lookup(ahead) >= OCC_TOO_CLOSE))


def _thin(points: np.ndarray, limit: int) -> np.ndarray:
    """Au plus `limit` points régulièrement choisis, extrémités comprises."""
    if points.shape[0] <= limit:
        return points
    index = np.unique(np.linspace(0, points.shape[0] - 1, limit).round().astype(np.int64))
    return points[index]


def route_deviation(new: np.ndarray, kept: np.ndarray) -> float:
    """Plus grand écart (m) entre le nouveau chemin et le chemin suivi.

    Pour chaque point de l'un, distance au point le plus proche de l'autre ;
    on retient le pire cas, dans les deux sens. Deux chemins qui contournent un
    obstacle par des côtés opposés s'écartent de plus d'un mètre ; un chemin
    simplement ajusté reste à quelques dizaines de centimètres. Au plus
    120 x 120 distances.
    """
    a, b = _thin(new, _DEVIATION_POINTS), _thin(kept, _DEVIATION_POINTS)
    if a.shape[0] == 0 or b.shape[0] == 0:
        return math.inf
    d2 = (a[:, None, 0] - b[None, :, 0]) ** 2 + (a[:, None, 1] - b[None, :, 1]) ** 2
    return float(math.sqrt(max(d2.min(axis=1).max(), d2.min(axis=0).max())))


def closest_index(points: np.ndarray, x: float, y: float) -> Tuple[int, float]:
    """(indice du point du chemin le plus proche de (x, y), distance)."""
    d2 = (points[:, 0] - x) ** 2 + (points[:, 1] - y) ** 2
    index = int(np.argmin(d2))
    return index, float(math.sqrt(d2[index]))


def heading_at(points: np.ndarray, ahead: float = 0.6) -> Optional[float]:
    """Direction (rad) du premier point à `ahead` mètres du début du chemin."""
    if points.shape[0] < 2:
        return None
    d = np.hypot(points[:, 0] - points[0, 0], points[:, 1] - points[0, 1])
    far = np.nonzero(d >= ahead)[0]
    target = points[far[0]] if far.size else points[-1]
    if math.hypot(target[0] - points[0, 0], target[1] - points[0, 1]) < 1e-6:
        return None
    return math.atan2(target[1] - points[0, 1], target[0] - points[0, 0])


def turn_between(rest: np.ndarray, new: np.ndarray, ahead: float = 0.6) -> float:
    """Angle (rad, 0 à pi) entre le départ de la route suivie et celui de la nouvelle.

    Le robot suit la route gardée : son cap est à peu près celui du début de
    `rest`. Prendre la nouvelle route l'oblige à tourner d'autant.
    """
    a, b = heading_at(rest, ahead), heading_at(new, ahead)
    if a is None or b is None:
        return 0.0
    return abs(math.atan2(math.sin(b - a), math.cos(b - a)))


@dataclass
class Choice:
    use_new: bool
    reason: str
    start_index: int = 0          # premier point du chemin gardé encore devant le robot
    kept_cost: Optional[float] = None
    new_cost: Optional[float] = None
    deviation: Optional[float] = None
    turn: float = 0.0             # pivot (rad) qu'imposerait la nouvelle route

    @property
    def gain(self) -> Optional[float]:
        if self.kept_cost is None or self.new_cost is None:
            return None
        return self.kept_cost - self.new_cost


def choose_path(kept: Optional[np.ndarray], new: np.ndarray, grid: Optional[CostGrid],
                min_gain: float = 1.5, same_route_dist: float = 1.0,
                max_offset: float = 0.50, goal_tolerance: float = 0.10,
                multiplier: float = 3.0, turn_cost: float = 1.0) -> Choice:
    """Décide entre le chemin suivi (`kept`) et le nouveau (`new`), sur un seul calcul.

    Les règles dans le temps (délai entre deux changements de route,
    confirmation d'un « trop près ») sont dans RouteKeeper.

    Les deux sont des tableaux N x 2 de points (x, y) dans le repère de la
    costmap. `new` commence à la position actuelle du robot. Si le chemin
    suivi est gardé, `start_index` donne son premier point encore utile.
    """
    if new.shape[0] == 0:
        raise ValueError('le nouveau chemin est vide')
    if kept is None or kept.shape[0] < 2:
        return Choice(True, FIRST)
    if math.hypot(kept[-1, 0] - new[-1, 0], kept[-1, 1] - new[-1, 1]) > goal_tolerance:
        return Choice(True, NEW_GOAL)

    robot_x, robot_y = float(new[0, 0]), float(new[0, 1])
    index, offset = closest_index(kept, robot_x, robot_y)
    rest = kept[index:]
    if rest.shape[0] < 2:
        return Choice(True, FIRST)
    deviation = route_deviation(new, rest)
    if deviation <= same_route_dist:
        return Choice(True, SAME_ROUTE, index, deviation=deviation)
    if offset > max_offset:
        return Choice(True, OFF_PATH, index, deviation=deviation)
    if grid is None:
        return Choice(True, NO_COSTMAP, index, deviation=deviation)

    kept_cost, kept_blocked = travel_cost(rest, grid, multiplier)
    kept_cost += offset                       # rejoindre le chemin gardé a aussi un coût
    new_cost, _ = travel_cost(new, grid, multiplier)
    if kept_blocked:
        return Choice(True, BLOCKED, index, kept_cost, new_cost, deviation)
    if too_close(rest, grid):
        return Choice(True, TOO_CLOSE, index, kept_cost, new_cost, deviation)
    turn = turn_between(rest, new)
    if kept_cost - new_cost > min_gain + turn_cost * turn:
        return Choice(True, BETTER, index, kept_cost, new_cost, deviation, turn)
    return Choice(False, KEPT, index, kept_cost, new_cost, deviation, turn)


class RouteKeeper:
    """Mémoire de la route suivie et règles dans le temps (sans ROS).

    `update()` reçoit chaque nouveau chemin du planificateur (points et, à
    côté, n'importe quelle liste de même longueur, par exemple les poses ROS)
    et rend la décision ainsi que la liste à transmettre au contrôleur.
    """

    def __init__(self, min_gain: float = 1.5, same_route_dist: float = 1.0,
                 max_offset: float = 0.50, min_dwell: float = 5.0,
                 reset_after: float = 2.0, multiplier: float = 3.0,
                 goal_tolerance: float = 0.30, turn_cost: float = 1.0):
        self.min_gain = float(min_gain)
        self.turn_cost = float(turn_cost)
        self.same_route_dist = float(same_route_dist)
        self.max_offset = float(max_offset)
        self.min_dwell = float(min_dwell)
        self.reset_after = float(reset_after)
        self.multiplier = float(multiplier)
        self.goal_tolerance = float(goal_tolerance)
        self.kept: Optional[np.ndarray] = None
        self.payload: list = []
        self.goal: Optional[Tuple[float, float]] = None
        self.last_request: Optional[float] = None
        self.last_switch: Optional[float] = None
        self.close_streak = 0
        self.switches = 0             # changements de route
        self.holds = 0                # autres routes refusées

    def forget(self) -> None:
        """Plus de route en mémoire (échec du planificateur, nouveau départ)."""
        self.kept, self.payload = None, []
        self.last_switch, self.close_streak = None, 0

    def update(self, now: float, new: np.ndarray, payload: list,
               grid: Optional[CostGrid], goal: Tuple[float, float]):
        """Retourne (Choice, liste à transmettre)."""
        if len(payload) != new.shape[0]:
            raise ValueError('chemin et poses de longueurs différentes')
        if self.last_request is not None and abs(now - self.last_request) > self.reset_after:
            self.forget()                        # reprise après une récupération
        self.last_request = now
        if self.goal is None or math.hypot(goal[0] - self.goal[0], goal[1] - self.goal[1]) > 0.01:
            self.forget()                        # nouveau but
        self.goal = (float(goal[0]), float(goal[1]))

        choice = choose_path(
            self.kept, new, grid, min_gain=self.min_gain,
            same_route_dist=self.same_route_dist, max_offset=self.max_offset,
            goal_tolerance=self.goal_tolerance, multiplier=self.multiplier,
            turn_cost=self.turn_cost)
        if choice.reason == TOO_CLOSE:
            self.close_streak += 1
            if self.close_streak < 2:
                choice.use_new = False           # à confirmer au calcul suivant
        else:
            self.close_streak = 0
        if choice.reason == BETTER and self.last_switch is not None \
                and now - self.last_switch < self.min_dwell:
            choice.use_new = False               # route choisie il y a trop peu de temps

        if choice.use_new:
            if choice.reason in (BLOCKED, TOO_CLOSE, BETTER, OFF_PATH):
                self.switches += 1
                self.last_switch = now
                self.close_streak = 0
            self.kept, self.payload = new, list(payload)
        else:
            self.holds += 1
            self.kept = self.kept[choice.start_index:]
            self.payload = self.payload[choice.start_index:]
        return choice, self.payload
