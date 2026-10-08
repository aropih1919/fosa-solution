"""Calculs purs de la solution Fosa (aucune dépendance ROS).

Tout ce qui est ici se teste avec `pytest`, sans simulateur :
  - poses 2D et changements de repère ;
  - fusion « distance des roues + cap de l'IMU » (localisation) ;
  - inclinaison du robot et distance à laquelle le plan du lidar coupe le sol ;
  - lecture d'une carte PGM et comparaison d'un scan avec cette carte ;
  - point libre le plus proche du but dans une costmap.

Coûts : tout ce qui s'exécute à chaque message de capteur est en O(M) (M = 360
rayons) ou O(1) ; les calculs sur la carte entière (N cellules) ne se font
qu'une fois, au démarrage.
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

GRAVITY = 9.80665

_AGREEMENT_MAX_D = 0.20


# --------------------------------------------------------------------------- #
# Angles et poses 2D
# --------------------------------------------------------------------------- #
def normalize_angle(angle: float) -> float:
    """Ramène un angle dans ]-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Lacet (rotation autour de z) d'un quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """Quaternion (x, y, z, w) d'une rotation pure autour de z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Quaternion (x, y, z, w) pour des angles roulis/tangage/lacet (convention ROS)."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


@dataclass(frozen=True)
class Pose2D:
    """Pose plane : position (m) et cap (rad)."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def compose(self, other: 'Pose2D') -> 'Pose2D':
        """Retourne self ∘ other (other exprimé dans le repère de self)."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2D(
            self.x + c * other.x - s * other.y,
            self.y + s * other.x + c * other.y,
            normalize_angle(self.yaw + other.yaw),
        )

    def inverse(self) -> 'Pose2D':
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2D(
            -(c * self.x + s * self.y),
            -(-s * self.x + c * self.y),
            normalize_angle(-self.yaw),
        )

    def distance_to(self, x: float, y: float) -> float:
        return math.hypot(x - self.x, y - self.y)


# --------------------------------------------------------------------------- #
# Localisation : distance des roues + cap de l'IMU
# --------------------------------------------------------------------------- #
class OdomImuFusion:
    """Estime la pose du robot dans `map` (= repère du monde Gazebo).

    Hypothèses, toutes vérifiées dans le dépôt officiel PARC :
      - l'odométrie du plugin DiffDrive démarre à (0, 0, 0) au point de spawn ;
      - le spawn (x, y, yaw) est donné dans task_params.yaml, en repère Gazebo ;
      - la distance parcourue par les roues est fiable, mais le CAP de cette
        odométrie ne l'est pas forcément (entraxe déclaré 0.3409 m pour des
        roues espacées de 0.393 m). Le cap est donc pris sur l'IMU ;
      - cette odométrie décrit le mouvement du MILIEU DE L'ESSIEU (elle est
        calculée à partir des roues), alors que base_footprint est
        `axle_offset` mètres derrière l'essieu. Quand le robot pivote, l'essieu
        ne bouge pas mais base_footprint décrit un arc : on en tient compte.

    `update()` renvoie la transformation map -> odom à publier sur /tf : c'est
    exactement le rôle qu'avait AMCL, sans carte ni filtre à particules, donc
    sans « localisation perdue ».
    """

    def __init__(self, spawn: Pose2D, max_step: float = 1.0, axle_offset: float = 0.0):
        self._spawn = spawn
        self._max_step = max_step
        self._axle_offset = float(axle_offset)
        self._axle: Optional[Pose2D] = None       # milieu de l'essieu dans `map`
        self._pose: Optional[Pose2D] = None       # base_footprint dans `map`
        self._last_odom: Optional[Pose2D] = None
        self._imu_offset: Optional[float] = None

    @property
    def pose(self) -> Optional[Pose2D]:
        """Dernière pose estimée dans `map` (None avant la première odométrie)."""
        return self._pose

    @property
    def uses_imu(self) -> bool:
        return self._imu_offset is not None

    def initial_transform(self) -> Pose2D:
        """map -> odom avant toute mesure : l'origine d'odom est le spawn."""
        return self._spawn

    def _set_axle(self, axle: Pose2D) -> None:
        """Mémorise l'essieu et en déduit base_footprint, `axle_offset` derrière."""
        self._axle = axle
        self._pose = Pose2D(
            axle.x - self._axle_offset * math.cos(axle.yaw),
            axle.y - self._axle_offset * math.sin(axle.yaw),
            axle.yaw)

    def update(self, odom: Pose2D, imu_yaw: Optional[float]) -> Pose2D:
        """Intègre une mesure d'odométrie (et le cap IMU s'il est disponible).

        odom    : pose base_footprint dans `odom`, telle que publiée par Gazebo.
        imu_yaw : lacet absolu de l'IMU (rad), ou None si l'IMU est muette.
        Retourne map -> odom.
        """
        if self._axle is None or self._last_odom is None:
            # L'essieu part de `axle_offset` devant le spawn ; l'odométrie donne
            # son déplacement depuis ce point.
            start = self._spawn.compose(Pose2D(self._axle_offset, 0.0, 0.0))
            self._set_axle(start.compose(odom))
            self._last_odom = odom
            self._imu_offset = None
            if imu_yaw is not None:
                self._imu_offset = normalize_angle(self._axle.yaw - imu_yaw)
            return self._pose.compose(odom.inverse())

        dx = odom.x - self._last_odom.x
        dy = odom.y - self._last_odom.y
        dyaw_odom = normalize_angle(odom.yaw - self._last_odom.yaw)
        if math.hypot(dx, dy) > self._max_step:
            # Saut d'odométrie (réinitialisation du simulateur) : on ne
            # l'intègre pas comme un déplacement.
            self._last_odom = odom
            return self._pose.compose(odom.inverse())

        # Distance signée le long de l'axe du robot (base différentielle).
        mid_odom_yaw = self._last_odom.yaw + 0.5 * dyaw_odom
        ds = dx * math.cos(mid_odom_yaw) + dy * math.sin(mid_odom_yaw)

        if imu_yaw is not None:
            if self._imu_offset is None:
                # L'IMU (ré)apparaît : on la recale sur le cap courant, sans saut.
                self._imu_offset = normalize_angle(self._axle.yaw - imu_yaw)
            new_yaw = normalize_angle(imu_yaw + self._imu_offset)
        else:
            self._imu_offset = None
            new_yaw = normalize_angle(self._axle.yaw + dyaw_odom)

        mid_yaw = self._axle.yaw + 0.5 * normalize_angle(new_yaw - self._axle.yaw)
        self._set_axle(Pose2D(
            self._axle.x + ds * math.cos(mid_yaw),
            self._axle.y + ds * math.sin(mid_yaw),
            new_yaw,
        ))
        self._last_odom = odom
        return self._pose.compose(odom.inverse())


# --------------------------------------------------------------------------- #
# Inclinaison du robot et intersection du plan lidar avec le sol
# --------------------------------------------------------------------------- #
def tilt_from_accel(ax: float, ay: float, az: float) -> Tuple[float, float]:
    """Roulis et tangage (rad) d'un capteur immobile à partir de son accéléromètre.

    Convention ROS : tangage positif = nez vers le bas. Au repos, le capteur
    mesure la réaction à la gravité, soit g * (-sin p, cos p sin r, cos p cos r).
    """
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.hypot(ay, az))
    return roll, pitch


def lidar_height(robot_pitch: float, mount_z: float, lever_x: float) -> float:
    """Hauteur du lidar au-dessus du sol quand le robot pivote sur ses roues.

    mount_z : hauteur du lidar dans base_footprint (robot à plat).
    lever_x : distance horizontale entre l'axe des roues et le lidar (positive
              si le lidar est derrière l'axe : il monte quand le nez descend).
    """
    return mount_z * math.cos(robot_pitch) + lever_x * math.sin(robot_pitch)


def floor_ranges(angles: Sequence[float], plane_pitch: float, plane_roll: float,
                 height: float) -> List[float]:
    """Distance à laquelle chaque rayon du lidar touche le sol (inf sinon).

    angles      : angle de chaque rayon dans le repère du lidar (0 = avant).
    plane_pitch : tangage du plan de balayage dans le monde (positif = l'avant
                  du plan plonge vers le sol).
    plane_roll  : roulis du plan de balayage.
    height      : hauteur du lidar au-dessus du sol.
    Un seul passage vectorisé sur les rayons : O(M).
    """
    a = np.asarray(angles, dtype=np.float64)
    # Composante verticale de chaque rayon (négative = le rayon descend).
    dz = -math.sin(plane_pitch) * np.cos(a) + math.cos(plane_pitch) * math.sin(plane_roll) * np.sin(a)
    out = np.full(a.shape, math.inf)
    down = dz < -1e-6
    out[down] = height / -dz[down]
    return out.tolist()


def split_floor_returns(ranges: Sequence[float], floor: Sequence[float],
                        range_max: float, floor_ratio: float = 0.8,
                        clear_ratio: float = 0.7) -> Tuple[List[float], List[float], int]:
    """Sépare obstacles réels et retours du sol.

    Retourne (marquage, effacement, nombre de retours de sol) :
      - marquage   : le scan sans les retours du sol (remplacés par inf) ;
      - effacement : le scan où un retour de sol devient une distance « libre
        jusque-là » (clear_ratio * distance du sol), pour que la costmap puisse
        encore nettoyer la zone devant le robot.
    Un retour est attribué au sol s'il est au moins à floor_ratio fois la
    distance théorique du sol dans cette direction : au-delà, le rayon serait
    sous le plancher, il ne peut donc s'agir d'un obstacle.
    Un seul passage vectorisé sur les rayons : O(M).
    """
    r = np.asarray(ranges, dtype=np.float64)
    f = np.asarray(floor, dtype=np.float64)
    finite_f = np.isfinite(f)
    limit = np.where(finite_f, floor_ratio * np.where(finite_f, f, 0.0), math.inf)
    is_floor = np.isfinite(r) & (r <= range_max) & finite_f & (r >= limit)
    mark = np.where(is_floor, math.inf, r)
    clear = np.where(is_floor, np.minimum(clear_ratio * np.where(finite_f, f, 0.0), range_max), r)
    return mark.tolist(), clear.tolist(), int(np.count_nonzero(is_floor))


# --------------------------------------------------------------------------- #
# Carte statique : lecture PGM et contrôle de cohérence avec un scan
# --------------------------------------------------------------------------- #
@dataclass
class GridMap:
    """Carte d'occupation minimale (ligne 0 = y minimal)."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    occupied: np.ndarray          # occupied[row][col] vrai si la cellule est occupée

    def __post_init__(self):
        # Accepte aussi une liste de lignes : tout est converti une seule fois.
        self.occupied = np.ascontiguousarray(np.asarray(self.occupied, dtype=bool))


def _read_pgm_tokens(data: bytes, count: int) -> Tuple[List[bytes], int]:
    """Lit `count` jetons d'en-tête PGM en sautant blancs et commentaires."""
    tokens, pos = [], 0
    while len(tokens) < count:
        while pos < len(data) and data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            while pos < len(data) and data[pos:pos + 1] != b'\n':
                pos += 1
            continue
        start = pos
        while pos < len(data) and not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    return tokens, pos + 1          # un seul blanc sépare l'en-tête des données


def load_pgm_map(pgm_bytes: bytes, resolution: float, origin_x: float, origin_y: float,
                 occupied_below: int = 90) -> GridMap:
    """Charge un PGM binaire (P5, 8 bits) : pixel sombre = occupé (mode trinary).

    Lecture en un seul bloc, sans boucle par pixel : O(N).
    """
    tokens, offset = _read_pgm_tokens(pgm_bytes, 4)
    if tokens[0] != b'P5':
        raise ValueError('format PGM non supporté (P5 attendu)')
    width, height, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if maxval > 255:
        raise ValueError('PGM 16 bits non supporté')
    if width <= 0 or height <= 0 or len(pgm_bytes) - offset < width * height:
        raise ValueError('PGM tronqué')
    pixels = np.frombuffer(pgm_bytes, dtype=np.uint8, count=width * height, offset=offset)
    # Le PGM commence par y maximal : on retourne les lignes.
    occupied = pixels.reshape(height, width)[::-1] < occupied_below
    return GridMap(width, height, resolution, origin_x, origin_y, occupied)


def scan_map_agreement(grid: GridMap, sensor: Pose2D, angle_min: float,
                       angle_increment: float, ranges: Sequence[float],
                       min_range: float = 0.45, max_range: float = 11.5,
                       field: Optional[np.ndarray] = None) -> Tuple[float, int]:
    """Part des retours du scan à 0,20 m ou moins d'un mur de la carte.

    `field` : champ de distance déjà calculé (sinon il est construit ici).
    """
    if field is None:
        field = build_distance_field(grid, max_dist=0.5)
    r = np.asarray(ranges, dtype=np.float64)
    ok = np.isfinite(r) & (r >= min_range) & (r <= max_range)
    idx = np.nonzero(ok)[0]
    if idx.size == 0:
        return 0.0, 0
    angles = sensor.yaw + angle_min + idx * angle_increment
    px = sensor.x + r[idx] * np.cos(angles)
    py = sensor.y + r[idx] * np.sin(angles)
    dists = field_lookup(field, grid, px, py)
    hits = int(np.count_nonzero(dists <= _AGREEMENT_MAX_D))
    return hits / idx.size, int(idx.size)


def build_distance_field(grid: GridMap, max_dist: float = 0.5) -> np.ndarray:
    """Distance de chaque cellule au mur le plus proche, plafonnée à max_dist.

    Calcul exact en deux passes séparées (lignes, puis colonnes) :
      1. g = distance au mur le plus proche sur la même ligne ;
      2. d² = min sur les lignes voisines de (g² + écart de lignes²).
    Coût O(N x R) opérations vectorisées (N cellules, R = max_dist en
    cellules), au lieu de O(N x R²) pour un balayage du disque. Calculé une
    seule fois au démarrage. Retourne un tableau float32 H x W, en mètres.
    """
    occ = grid.occupied
    height, width = occ.shape
    radius = int(round(max_dist / grid.resolution))
    far = radius + 1                      # « aucun mur à portée »

    # Passe 1 : distance horizontale, en cellules.
    g = np.where(occ, 0, far).astype(np.int32)
    for d in range(1, min(radius, width - 1) + 1):
        hit = np.zeros_like(occ)
        hit[:, d:] |= occ[:, :-d]         # mur à d cellules à gauche
        hit[:, :-d] |= occ[:, d:]         # mur à d cellules à droite
        np.minimum(g, np.where(hit, d, far), out=g)

    # Passe 2 : combinaison verticale.
    g2 = g * g
    best = g2.copy()
    for d in range(1, min(radius, height - 1) + 1):
        np.minimum(best[d:, :], g2[:-d, :] + d * d, out=best[d:, :])
        np.minimum(best[:-d, :], g2[d:, :] + d * d, out=best[:-d, :])

    field = np.sqrt(best.astype(np.float32)) * np.float32(grid.resolution)
    np.minimum(field, np.float32(max_dist), out=field)
    return field


def field_lookup(field: np.ndarray, grid: GridMap, xs, ys) -> np.ndarray:
    """Distance au mur pour chaque point, O(1) par point vectorise."""
    xs_arr = np.asarray(xs, dtype=np.float64)
    ys_arr = np.asarray(ys, dtype=np.float64)
    cols = np.floor((xs_arr - grid.origin_x) / grid.resolution).astype(np.int64)
    rows = np.floor((ys_arr - grid.origin_y) / grid.resolution).astype(np.int64)
    fallback = float(np.max(field))
    out = np.full(np.shape(xs_arr), fallback, dtype=np.float64)
    valid = (cols >= 0) & (cols < grid.width) & (rows >= 0) & (rows < grid.height)
    if np.any(valid):
        out[valid] = field[rows[valid], cols[valid]]
    return out


# Boîte de collision du châssis dans base_footprint (URDF officiel).
ROBOT_FRONT, ROBOT_REAR, ROBOT_HALF_WIDTH = 0.198, -0.288, 0.222


def footprint_gap(pose: Pose2D, x: float, y: float, front: float = ROBOT_FRONT,
                  rear: float = ROBOT_REAR, half_width: float = ROBOT_HALF_WIDTH) -> float:
    """Distance (m) du point (x, y) au contour du robot ; 0 si le point est dessous. O(1)."""
    dx, dy = x - pose.x, y - pose.y
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    local_x, local_y = c * dx + s * dy, -s * dx + c * dy
    out_x = max(rear - local_x, 0.0, local_x - front)
    out_y = max(abs(local_y) - half_width, 0.0)
    return math.hypot(out_x, out_y)


def cell_cost(values, width: int, height: int, resolution: float,
              origin_x: float, origin_y: float, x: float, y: float) -> Optional[int]:
    """Valeur de la cellule d'une OccupancyGrid contenant (x, y), ou None hors grille. O(1)."""
    col = int(math.floor((x - origin_x) / resolution))
    row = int(math.floor((y - origin_y) / resolution))
    if not (0 <= col < width and 0 <= row < height):
        return None
    return int(values[row * width + col])


def nearest_free_cell(values, width: int, height: int, resolution: float,
                      origin_x: float, origin_y: float,
                      goal_x: float, goal_y: float,
                      max_radius: float, max_cost: int,
                      excluded: Sequence[Tuple[float, float]] = (),
                      exclusion_radius: float = 0.15
                      ) -> Optional[Tuple[float, float, float]]:
    """Centre de la cellule admissible la plus proche du but.

    `values` est le tableau d'une OccupancyGrid (ligne 0 = y minimal,
    -1 inconnu, 0..100). Une cellule est admissible si 0 <= valeur <= max_cost,
    si elle est à moins de max_radius du but et à plus de exclusion_radius de
    chaque point de `excluded`. Retourne (x, y, distance_au_but) ou None.
    Coût : O(k), k = nombre de cellules de la fenêtre carrée autour du but.
    """
    grid = np.asarray(values, dtype=np.int16).reshape(height, width)
    cx = int(math.floor((goal_x - origin_x) / resolution))
    cy = int(math.floor((goal_y - origin_y) / resolution))
    n = int(math.ceil(max_radius / resolution))
    x0, x1 = max(0, cx - n), min(width, cx + n + 1)
    y0, y1 = max(0, cy - n), min(height, cy + n + 1)
    if x0 >= x1 or y0 >= y1:
        return None
    window = grid[y0:y1, x0:x1]
    xs = origin_x + (np.arange(x0, x1) + 0.5) * resolution
    ys = origin_y + (np.arange(y0, y1) + 0.5) * resolution
    d2 = (ys[:, None] - goal_y) ** 2 + (xs[None, :] - goal_x) ** 2
    ok = (window >= 0) & (window <= max_cost) & (d2 <= max_radius ** 2)
    for ex, ey in excluded:
        ok &= ((ys[:, None] - ey) ** 2 + (xs[None, :] - ex) ** 2) > exclusion_radius ** 2
    if not ok.any():
        return None
    j, i = divmod(int(np.argmin(np.where(ok, d2, np.inf))), window.shape[1])
    return float(xs[i]), float(ys[j]), float(math.sqrt(d2[j, i]))
