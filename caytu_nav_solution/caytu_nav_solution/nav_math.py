"""Calculs purs de la solution Fosa (aucune dépendance ROS).

Tout ce qui est ici se teste avec `pytest`, sans simulateur :
  - poses 2D et changements de repère ;
  - fusion « distance des roues + cap de l'IMU » (localisation) ;
  - inclinaison du robot et distance à laquelle le plan du lidar coupe le sol ;
  - lecture d'une carte PGM et comparaison d'un scan avec cette carte.
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

GRAVITY = 9.80665

# Recalage sur les murs : seuils valides dans le cafe (ne pas modifier sans le signaler).
_CORR_R_MIN = 0.45
_CORR_R_MAX = 8.0
_CORR_WALL_MAX_D = 0.30
_CORR_MIN_POINTS = 40
_CORR_IMPROVE_RATIO = 0.15
_CORR_IMPROVE_MIN = 0.005
_CORR_AXIS_MIN = 0.003
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
        roues espacées de 0.393 m). Le cap est donc pris sur l'IMU.

    `update()` renvoie la transformation map -> odom à publier sur /tf : c'est
    exactement le rôle qu'avait AMCL, sans carte ni filtre à particules, donc
    sans « localisation perdue ».
    """

    def __init__(self, spawn: Pose2D, max_step: float = 1.0):
        self._spawn = spawn
        self._max_step = max_step
        self._pose: Optional[Pose2D] = None
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

    def update(self, odom: Pose2D, imu_yaw: Optional[float]) -> Pose2D:
        """Intègre une mesure d'odométrie (et le cap IMU s'il est disponible).

        odom    : pose base_footprint dans `odom`, telle que publiée par Gazebo.
        imu_yaw : lacet absolu de l'IMU (rad), ou None si l'IMU est muette.
        Retourne map -> odom.
        """
        if self._pose is None or self._last_odom is None:
            self._pose = self._spawn.compose(odom)
            self._last_odom = odom
            self._imu_offset = None
            if imu_yaw is not None:
                self._imu_offset = normalize_angle(self._pose.yaw - imu_yaw)
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
                self._imu_offset = normalize_angle(self._pose.yaw - imu_yaw)
            new_yaw = normalize_angle(imu_yaw + self._imu_offset)
        else:
            self._imu_offset = None
            new_yaw = normalize_angle(self._pose.yaw + dyaw_odom)

        mid_yaw = self._pose.yaw + 0.5 * normalize_angle(new_yaw - self._pose.yaw)
        self._pose = Pose2D(
            self._pose.x + ds * math.cos(mid_yaw),
            self._pose.y + ds * math.sin(mid_yaw),
            new_yaw,
        )
        self._last_odom = odom
        return self._pose.compose(odom.inverse())

    def apply_correction(self, dx: float, dy: float) -> None:
        """Ajoute (dx, dy) a la pose estimee, sans toucher au cap."""
        if self._pose is not None:
            self._pose = Pose2D(self._pose.x + dx, self._pose.y + dy, self._pose.yaw)


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
    """
    sp, cp = math.sin(plane_pitch), math.cos(plane_pitch)
    sr = math.sin(plane_roll)
    out = []
    for a in angles:
        dz = -sp * math.cos(a) + cp * sr * math.sin(a)   # composante verticale du rayon
        out.append(height / -dz if dz < -1e-6 else math.inf)
    return out


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
    """
    mark, clear = [], []
    n_floor = 0
    for r, f in zip(ranges, floor):
        valid = math.isfinite(r) and r <= range_max
        if valid and math.isfinite(f) and r >= floor_ratio * f:
            n_floor += 1
            mark.append(math.inf)
            clear.append(min(clear_ratio * f, range_max))
        else:
            mark.append(r)
            clear.append(r)
    return mark, clear, n_floor


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
    occupied: List[bytearray]     # occupied[row][col] = 1 si cellule occupée


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
    """Charge un PGM binaire (P5, 8 bits) : pixel sombre = occupé (mode trinary)."""
    tokens, offset = _read_pgm_tokens(pgm_bytes, 4)
    if tokens[0] != b'P5':
        raise ValueError('format PGM non supporté (P5 attendu)')
    width, height, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if maxval > 255:
        raise ValueError('PGM 16 bits non supporté')
    pixels = pgm_bytes[offset:offset + width * height]
    if len(pixels) != width * height:
        raise ValueError('PGM tronqué')
    rows = []
    for r in range(height - 1, -1, -1):          # le PGM commence par y maximal
        line = pixels[r * width:(r + 1) * width]
        rows.append(bytearray(1 if p < occupied_below else 0 for p in line))
    return GridMap(width, height, resolution, origin_x, origin_y, rows)


def scan_map_agreement(grid: GridMap, sensor: Pose2D, angle_min: float,
                       angle_increment: float, ranges: Sequence[float],
                       min_range: float = 0.45, max_range: float = 11.5) -> Tuple[float, int]:
    """Part des retours du scan a 0.20 m ou moins d'un mur (champ de distance)."""
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
    """Champ de distance aux murs, calcule une seule fois au demarrage.

    Retourne un tableau float32 H x W. Aucune boucle sur les cellules :
    une passe vectorisee par decalage (di, dj) dans le disque de rayon R.
    """
    occ = np.asarray(grid.occupied, dtype=bool)
    height, width = occ.shape
    field = np.full((height, width), max_dist, dtype=np.float32)
    radius = int(round(max_dist / grid.resolution))
    res = grid.resolution
    for di in range(-radius, radius + 1):
        for dj in range(-radius, radius + 1):
            if di * di + dj * dj > radius * radius:
                continue
            dist = res * math.hypot(di, dj)
            r0s, r1s = max(0, -di), height - max(0, di)
            c0s, c1s = max(0, -dj), width - max(0, dj)
            r0d, r1d = max(0, di), height - max(0, -di)
            c0d, c1d = max(0, dj), width - max(0, -dj)
            if r0s >= r1s or c0s >= c1s:
                continue
            src = occ[r0s:r1s, c0s:c1s]
            dst = field[r0d:r1d, c0d:c1d]
            np.minimum(dst, np.where(src, dist, max_dist), out=dst)
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


def estimate_translation_correction(field: np.ndarray, grid: GridMap,
                                    lidar_pose: Pose2D, angle_min: float,
                                    angle_increment: float,
                                    ranges: Sequence[float]) -> Optional[Tuple[float, float, float]]:
    """Estime (dx, dy, qualite) en comparant le scan aux murs (O(81 x M))."""
    r = np.asarray(ranges, dtype=np.float64)
    ok = np.isfinite(r) & (r >= _CORR_R_MIN) & (r <= _CORR_R_MAX)
    idx = np.nonzero(ok)[0]
    if idx.size == 0:
        return None
    angles = lidar_pose.yaw + angle_min + idx * angle_increment
    px = lidar_pose.x + r[idx] * np.cos(angles)
    py = lidar_pose.y + r[idx] * np.sin(angles)
    near = field_lookup(field, grid, px, py) <= _CORR_WALL_MAX_D
    px, py = px[near], py[near]
    if px.size < _CORR_MIN_POINTS:
        return None
    shifts = np.linspace(-0.10, 0.10, 9)
    px3 = px[None, None, :] + shifts[:, None, None] + np.zeros((1, 9, 1))
    py3 = py[None, None, :] + np.zeros((9, 1, 1)) + shifts[None, :, None]
    scores = field_lookup(field, grid, px3, py3).mean(axis=2)
    flat = int(np.argmin(scores))
    i, j = flat // 9, flat % 9
    if i == 0 or i == 8 or j == 0 or j == 8:
        return None
    center, best = float(scores[4, 4]), float(scores[i, j])
    if center - best < max(_CORR_IMPROVE_RATIO * center, _CORR_IMPROVE_MIN):
        return None
    dx = float(shifts[i]) if min(float(scores[i - 1, j]), float(scores[i + 1, j])) - best >= _CORR_AXIS_MIN else 0.0
    dy = float(shifts[j]) if min(float(scores[i, j - 1]), float(scores[i, j + 1])) - best >= _CORR_AXIS_MIN else 0.0
    if dx == 0.0 and dy == 0.0:
        return None
    return dx, dy, 1.0 - best / 0.5
