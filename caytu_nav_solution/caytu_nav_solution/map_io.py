"""Cartes générées par slam_toolbox : fichiers, métadonnées et retouches.

Aucune dépendance ROS : tout se teste avec `pytest`, sans simulateur.

Un dossier de carte (par défaut ~/fosa_maps/<nom>) contient :
  map.pgm, map.yaml        la carte brute de slam_toolbox (format map_server) ;
  map.posegraph, map.data  le graphe de slam_toolbox, pour se localiser dessus ;
  nav_map.pgm, .yaml       la carte retouchée avec edit_map (facultative) ;
  fosa_map.yaml            nos métadonnées : pose de départ, lien avec Gazebo ;
  auto/                    la dernière sauvegarde automatique, même contenu.
"""

import datetime
import math
import os
import shutil
from array import array
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import yaml

from caytu_nav_solution.nav_math import Pose2D

PGM_FREE, PGM_OCCUPIED, PGM_UNKNOWN = 254, 0, 205
META_FILE = 'fosa_map.yaml'
RAW_MAP = 'map'             # map.pgm / map.yaml
NAV_MAP = 'nav_map'         # nav_map.pgm / nav_map.yaml (retouchée)
POSEGRAPH = 'map'           # map.posegraph / map.data
AUTOSAVE_DIR = 'auto'

# Seuils de map_server en mode trinary. free_thresh = 0.196 et non 0.25 :
# un pixel 205 vaut (255 - 205) / 255 = 0.196, il doit rester « inconnu ».
OCCUPIED_THRESH = 0.65
FREE_THRESH = 0.196


# --------------------------------------------------------------------------- #
# Dossiers
# --------------------------------------------------------------------------- #
def default_maps_dir() -> str:
    """Dossier des cartes : $FOSA_MAPS_DIR, sinon ~/fosa_maps."""
    return os.path.expanduser(os.environ.get('FOSA_MAPS_DIR', '~/fosa_maps'))


def check_map_name(name: str) -> str:
    """Un nom de carte devient un nom de dossier : ni vide, ni chemin."""
    name = str(name).strip()
    if not name or name in ('.', '..') or '/' in name or '\\' in name:
        raise ValueError(f'nom de carte invalide : {name!r}')
    return name


def prepare_map_folder(maps_dir: str, name: str,
                       now: Optional[datetime.datetime] = None) -> Tuple[str, Optional[str]]:
    """Crée <maps_dir>/<name> pour une nouvelle cartographie.

    Une carte déjà présente sous ce nom n'est jamais écrasée : elle est
    déplacée dans <name>_ancienne_<date>. Retourne (dossier, ancien dossier
    déplacé ou None).
    """
    name = check_map_name(name)
    maps_dir = os.path.expanduser(maps_dir)
    folder = os.path.join(maps_dir, name)
    moved = None
    if os.path.isdir(folder) and os.listdir(folder):
        stamp = (now or datetime.datetime.now()).strftime('%Y%m%d-%H%M%S')
        moved = os.path.join(maps_dir, f'{name}_ancienne_{stamp}')
        suffix = 2
        while os.path.exists(moved):
            moved = os.path.join(maps_dir, f'{name}_ancienne_{stamp}_{suffix}')
            suffix += 1
        shutil.move(folder, moved)
    os.makedirs(folder, exist_ok=True)
    return folder, moved


def _write_atomic(path: str, data: bytes):
    """Écrit un fichier sans jamais laisser de version à moitié écrite."""
    tmp = path + '.tmp'
    with open(tmp, 'wb') as stream:
        stream.write(data)
    os.replace(tmp, path)


# --------------------------------------------------------------------------- #
# Grille d'occupation <-> PGM
# --------------------------------------------------------------------------- #
def _pixel_table(occupied: int, free: int) -> bytes:
    """Table octet signé (valeur d'OccupancyGrid) -> niveau de gris PGM."""
    table = bytearray(256)
    for byte in range(256):
        value = byte - 256 if byte > 127 else byte
        if value < 0 or value > 100:
            table[byte] = PGM_UNKNOWN
        elif value >= occupied:
            table[byte] = PGM_OCCUPIED
        elif value <= free:
            table[byte] = PGM_FREE
        else:
            table[byte] = PGM_UNKNOWN
    return bytes(table)


def occupancy_to_pixels(width: int, height: int, data: Sequence[int],
                        occupied: int = 65, free: int = 25) -> bytes:
    """OccupancyGrid.data (ligne 0 = y minimal) -> pixels PGM (ligne 0 = y maximal).

    Mêmes seuils que map_saver : >= 65 occupé, <= 25 libre, sinon inconnu.
    """
    if len(data) != width * height:
        raise ValueError(f'{len(data)} cellules pour une grille {width} x {height}')
    raw = array('b', data).tobytes().translate(_pixel_table(occupied, free))
    rows = [raw[r * width:(r + 1) * width] for r in range(height)]
    return b''.join(reversed(rows))


def read_pgm(pgm_bytes: bytes) -> Tuple[int, int, bytearray]:
    """Lit un PGM binaire 8 bits (P5). Retourne (largeur, hauteur, pixels)."""
    tokens, pos = [], 0
    while len(tokens) < 4:
        while pos < len(pgm_bytes) and pgm_bytes[pos:pos + 1].isspace():
            pos += 1
        if pgm_bytes[pos:pos + 1] == b'#':
            while pos < len(pgm_bytes) and pgm_bytes[pos:pos + 1] != b'\n':
                pos += 1
            continue
        start = pos
        while pos < len(pgm_bytes) and not pgm_bytes[pos:pos + 1].isspace():
            pos += 1
        if start == pos:
            raise ValueError('en-tête PGM incomplet')
        tokens.append(pgm_bytes[start:pos])
    if tokens[0] != b'P5':
        raise ValueError('format PGM non supporté (P5 attendu)')
    width, height, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if maxval > 255:
        raise ValueError('PGM 16 bits non supporté')
    pixels = bytearray(pgm_bytes[pos + 1:pos + 1 + width * height])
    if len(pixels) != width * height:
        raise ValueError('PGM tronqué')
    return width, height, pixels


def pgm_bytes(width: int, height: int, pixels: bytes) -> bytes:
    return f'P5\n{width} {height}\n255\n'.encode('ascii') + bytes(pixels)


def map_yaml_text(image: str, resolution: float, origin: Sequence[float]) -> str:
    """Fichier YAML lu par nav2_map_server."""
    return (
        f'image: {image}\n'
        'mode: trinary\n'
        f'resolution: {resolution:.4f}\n'
        f'origin: [{origin[0]:.4f}, {origin[1]:.4f}, {origin[2]:.4f}]\n'
        'negate: 0\n'
        f'occupied_thresh: {OCCUPIED_THRESH}\n'
        f'free_thresh: {FREE_THRESH}\n')


def write_map_files(folder: str, basename: str, width: int, height: int,
                    pixels: bytes, resolution: float, origin: Sequence[float]) -> str:
    """Écrit <basename>.pgm et <basename>.yaml. Retourne le chemin du YAML."""
    os.makedirs(folder, exist_ok=True)
    image = basename + '.pgm'
    _write_atomic(os.path.join(folder, image), pgm_bytes(width, height, pixels))
    yaml_path = os.path.join(folder, basename + '.yaml')
    _write_atomic(yaml_path, map_yaml_text(image, resolution, origin).encode('utf-8'))
    return yaml_path


def load_map_yaml(yaml_path: str) -> dict:
    """Lit un YAML de map_server ; `image` devient un chemin absolu."""
    with open(yaml_path, encoding='utf-8') as stream:
        meta = yaml.safe_load(stream)
    image = meta['image']
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image)
    return {
        'image': image,
        'resolution': float(meta['resolution']),
        'origin': [float(v) for v in meta['origin']],
    }


@dataclass(frozen=True)
class MapStats:
    known_cells: int
    occupied_cells: int
    known_width_m: float        # emprise de la zone connue
    known_height_m: float


def pixel_stats(width: int, height: int, pixels: bytes, resolution: float) -> MapStats:
    """Mesure la zone réellement explorée (cellules libres ou occupées)."""
    known = occupied = 0
    rows, cols = [], []
    for r in range(height):
        line = pixels[r * width:(r + 1) * width]
        n_unknown = line.count(PGM_UNKNOWN)
        if n_unknown == width:
            continue
        known += width - n_unknown
        occupied += line.count(PGM_OCCUPIED)
        rows.append(r)
        first = next(c for c in range(width) if line[c] != PGM_UNKNOWN)
        last = next(c for c in range(width - 1, -1, -1) if line[c] != PGM_UNKNOWN)
        cols.extend((first, last))
    if not rows:
        return MapStats(0, 0, 0.0, 0.0)
    return MapStats(known, occupied,
                    (max(cols) - min(cols) + 1) * resolution,
                    (max(rows) - min(rows) + 1) * resolution)


# --------------------------------------------------------------------------- #
# Retouches (edit_map)
# --------------------------------------------------------------------------- #
def paint_rectangle(pixels: bytearray, width: int, height: int, resolution: float,
                    origin_x: float, origin_y: float,
                    rect: Sequence[float], value: int) -> int:
    """Peint un rectangle (x1, y1, x2, y2 en mètres, repère map). Retourne le
    nombre de cellules modifiées."""
    x1, y1, x2, y2 = (float(v) for v in rect)
    x1, x2 = min(x1, x2), max(x1, x2)
    y1, y2 = min(y1, y2), max(y1, y2)
    c1 = max(0, int(math.floor((x1 - origin_x) / resolution)))
    c2 = min(width - 1, int(math.floor((x2 - origin_x) / resolution)))
    r1 = max(0, int(math.floor((y1 - origin_y) / resolution)))
    r2 = min(height - 1, int(math.floor((y2 - origin_y) / resolution)))
    changed = 0
    for row in range(r1, r2 + 1):
        base = (height - 1 - row) * width       # le PGM commence par y maximal
        for col in range(c1, c2 + 1):
            if pixels[base + col] != value:
                pixels[base + col] = value
                changed += 1
    return changed


# --------------------------------------------------------------------------- #
# Métadonnées fosa_map.yaml
# --------------------------------------------------------------------------- #
def _pose_to_list(pose: Optional[Pose2D]) -> Optional[List[float]]:
    if pose is None:
        return None
    return [round(float(pose.x), 4), round(float(pose.y), 4), round(float(pose.yaw), 4)]


def _pose_from_list(values) -> Optional[Pose2D]:
    if values is None:
        return None
    x, y, yaw = (float(v) for v in values)
    return Pose2D(x, y, yaw)


@dataclass
class MapInfo:
    """Contenu de fosa_map.yaml, plus le dossier où il a été lu."""

    folder: str
    name: str
    created: str = ''
    saved_by: str = ''
    nav_map: str = RAW_MAP + '.yaml'        # carte donnée à Nav2
    posegraph: str = POSEGRAPH              # préfixe des fichiers slam_toolbox
    resolution: float = 0.05
    explored_m: Tuple[float, float] = (0.0, 0.0)
    # Pose du robot dans `map` quand la cartographie a commencé. Le robot doit
    # repartir de ce même endroit pour se localiser sur la carte.
    start_pose: Optional[Pose2D] = None
    # Gazebo seulement : pose du repère `map` dans le repère du monde, pour
    # convertir le but de task_params.yaml.
    world_from_map: Optional[Pose2D] = None

    @property
    def nav_map_path(self) -> str:
        return os.path.join(self.folder, self.nav_map)

    @property
    def posegraph_prefix(self) -> str:
        return os.path.join(self.folder, self.posegraph)

    def has_posegraph(self) -> bool:
        return (os.path.isfile(self.posegraph_prefix + '.posegraph')
                and os.path.isfile(self.posegraph_prefix + '.data'))

    def to_dict(self) -> dict:
        return {
            'name': self.name,
            'created': self.created,
            'saved_by': self.saved_by,
            'nav_map': self.nav_map,
            'posegraph': self.posegraph,
            'resolution': round(float(self.resolution), 4),
            'explored_m': [round(float(v), 2) for v in self.explored_m],
            'start_pose': _pose_to_list(self.start_pose),
            'world_from_map': _pose_to_list(self.world_from_map),
        }


def save_map_info(info: MapInfo):
    header = ('# Carte générée par la cartographie manuelle Fosa (mapping.launch.py).\n'
              '# start_pose     : pose du robot dans `map` au début de la cartographie.\n'
              '# world_from_map : Gazebo seulement, repère `map` vu du monde Gazebo.\n')
    text = header + yaml.safe_dump(info.to_dict(), sort_keys=False, allow_unicode=True)
    _write_atomic(os.path.join(info.folder, META_FILE), text.encode('utf-8'))


def load_map_info(folder: str) -> MapInfo:
    """Lit <folder>/fosa_map.yaml. Lève une erreur explicite si la carte manque."""
    folder = os.path.abspath(os.path.expanduser(folder))
    path = os.path.join(folder, META_FILE)
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f'{path} introuvable : ce dossier ne contient pas de carte Fosa '
            '(sauvegardez-la avec « ros2 run caytu_nav_solution save_map »).')
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    info = MapInfo(
        folder=folder,
        name=str(data.get('name', os.path.basename(folder))),
        created=str(data.get('created', '')),
        saved_by=str(data.get('saved_by', '')),
        nav_map=str(data.get('nav_map', RAW_MAP + '.yaml')),
        posegraph=str(data.get('posegraph', POSEGRAPH)),
        resolution=float(data.get('resolution', 0.05)),
        explored_m=tuple(float(v) for v in data.get('explored_m', (0.0, 0.0))),
        start_pose=_pose_from_list(data.get('start_pose')),
        world_from_map=_pose_from_list(data.get('world_from_map')),
    )
    if not os.path.isfile(info.nav_map_path):
        raise FileNotFoundError(f'carte {info.nav_map_path} introuvable')
    return info


# --------------------------------------------------------------------------- #
# Changements de repère
# --------------------------------------------------------------------------- #
def world_from_map_at_start(spawn: Pose2D, map_from_odom: Pose2D) -> Pose2D:
    """Repère `map` vu du monde Gazebo.

    Dans Gazebo, l'odométrie démarre à (0, 0, 0) au point de spawn : le repère
    odom est donc le spawn vu du monde. slam_toolbox publie map -> odom ; au
    début de la cartographie on en déduit monde <- map = spawn ∘ (map <- odom)⁻¹.
    """
    return spawn.composer(map_from_odom.inverse())


def world_point_in_map(world_from_map: Pose2D, x: float, y: float) -> Tuple[float, float]:
    """Coordonnées dans `map` d'un point donné dans le repère du monde."""
    point = world_from_map.inverse().composer(Pose2D(x, y, 0.0))
    return point.x, point.y


def iter_occupied_points(width: int, height: int, pixels: bytes, resolution: float,
                         origin_x: float, origin_y: float) -> Iterable[Tuple[float, float]]:
    """Centres (x, y) des cellules occupées d'un PGM, dans le repère de la carte."""
    for r in range(height):
        row = height - 1 - r                    # ligne 0 du PGM = y maximal
        base = r * width
        for c in range(width):
            if pixels[base + c] == PGM_OCCUPIED:
                yield (origin_x + (c + 0.5) * resolution,
                       origin_y + (row + 0.5) * resolution)
