#!/usr/bin/env python3
"""Genere la carte statique du cafe PARC directement depuis le monde officiel.

Outil de developpement (non execute pendant la tache). Il lit le monde et le
modele `cafe` du depot officiel PARC2026-Engineers-League et produit une carte
d'occupation 2D dans le REPERE GAZEBO : les coordonnees de task_params.yaml
(spawn et but) s'utilisent donc telles quelles dans le repere `map`.

Seule la structure fixe du batiment est cartographiee (murs, comptoir, cuisine,
banc, distributeur integre au modele). Tables, chaises et visiteurs sont
volontairement absents : ils peuvent etre deplaces et sont detectes en direct
par le lidar et les cameras.

Dependances : numpy, trimesh, pycollada, pillow.

Usage :
    python3 generate_world_map.py <chemin>/parc_robot_bringup <dossier_sortie>
"""
import sys
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path

import numpy as np
import trimesh
from PIL import Image

RESOLUTION = 0.05          # m / cellule (identique aux costmaps)
MARGIN = 0.50              # bordure inconnue autour du batiment (m)
BAND_LOW = 0.04            # hauteur min au-dessus du sol prise en compte (m)
BAND_HIGH = 1.40           # hauteur max : le robot mesure 1.35 m
SAMPLES_PER_M2 = 6000      # densite d'echantillonnage des surfaces
INCH = 0.0254              # cafe.dae est modelise en pouces

OCCUPIED, FREE, UNKNOWN = 0, 254, 205


def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def pose_of(text):
    v = [float(x) for x in (text or '0 0 0 0 0 0').split()]
    return np.array(v[:3]), rpy_matrix(*v[3:])


def main(bringup_dir, out_dir):
    bringup = Path(bringup_dir)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    world = ET.parse(bringup / 'worlds' / 'task_world.sdf').getroot().find('world')
    cafe_inc = next(i for i in world.findall('include')
                    if i.findtext('uri') == 'model://cafe')
    t_cafe, r_cafe = pose_of(cafe_inc.findtext('pose'))

    model = ET.parse(bringup / 'models' / 'cafe' / 'model.sdf').getroot().find('model')
    link = model.find('link')

    # ---- maillage visuel (ce que voient lidar et cameras) -------------------
    mesh = trimesh.load(bringup / 'models' / 'cafe' / 'meshes' / 'cafe.dae',
                        force='mesh', process=False)
    verts = (r_cafe @ (np.asarray(mesh.vertices) * INCH).T).T + t_cafe
    faces = np.asarray(mesh.faces)

    # ---- boites de collision (ce que le robot peut heurter) -----------------
    boxes = []
    floor_box = None
    for col in link.findall('collision'):
        size = np.array([float(x) for x in col.find('geometry/box/size').text.split()])
        t, r = pose_of(col.findtext('pose'))
        center = r_cafe @ t + t_cafe
        rot = r_cafe @ r
        if col.get('name') == 'main_floor':
            floor_box = (center, size)
        else:
            boxes.append((col.get('name'), center, rot, size))
    floor_z = floor_box[0][2] + floor_box[1][2] / 2.0
    z_lo, z_hi = floor_z + BAND_LOW, floor_z + BAND_HIGH

    # ---- emprise de la carte ------------------------------------------------
    fc, fs = floor_box
    x_min = np.floor((fc[0] - fs[0] / 2 - MARGIN) / RESOLUTION) * RESOLUTION
    y_min = np.floor((fc[1] - fs[1] / 2 - MARGIN) / RESOLUTION) * RESOLUTION
    x_max = fc[0] + fs[0] / 2 + MARGIN
    y_max = fc[1] + fs[1] / 2 + MARGIN
    width = int(np.ceil((x_max - x_min) / RESOLUTION))
    height = int(np.ceil((y_max - y_min) / RESOLUTION))
    occ = np.zeros((height, width), dtype=bool)      # ligne 0 = y_min

    def mark(points):
        cx = np.floor((points[:, 0] - x_min) / RESOLUTION).astype(int)
        cy = np.floor((points[:, 1] - y_min) / RESOLUTION).astype(int)
        ok = (cx >= 0) & (cx < width) & (cy >= 0) & (cy < height)
        occ[cy[ok], cx[ok]] = True

    # 1) surfaces du maillage dans la bande de hauteur du robot
    tri = verts[faces]
    zmin, zmax = tri[:, :, 2].min(1), tri[:, :, 2].max(1)
    sel = (zmax > z_lo) & (zmin < z_hi)
    tri = tri[sel]
    area = 0.5 * np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
    rng = np.random.default_rng(0)
    counts = np.maximum(3, np.ceil(area * SAMPLES_PER_M2).astype(int))
    idx = np.repeat(np.arange(len(tri)), counts)
    u, v = rng.random(len(idx)), rng.random(len(idx))
    flip = u + v > 1
    u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
    pts = tri[idx, 0] + u[:, None] * (tri[idx, 1] - tri[idx, 0]) + v[:, None] * (tri[idx, 2] - tri[idx, 0])
    pts = np.vstack([pts, tri.reshape(-1, 3)])
    pts = pts[(pts[:, 2] > z_lo) & (pts[:, 2] < z_hi)]
    mark(pts)
    n_mesh = int(occ.sum())

    # 2) boites de collision dans la meme bande
    for name, center, rot, size in boxes:
        if center[2] + size[2] / 2 < z_lo or center[2] - size[2] / 2 > z_hi:
            continue                                   # linteau de porte, etc.
        nx = max(2, int(size[0] / (RESOLUTION / 3)) + 1)
        ny = max(2, int(size[1] / (RESOLUTION / 3)) + 1)
        gx, gy = np.meshgrid(np.linspace(-size[0] / 2, size[0] / 2, nx),
                             np.linspace(-size[1] / 2, size[1] / 2, ny))
        local = np.stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)], 1)
        mark((rot @ local.T).T + center)

    # ---- espace libre : remplissage depuis l'interieur de la salle ----------
    in_floor = np.zeros_like(occ)
    cx0 = int(np.floor((fc[0] - fs[0] / 2 - x_min) / RESOLUTION))
    cx1 = int(np.ceil((fc[0] + fs[0] / 2 - x_min) / RESOLUTION))
    cy0 = int(np.floor((fc[1] - fs[1] / 2 - y_min) / RESOLUTION))
    cy1 = int(np.ceil((fc[1] + fs[1] / 2 - y_min) / RESOLUTION))
    in_floor[cy0:cy1, cx0:cx1] = True
    free = np.zeros_like(occ)
    seed = (int((fc[1] - y_min) / RESOLUTION), int((fc[0] - x_min) / RESOLUTION))
    # la graine doit etre dans une cellule libre : on cherche autour du centre
    if occ[seed]:
        ys, xs = np.where(in_floor & ~occ)
        k = np.argmin((ys - seed[0]) ** 2 + (xs - seed[1]) ** 2)
        seed = (ys[k], xs[k])
    queue = deque([seed])
    free[seed] = True
    while queue:
        y, x = queue.popleft()
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            yy, xx = y + dy, x + dx
            if 0 <= yy < height and 0 <= xx < width and in_floor[yy, xx] \
                    and not occ[yy, xx] and not free[yy, xx]:
                free[yy, xx] = True
                queue.append((yy, xx))

    grid = np.full((height, width), UNKNOWN, dtype=np.uint8)
    grid[free] = FREE
    grid[occ] = OCCUPIED
    Image.fromarray(np.flipud(grid), mode='L').save(out / 'cafe_map.pgm')   # ligne 0 du PGM = y_max
    (out / 'cafe_map.yaml').write_text(
        'image: cafe_map.pgm\n'
        'mode: trinary\n'
        f'resolution: {RESOLUTION:.3f}\n'
        f'origin: [{x_min:.3f}, {y_min:.3f}, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.196\n')
    print(f'sol z={floor_z:.3f} m ; bande [{z_lo:.3f}, {z_hi:.3f}]')
    print(f'carte {width}x{height} cellules, origine ({x_min:.3f}, {y_min:.3f}), '
          f'etendue x[{x_min:.2f},{x_min + width * RESOLUTION:.2f}] y[{y_min:.2f},{y_min + height * RESOLUTION:.2f}]')
    print(f'occupees: {int(occ.sum())} (dont maillage {n_mesh}) ; libres: {int(free.sum())} '
          f'({free.sum() * RESOLUTION ** 2:.0f} m2)')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
