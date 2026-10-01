"""Tests hors ROS : scène synthétique fidèle à la géométrie du robot/monde PARC.

Lancer :  python3 -m pytest -q -s test/test_scan_projection.py
"""
import time

import numpy as np

from caytu_camera_perception.scan_projection import ProjectionParams, project_to_scan

CAM = np.array([-0.0525, 0.0, 1.0126])   # top_camera dans base_footprint (calculé depuis l'URDF)
SIGMA = 0.10                              # bruit gaussien déclaré dans d435.xacro


def render(scene, row_stride=2, col_stride=4, sigma=SIGMA, rng=None):
    """Lance des rayons (FOV horizontal 90 deg, 640x480) contre les surfaces de la scène."""
    rng = rng or np.random.default_rng(0)
    u, v = np.meshgrid(np.arange(0, 640, col_stride) + 0.5, np.arange(0, 480, row_stride) + 0.5)
    d = np.stack([np.ones_like(u), -(u - 320) / 320.0, -(v - 240) / 320.0], -1).reshape(-1, 3)
    du = d / np.linalg.norm(d, axis=1, keepdims=True)
    best = np.full(len(du), np.inf)
    for surf in scene:
        best = np.minimum(best, surf(du))
    ok = np.isfinite(best)
    r = best + rng.normal(0, sigma, len(best))
    pts = du * r[:, None]
    pts[~ok] = np.nan
    return pts, pts[:, 2] + CAM[2]


def ground(du):
    with np.errstate(divide="ignore", invalid="ignore"):
        t = -CAM[2] / du[:, 2]
    return np.where(du[:, 2] < 0, t, np.inf)


def wall_at(x_world):
    def f(du):
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (x_world - CAM[0]) / du[:, 0]
        return np.where(du[:, 0] > 0, t, np.inf)
    return f


def table(x0, half=0.4565, z_top=0.775, z_bot=0.735):
    """Plateau de cafe_table (0.913 x 0.913 x 0.04) : face du dessus + chant avant."""
    def f(du):
        out = np.full(len(du), np.inf)
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (z_top - CAM[2]) / du[:, 2]
            p = CAM + du * t[:, None]
            m = (du[:, 2] < 0) & (p[:, 0] >= x0) & (p[:, 0] <= x0 + 2 * half) & (abs(p[:, 1]) <= half)
            out[m] = t[m]
            t2 = (x0 - CAM[0]) / du[:, 0]
            p2 = CAM + du * t2[:, None]
            m2 = (du[:, 0] > 0) & (p2[:, 2] >= z_bot) & (p2[:, 2] <= z_top) & (abs(p2[:, 1]) <= half)
            out = np.where(m2, np.minimum(out, t2), out)
        return out
    return f


def azimuths(p):
    return p.angle_min + (np.arange(p.n_bins) + 0.5) * p.angle_inc


def test_ground_alone_never_creates_obstacle():
    p = ProjectionParams()
    rng = np.random.default_rng(1)
    false_pos = 0
    for _ in range(300):
        pts, zb = render([ground, wall_at(9.0)], rng=rng)
        rg = project_to_scan(pts, zb, p)
        false_pos += int(np.isfinite(rg).sum())          # valeur finie = obstacle annoncé
    print(f"\n  sol seul : {false_pos} faux positifs sur 300 images")
    assert false_pos == 0


def test_tabletop_detected_at_all_useful_ranges():
    p = ProjectionParams()
    print()
    for x_edge in (1.0, 2.0, 3.0, 3.5):
        rng = np.random.default_rng(2)
        hits, errs = [], []
        half_ang = np.arctan2(0.4565, x_edge - CAM[0])
        core = np.abs(azimuths(p)) < 0.6 * half_ang
        for _ in range(100):
            pts, zb = render([ground, table(x_edge), wall_at(9.0)], rng=rng)
            rg = project_to_scan(pts, zb, p)
            hits.append(np.isfinite(rg[core]).mean())
            fin = rg[core][np.isfinite(rg[core])]
            if fin.size:
                errs.append(np.median(fin) - (x_edge - CAM[0]))
        rate, err = float(np.mean(hits)), float(np.mean(errs))
        print(f"  bord de table à {x_edge:.1f} m : détecté {rate*100:5.1f} % des images, biais distance {err*100:+.1f} cm")
        assert rate > 0.95, (x_edge, rate)
        assert abs(err) < 0.20


def test_free_space_is_free_and_blind_is_unknown():
    p = ProjectionParams()
    pts, zb = render([ground, table(2.0), wall_at(9.0)], rng=np.random.default_rng(3))
    rg = project_to_scan(pts, zb, p)
    az = azimuths(p)
    side = (np.abs(az) > 0.6) & (np.abs(az) < 0.77)
    assert np.isposinf(rg[side]).mean() > 0.9        # hors table, dans le FOV : libre
    assert np.isnan(rg[np.abs(az) > 0.79]).all()     # au-delà du FOV réel : inconnu
    assert not np.isnan(rg[np.abs(az) < 0.75]).any() # aucun secteur vide dans le FOV


def test_processing_time_budget():
    p = ProjectionParams()
    pts, zb = render([ground, table(2.0), wall_at(9.0)])
    t0 = time.perf_counter()
    for _ in range(50):
        project_to_scan(pts, zb, p)
    ms = (time.perf_counter() - t0) / 50 * 1e3
    print(f"\n  {len(pts)} points -> {ms:.2f} ms / image")
    assert ms < 30.0


def test_row_decimation_comparison():
    """Mesure (informative) : décimer les lignes par 4 au lieu de 2 ne coûte rien ici."""
    p = ProjectionParams()
    core = np.abs(azimuths(p)) < 0.06

    def rate(rs):
        rng = np.random.default_rng(4)
        h = []
        for _ in range(100):
            pts, zb = render([ground, table(3.5), wall_at(9.0)], row_stride=rs, rng=rng)
            h.append(np.isfinite(project_to_scan(pts, zb, p)[core]).mean())
        return np.mean(h)

    r2, r4 = rate(2), rate(4)
    print(f"\n  plateau à 3.5 m : détection {r2*100:.0f} % (lignes/2)  vs  {r4*100:.0f} % (lignes/4)")
    assert r2 >= 0.95 and r4 >= 0.90
