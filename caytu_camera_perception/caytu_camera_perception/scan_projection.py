"""Coeur du filtre caméra : nuage de points -> LaserScan synthétique.

Module 100 % numpy (aucune dépendance ROS) : testable hors simulation.

Idée : pour chaque secteur d'azimut autour de la caméra, on garde la distance
du premier obstacle situé DANS la bande de hauteur du robot (z_min..z_max),
en ignorant le sol. Un secteur sans obstacle est annoncé « libre » (+inf), ce qui
permet à Nav2 d'EFFACER un obstacle qui a disparu (un PointCloud2 ne sait pas
dire « il n'y a rien ici »). Un secteur sans mesure valide est annoncé NaN
(inconnu : ni marquage, ni effacement).
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class ProjectionParams:
    z_min: float = 0.30       # m au-dessus du sol : marge contre le sol bruité
    z_max: float = 1.40       # m : hauteur du robot (1.349) + marge
    r_min: float = 0.30       # m : portée minimale utile
    r_max: float = 4.0        # m : portée maximale publiée
    angle_min: float = -0.795  # rad (FOV caméra = +-0.785)
    angle_max: float = 0.795
    # DOIT rester >= à l'écart angulaire entre deux colonnes gardées, sinon des secteurs
    # restent vides : décimation 1 colonne sur 4 -> atan(4/320) = 0.0125 rad au centre.
    angle_inc: float = 0.015  # rad (~0.86 deg)
    quantile: float = 0.3     # quantile des distances du secteur (robuste au bruit)
    min_points: int = 2       # points dans la bande pour valider un obstacle
    min_valid: int = 3        # mesures valides (tout z) pour déclarer « libre »

    @property
    def n_bins(self) -> int:
        return int(round((self.angle_max - self.angle_min) / self.angle_inc))


def project_to_scan(pts_scan: np.ndarray, z_base: np.ndarray, p: ProjectionParams) -> np.ndarray:
    """pts_scan : (N,3) points dans le repère du scan (horizontal, origine caméra).
    z_base : (N,) hauteur des mêmes points par rapport au sol (base_footprint).
    Retourne (n_bins,) float32 : distance, +inf (libre) ou NaN (inconnu)."""
    n_bins = p.n_bins
    x, y = pts_scan[:, 0], pts_scan[:, 1]
    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z_base)
    r = np.hypot(x, y)
    b = np.floor((np.arctan2(y, x) - p.angle_min) / p.angle_inc).astype(np.int64)
    in_fov = finite & (b >= 0) & (b < n_bins) & (r >= p.r_min)

    ranges = np.full(n_bins, np.nan, dtype=np.float32)

    # Secteurs « vus » (n'importe quelle hauteur, n'importe quelle portée) -> libres par défaut
    seen = np.bincount(b[in_fov], minlength=n_bins)
    ranges[seen >= p.min_valid] = np.inf

    band = in_fov & (r <= p.r_max) & (z_base >= p.z_min) & (z_base <= p.z_max)
    if not band.any():
        return ranges
    bb, rr = b[band], r[band]
    order = np.lexsort((rr, bb))
    bb, rr = bb[order], rr[order]
    counts = np.bincount(bb, minlength=n_bins)
    starts = np.cumsum(counts) - counts

    confirmed = counts >= p.min_points
    idx = starts[confirmed] + np.floor(p.quantile * (counts[confirmed] - 1)).astype(np.int64)
    ranges[confirmed] = rr[idx]
    # 1 seul point isolé : douteux -> ni obstacle ni « libre »
    ranges[(counts > 0) & ~confirmed] = np.nan
    return ranges
