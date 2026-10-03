"""Filtrage geometrique pur (numpy only) : isole les points a hauteur d'obstacle."""

import numpy as np


def filter_obstacle_points(points_xyz, z_min=0.15, z_max=1.2, max_range=3.5):
    """Garde les points finis dont z est dans [z_min, z_max] et la distance
    horizontale <= max_range. points_xyz : tableau (N, 3) dans le repere sol
    (z = hauteur au-dessus du sol). Retour (kept_float32, stats)."""
    pts = np.asarray(points_xyz, dtype=np.float64).reshape(-1, 3)
    total = int(pts.shape[0])
    pts = pts[np.isfinite(pts).all(axis=1)]
    if pts.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float32), {'total': total, 'kept': 0}
    keep = (
        (pts[:, 2] >= z_min) & (pts[:, 2] <= z_max)
        & (np.hypot(pts[:, 0], pts[:, 1]) <= max_range)
    )
    kept = pts[keep].astype(np.float32)
    return kept, {'total': total, 'kept': int(kept.shape[0])}
