"""Politique d'inflation adaptative pure (sans ROS) : ouverte si vite+degage, prudente si lent+encombre."""


def target_inflation(speed, min_range, open_r=0.20, mid_r=0.30, closed_r=0.35):
    """speed : |vx| odom (m/s). min_range : obstacle le plus proche (m, inf si libre)."""
    cluttered = min_range < 1.0
    free = min_range > 2.0
    if cluttered or speed < 0.1:
        return closed_r
    if free and speed > 0.3:
        return open_r
    return mid_r
