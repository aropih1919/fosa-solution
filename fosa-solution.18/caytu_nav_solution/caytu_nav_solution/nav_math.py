"""Calculs purs de la solution Fosa (aucune dépendance ROS).

Poses 2D et angles : tout se teste avec `pytest`, sans simulateur ni ROS.

Règle de nommage : les mots du métier sont en français, les termes techniques
ROS/géométrie restent en anglais (Pose2D, yaw, quaternion).
"""

import math


def normaliser_angle(angle):
    """Ramène un angle dans [-pi, pi] (résultat d'atan2)."""
    return math.atan2(math.sin(angle), math.cos(angle))


def lacet_depuis_quaternion(x, y, z, w):
    """Lacet (rotation autour de z) d'un quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_depuis_lacet(lacet):
    """Quaternion (x, y, z, w) d'une rotation pure autour de z."""
    return 0.0, 0.0, math.sin(lacet / 2.0), math.cos(lacet / 2.0)


class Pose2D:
    """Pose plane : position (m) et cap (rad)."""

    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw

    def composer(self, autre):
        """Retourne self ∘ autre (autre exprimé dans le repère de self)."""
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        return Pose2D(
            self.x + cos_yaw * autre.x - sin_yaw * autre.y,
            self.y + sin_yaw * autre.x + cos_yaw * autre.y,
            normaliser_angle(self.yaw + autre.yaw),
        )

    def inverse(self):
        """Retourne la pose inverse : le changement de repère opposé."""
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        return Pose2D(
            -(cos_yaw * self.x + sin_yaw * self.y),
            -(-sin_yaw * self.x + cos_yaw * self.y),
            normaliser_angle(-self.yaw),
        )

    def distance_vers(self, x, y):
        """Distance en ligne droite entre cette pose et le point (x, y)."""
        return math.hypot(x - self.x, y - self.y)
