"""Mémoire des tags vus pendant la course, dans le repère `map`.

Chaque observation est pondérée par 1/distance² : une vue de près, plus
précise, l'emporte sur les vues lointaines. Une vue nettement plus proche que
les précédentes repart de zéro, et une vue nettement plus lointaine que la
meilleure est ignorée : mesuré en simulation, les vues à 1,5 m dispersaient
la pose de 0,6 m et la normale de 36°. La normale (cap) est moyennée sur
le cercle. La mémoire peut être exportée en fin de course et rechargée (poses
mémorisées pendant les essais, seulement si le règlement l'autorise).
"""

from dataclasses import dataclass
import math

# Les vues très proches ne doivent pas écraser toutes les autres.
DISTANCE_PONDERATION_MIN = 0.3
# Vue au moins 1,5 fois plus proche que la meilleure : elle remplace tout.
RAPPORT_REMPLACEMENT = 1.0 / 1.5
# Vue plus de 1,5 fois plus lointaine que la meilleure : ignorée.
RAPPORT_REJET = 1.5


@dataclass(frozen=True)
class PoseTag:
    x: float
    y: float
    cap: float          # cap de la normale du tag (rad)
    nombre: int         # nombre d'observations fusionnées
    distance_min: float  # plus courte distance d'observation (m)


class _Accumulateur:

    def __init__(self):
        self.poids = 0.0
        self.somme_x = 0.0
        self.somme_y = 0.0
        self.somme_cos = 0.0
        self.somme_sin = 0.0
        self.nombre = 0
        self.distance_min = math.inf

    def ajouter(self, x, y, cap, distance, poids):
        self.poids += poids
        self.somme_x += poids * x
        self.somme_y += poids * y
        self.somme_cos += poids * math.cos(cap)
        self.somme_sin += poids * math.sin(cap)
        self.nombre += 1
        self.distance_min = min(self.distance_min, distance)

    def pose(self):
        return PoseTag(
            self.somme_x / self.poids, self.somme_y / self.poids,
            math.atan2(self.somme_sin, self.somme_cos), self.nombre,
            self.distance_min)


class MemoireTags:

    def __init__(self, distance_max):
        self._distance_max = float(distance_max)
        self._tags = {}

    def ajouter(self, tag_id, x, y, cap, distance):
        """Fusionne une observation ; retourne False si elle est ignorée."""
        if not math.isfinite(distance) or distance > self._distance_max:
            return False
        accumulateur = self._tags.get(int(tag_id))
        if accumulateur is not None:
            if distance > RAPPORT_REJET * accumulateur.distance_min:
                return False
            if distance < RAPPORT_REMPLACEMENT * accumulateur.distance_min:
                accumulateur = None
        if accumulateur is None:
            accumulateur = self._tags[int(tag_id)] = _Accumulateur()
        poids = 1.0 / max(distance, DISTANCE_PONDERATION_MIN) ** 2
        accumulateur.ajouter(x, y, cap, distance, poids)
        return True

    def connait(self, tag_id):
        """Faux pour None : la préparation explore sans tag cible."""
        return tag_id is not None and int(tag_id) in self._tags

    def pose(self, tag_id):
        accumulateur = self._tags.get(int(tag_id))
        return None if accumulateur is None else accumulateur.pose()

    def oublier(self, tag_id):
        self._tags.pop(int(tag_id), None)

    def ids(self):
        return sorted(self._tags)

    def exporter(self):
        return {
            'frame': 'map',
            'tags': {
                tag_id: {
                    'x': pose.x, 'y': pose.y, 'cap': pose.cap,
                    'nombre': pose.nombre, 'distance_min': pose.distance_min,
                }
                for tag_id, pose in ((i, self.pose(i)) for i in self.ids())
            },
        }

    @classmethod
    def importer(cls, donnees, distance_max):
        """Recharge une mémoire exportée : chaque tag compte pour une vue."""
        memoire = cls(distance_max)
        for tag_id, pose in (donnees.get('tags') or {}).items():
            memoire.ajouter(
                int(tag_id), float(pose['x']), float(pose['y']),
                float(pose['cap']), float(pose['distance_min']))
        return memoire
