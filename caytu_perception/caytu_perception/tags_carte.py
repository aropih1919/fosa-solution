"""Position des AprilTags dans la carte : calculs purs, sans ROS.

Une observation donne le centre du tag et sa normale horizontale dans `map`.
La normale pointe du côté libre, celui d'où la caméra voit le tag : c'est de
ce côté que le robot doit se placer, à 0,5 m, face au tag.

Le registre garde les dernières observations de chaque tag et en tire une
estimation robuste (médiane des positions, moyenne circulaire des normales).
Il s'enregistre dans un fichier YAML du dossier de la carte.
"""

from dataclasses import dataclass, field
import math
from pathlib import Path

import yaml


def normaliser_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def tourner_vecteur(quaternion, vecteur):
    """Applique la rotation (x, y, z, w) au vecteur (vx, vy, vz)."""
    x, y, z, w = quaternion
    vx, vy, vz = vecteur
    # v' = v + 2w (q x v) + 2 q x (q x v)
    cx, cy, cz = y * vz - z * vy, z * vx - x * vz, x * vy - y * vx
    ccx, ccy, ccz = y * cz - z * cy, z * cx - x * cz, x * cy - y * cx
    return (vx + 2.0 * (w * cx + ccx), vy + 2.0 * (w * cy + ccy),
            vz + 2.0 * (w * cz + ccz))


def lacet_normale(quaternion_tag, position_tag, position_camera=None):
    """Cap (dans map) de la normale horizontale du tag, côté caméra.

    L'axe z du tag est sa normale, mais le sens dépend du détecteur (OpenCV :
    vers la caméra ; d'autres : vers le mur). On l'oriente vers la caméra
    quand sa position est connue. Si le tag est presque horizontal, la
    direction tag -> caméra sert de normale.
    """
    nx, ny, _ = tourner_vecteur(quaternion_tag, (0.0, 0.0, 1.0))
    if position_camera is not None:
        vers_camera = (position_camera[0] - position_tag[0],
                       position_camera[1] - position_tag[1])
        if math.hypot(nx, ny) < 0.3:
            return math.atan2(vers_camera[1], vers_camera[0])
        if nx * vers_camera[0] + ny * vers_camera[1] < 0.0:
            nx, ny = -nx, -ny
    return math.atan2(ny, nx)


@dataclass
class ObservationTag:
    identifiant: int
    x: float
    y: float
    z: float
    lacet_normale: float
    distance: float = 0.0
    horodatage: float = 0.0
    angle_vue: float = None       # angle entre la normale et la direction de la caméra


@dataclass
class EstimationTag:
    identifiant: int
    x: float
    y: float
    z: float
    lacet_normale: float
    observations: int

    def pose_visite(self, distance):
        """Pose du robot (x, y, cap) à `distance` du tag, face à lui."""
        return (self.x + distance * math.cos(self.lacet_normale),
                self.y + distance * math.sin(self.lacet_normale),
                normaliser_angle(self.lacet_normale + math.pi))


def _mediane(valeurs):
    tri = sorted(valeurs)
    milieu = len(tri) // 2
    return tri[milieu] if len(tri) % 2 else 0.5 * (tri[milieu - 1] + tri[milieu])


@dataclass
class RegistreTags:
    """Mémoire des tags vus, avec rejet des observations aberrantes."""
    identifiants_attendus: tuple = (1, 4, 7, 9)
    distance_max: float = 4.0
    taille_memoire: int = 40
    ecart_aberrant: float = 0.6
    # Vue de près : position fiable, jamais rejetée, et seule utilisée dès
    # qu'il y en a assez (une vue lointaine pendant un virage peut être fausse
    # de plusieurs dizaines de centimètres).
    distance_proche: float = 1.6
    vues_proches_suffisantes: int = 3
    angle_vue_max: float = math.radians(65.0)
    observations: dict = field(default_factory=dict)
    estimations_chargees: dict = field(default_factory=dict)

    def ajouter(self, observation):
        """Ajoute une observation ; renvoie False si elle est refusée."""
        if self.identifiants_attendus and observation.identifiant not in (
                self.identifiants_attendus):
            return False
        if not self.observation_valide(observation):
            return False
        liste = self.observations.setdefault(observation.identifiant, [])
        estimation = self.estimation(observation.identifiant)
        if (observation.distance > self.distance_proche
                and estimation is not None and estimation.observations >= 5
                and math.hypot(observation.x - estimation.x,
                               observation.y - estimation.y) > self.ecart_aberrant):
            return False
        liste.append(observation)
        if len(liste) > self.taille_memoire:
            # On garde les plus proches : leur position est la plus précise.
            liste.sort(key=lambda obs: (obs.distance, -obs.horodatage))
            del liste[self.taille_memoire:]
        return True

    def observation_valide(self, observation):
        """Identifiant attendu, portée raisonnable, tag pas vu trop de biais."""
        if self.identifiants_attendus and observation.identifiant not in (
                self.identifiants_attendus):
            return False
        # Une seule valeur NaN rendrait l'estimation NaN pour toujours.
        if not all(math.isfinite(valeur) for valeur in (
                observation.x, observation.y, observation.z,
                observation.lacet_normale, observation.distance)):
            return False
        if observation.distance > self.distance_max:
            return False
        if observation.angle_vue is not None and observation.angle_vue > self.angle_vue_max:
            return False
        return True

    def estimation(self, identifiant):
        liste = self.observations.get(identifiant) or []
        if not liste:
            return self.estimations_chargees.get(identifiant)
        proches = [obs for obs in liste if obs.distance <= self.distance_proche]
        if len(proches) >= self.vues_proches_suffisantes:
            liste = proches
        poids = [1.0 / max(obs.distance, 0.3) for obs in liste]
        somme_x = sum(p * math.cos(obs.lacet_normale) for p, obs in zip(poids, liste))
        somme_y = sum(p * math.sin(obs.lacet_normale) for p, obs in zip(poids, liste))
        return EstimationTag(
            identifiant=identifiant,
            x=_mediane([obs.x for obs in liste]),
            y=_mediane([obs.y for obs in liste]),
            z=_mediane([obs.z for obs in liste]),
            lacet_normale=math.atan2(somme_y, somme_x),
            observations=len(self.observations.get(identifiant) or []),
        )

    def estimations(self):
        identifiants = sorted(set(self.observations) | set(self.estimations_chargees))
        resultat = (self.estimation(identifiant) for identifiant in identifiants)
        return [estimation for estimation in resultat if estimation is not None]

    # ----------------------------------------------------------- fichier
    def enregistrer(self, chemin, repere='map'):
        contenu = {
            'repere': repere,
            'tags': {
                int(estimation.identifiant): {
                    'x': round(estimation.x, 4),
                    'y': round(estimation.y, 4),
                    'z': round(estimation.z, 4),
                    'lacet_normale': round(estimation.lacet_normale, 4),
                    'observations': int(estimation.observations),
                }
                for estimation in self.estimations()
            },
        }
        chemin = Path(chemin)
        temporaire = chemin.with_suffix(chemin.suffix + '.tmp')
        temporaire.write_text(
            '# Tags enregistrés par registre_tags (repère de la carte).\n'
            '# lacet_normale : direction (rad) du côté libre, face visible du tag.\n'
            + yaml.safe_dump(contenu, sort_keys=True, allow_unicode=True),
            encoding='utf-8')
        temporaire.replace(chemin)

    def charger(self, chemin):
        """Charge un fichier de tags ; renvoie le nombre de tags lus."""
        chemin = Path(chemin)
        if not chemin.is_file():
            return 0
        contenu = yaml.safe_load(chemin.read_text(encoding='utf-8')) or {}
        for identifiant, valeurs in (contenu.get('tags') or {}).items():
            self.estimations_chargees[int(identifiant)] = EstimationTag(
                identifiant=int(identifiant),
                x=float(valeurs['x']), y=float(valeurs['y']),
                z=float(valeurs.get('z', 1.0)),
                lacet_normale=float(valeurs['lacet_normale']),
                observations=int(valeurs.get('observations', 1)),
            )
        return len(self.estimations_chargees)
