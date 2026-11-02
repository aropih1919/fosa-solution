"""Résolution déterministe des noms, alias et ancres AprilTag."""

from dataclasses import dataclass
import math
import unicodedata


def normaliser(texte):
    decompose = unicodedata.normalize('NFKD', str(texte))
    sans_accents = ''.join(
        caractere for caractere in decompose
        if not unicodedata.combining(caractere))
    return ' '.join(
        sans_accents.casefold().replace('_', ' ').replace('-', ' ').split())


@dataclass(frozen=True)
class LieuRegistre:
    name: str
    aliases: tuple
    tag_ids: tuple
    description: str
    category: str
    approach_pose: object
    approach_offset_m: float


class RegistreLieux:
    """Index immuable nom/alias/tag construit depuis `caytu_config`."""

    def __init__(self, semantic_map):
        self.world = semantic_map.world
        self.frame = semantic_map.frame
        self._by_name = {}
        self._by_tag = {}
        self._by_alias = {}
        for location in semantic_map.locations:
            record = LieuRegistre(
                name=location.name,
                aliases=tuple(location.aliases),
                tag_ids=tuple(location.tag_ids),
                description=location.description,
                category=location.category,
                approach_pose=location.approach_pose,
                approach_offset_m=location.approach_offset_m,
            )
            self._by_name[normaliser(record.name)] = record
            for alias in record.aliases:
                self._by_alias[normaliser(alias)] = record
            for tag_id in record.tag_ids:
                self._by_tag[tag_id] = record

    def resoudre(self, requete):
        key = normaliser(requete)
        return self._by_name.get(key) or self._by_alias.get(key)

    def par_tag(self, tag_id):
        return self._by_tag.get(int(tag_id))

    def tous(self):
        return tuple(self._by_name.values())


def pose_approche_depuis_tag(pose_tag, offset):
    """Place le robot à distance du tag et orienté vers celui-ci."""
    yaw_tag = quaternion_yaw(pose_tag.orientation.x, pose_tag.orientation.y,
                             pose_tag.orientation.z, pose_tag.orientation.w)
    x = pose_tag.position.x - offset * math.cos(yaw_tag)
    y = pose_tag.position.y - offset * math.sin(yaw_tag)
    yaw = yaw_tag + math.pi
    return x, y, yaw


def quaternion_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))
