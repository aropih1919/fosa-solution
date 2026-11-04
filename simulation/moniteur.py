#!/usr/bin/env python3
"""Moniteur d'essai : joue le rôle de l'arbitre et des juges en simulation.

Affiche l'état de la course et chaque message envoyé à l'arbitre. Pour chaque
visite, mesure la vraie position du robot dans Gazebo et vérifie le
règlement : centre à 0,5 m du tag (±5 cm ici) et face au tag à ±12°.
"""

import math
import re
import subprocess
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from parc_msgs.msg import Finish, StartSignal, TagVisit

# Mêmes positions que monde_tags.launch.py ; le tag est 6 mm devant le modèle.
TAGS = {7: (-2.3, -7.3, 0.0), 1: (-1.0, -9.9, math.pi / 2),
        9: (1.0, -6.2, -math.pi / 2), 4: (2.2, -8.3, math.pi)}
DEBUT = time.monotonic()


def journal(texte):
    print(f'[{time.monotonic() - DEBUT:6.1f}s] {texte}', flush=True)


def pose_gazebo(modele='sitoe_robot'):
    """(x, y, cap) du modèle dans le monde Gazebo, ou None."""
    try:
        sortie = subprocess.run(
            ['gz', 'topic', '-e', '-t', '/world/task/pose/info', '-n', '1'],
            capture_output=True, text=True, timeout=10).stdout
    except (OSError, subprocess.TimeoutExpired):
        return None
    bloc = re.search(r'name: "' + modele + r'".*?position \{(.*?)\}.*?orientation \{(.*?)\}',
                     sortie, re.S)
    if bloc is None:
        return None

    def valeurs(texte):
        return {cle: float(val) for cle, val in re.findall(r'(\w): (-?[\d.e+-]+)', texte)}
    p, q = valeurs(bloc.group(1)), valeurs(bloc.group(2))
    cap = math.atan2(2 * (q.get('w', 1) * q.get('z', 0) + q.get('x', 0) * q.get('y', 0)),
                     1 - 2 * (q.get('y', 0) ** 2 + q.get('z', 0) ** 2))
    return p.get('x', 0.0), p.get('y', 0.0), cap


def visite(message):
    tag = message.tag_id
    journal(f'ARBITRE visite tag {tag} : image {message.image.format or "absente"} '
            f'({len(message.image.data)} octets)')
    reelle = pose_gazebo()
    if reelle is None or tag not in TAGS:
        journal(f'VISITE tag {tag} : pose Gazebo inconnue')
        return
    x, y, cap_robot = reelle
    tx, ty, cap_tag = TAGS[tag]
    tx += 0.006 * math.cos(cap_tag)
    ty += 0.006 * math.sin(cap_tag)
    distance = math.hypot(x - tx, y - ty)
    face = cap_tag + math.pi
    erreur = math.degrees(math.atan2(math.sin(cap_robot - face), math.cos(cap_robot - face)))
    verdict = 'VALIDE' if abs(distance - 0.5) <= 0.05 and abs(erreur) <= 12 else 'HORS TOLÉRANCE'
    journal(f'VISITE tag {tag} : distance réelle {distance:.3f} m, erreur de cap '
            f'{erreur:+.1f}° -> {verdict}')


def main():
    rclpy.init()
    noeud = rclpy.create_node('moniteur_essai')
    etat = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)
    noeud.create_subscription(String, '/fosa/course/etat',
                              lambda m: journal(f'état {m.data}'), etat)
    noeud.create_subscription(
        StartSignal, '/parc/start',
        lambda m: journal(f'ARBITRE départ méthode {m.method} « {m.heard} »'), 10)
    noeud.create_subscription(TagVisit, '/parc/tag_visit', visite, 10)
    noeud.create_subscription(
        Finish, '/parc/finish', lambda m: journal(f'ARBITRE fin raison {m.reason}'), 10)
    try:
        rclpy.spin(noeud)
    except KeyboardInterrupt:
        pass
    finally:
        noeud.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
