#!/usr/bin/env python3
"""Moniteur d'essai : récit de la course, arbitre et juges simulés.

Affiche, dans le terminal, ce que fait le robot (mêmes phrases que
`./run.sh suivi`) et chaque message envoyé à l'arbitre. Pour chaque visite,
mesure la vraie position du robot dans Gazebo et vérifie le règlement :
centre à 0,5 m du tag (±5 cm ici) et face au tag à ±12°.

Option --quitter-a-la-fin : s'arrête au résumé final de la course.
"""

import json
import math
import re
import subprocess
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String

from caytu_config import load_config
from caytu_course.statut import qos_evenements
from caytu_course.suivi import formater
from parc_msgs.msg import Finish, StartSignal, TagVisit

# Mêmes positions que monde_tags.launch.py ; le tag est 6 mm devant le modèle.
TAGS = {7: (-2.3, -7.3, 0.0), 1: (-1.0, -9.9, math.pi / 2),
        9: (1.0, -6.2, -math.pi / 2), 4: (2.2, -8.3, math.pi)}


def journal(texte):
    print(f'    [juge simulé] {texte}', flush=True)


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
    journal(f'arbitre : visite du tag {tag} reçue, photo {message.image.format or "ABSENTE"} '
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
    journal(f'VISITE tag {tag} mesurée dans Gazebo : centre à {distance:.3f} m, '
            f'erreur de cap {erreur:+.1f}° -> {verdict}')


def main():
    quitter_a_la_fin = '--quitter-a-la-fin' in sys.argv
    rclpy.init()
    noeud = rclpy.create_node('moniteur_essai')
    fini = []

    def recit(message):
        try:
            evenement = json.loads(message.data)
        except ValueError:
            return
        for ligne in formater(evenement, couleurs=sys.stdout.isatty()):
            print(ligne, flush=True)
        if evenement.get('type') == 'fin':
            fini.append(True)

    noeud.create_subscription(
        String, load_config().course.events_topic, recit, qos_evenements())
    noeud.create_subscription(
        StartSignal, '/parc/start',
        lambda m: journal(f'arbitre : départ reçu (méthode {m.method}, « {m.heard} »)'), 10)
    noeud.create_subscription(TagVisit, '/parc/tag_visit', visite, 10)
    noeud.create_subscription(
        Finish, '/parc/finish', lambda m: journal(f'arbitre : fin reçue (raison {m.reason})'), 10)
    try:
        while rclpy.ok() and not (quitter_a_la_fin and fini):
            rclpy.spin_once(noeud, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        noeud.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
