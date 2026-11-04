#!/usr/bin/env python3
"""Moniteur d'essai en simulation : tout ce qu'affiche `./run.sh suivi`
(récit + surveillance technique), plus l'arbitre et des juges simulés.

Pour chaque visite, les juges mesurent la vraie position du robot dans Gazebo
et vérifient le règlement : centre à 0,5 m du tag (±5 cm ici) et face au tag
à ±12°. Les contacts du robot avec le décor sont signalés.

Option --quitter-a-la-fin : s'arrête au résumé final de la course.
"""

import math
import re
import subprocess
import sys

import rclpy

from caytu_course.suivi import creer_noeud_suivi, tourner
from parc_msgs.msg import Finish, StartSignal, TagVisit

# Mêmes positions que monde_tags.launch.py ; le tag est 6 mm devant le modèle.
TAGS = {7: (-2.3, -7.3, 0.0), 1: (-1.0, -9.9, math.pi / 2),
        9: (1.0, -6.2, -math.pi / 2), 4: (2.2, -8.3, math.pi)}
# Capteurs de contact du modèle Gazebo du robot (pont gz_bridge.yaml de PARC).
TOPICS_CONTACTS = ('/base_collisions', '/top_chassis_collisions')
SOLS = ('ground', 'floor', 'sol')


def juge(noeud, texte):
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


def visite(noeud, message):
    tag = message.tag_id
    juge(noeud, f'arbitre : visite du tag {tag} reçue, photo '
                f'{message.image.format or "ABSENTE"} ({len(message.image.data)} octets)')
    reelle = pose_gazebo()
    if reelle is None or tag not in TAGS:
        juge(noeud, f'VISITE tag {tag} : pose Gazebo inconnue')
        return
    x, y, cap_robot = reelle
    tx, ty, cap_tag = TAGS[tag]
    tx += 0.006 * math.cos(cap_tag)
    ty += 0.006 * math.sin(cap_tag)
    distance = math.hypot(x - tx, y - ty)
    face = cap_tag + math.pi
    erreur = math.degrees(math.atan2(math.sin(cap_robot - face), math.cos(cap_robot - face)))
    verdict = 'VALIDE' if abs(distance - 0.5) <= 0.05 and abs(erreur) <= 12 else 'HORS TOLÉRANCE'
    juge(noeud, f'VISITE tag {tag} mesurée dans Gazebo : centre à {distance:.3f} m, '
                f'erreur de cap {erreur:+.1f}° -> {verdict}')


def abonner_contacts(noeud):
    try:
        from ros_gz_interfaces.msg import Contacts
    except ImportError:
        return
    deja = set()

    def recevoir(message):
        objets = set()
        for contact in message.contacts:
            for nom in (contact.collision1.name, contact.collision2.name):
                if 'sitoe_robot' not in nom and not any(sol in nom for sol in SOLS):
                    objets.add(nom.split('::')[0])
        nouveaux = objets - deja
        if nouveaux:
            deja.update(nouveaux)
            noeud.afficher(noeud.surveillance.contact(nouveaux))

    for topic in TOPICS_CONTACTS:
        noeud.create_subscription(Contacts, topic, recevoir, 10)


def main():
    rclpy.init()
    noeud = creer_noeud_suivi('moniteur_essai', technique=True,
                              quitter_a_la_fin='--quitter-a-la-fin' in sys.argv)
    noeud.create_subscription(
        StartSignal, '/parc/start',
        lambda m: juge(noeud, f'arbitre : départ reçu (méthode {m.method}, « {m.heard} »)'), 10)
    noeud.create_subscription(TagVisit, '/parc/tag_visit', lambda m: visite(noeud, m), 10)
    noeud.create_subscription(
        Finish, '/parc/finish', lambda m: juge(noeud, f'arbitre : fin reçue (raison {m.reason})'),
        10)
    abonner_contacts(noeud)
    tourner(noeud)


if __name__ == '__main__':
    main()
