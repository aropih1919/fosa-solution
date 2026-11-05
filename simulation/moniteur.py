#!/usr/bin/env python3
"""Moniteur d'essai en simulation : tout ce qu'affiche `./run.sh suivi`
(récit + surveillance technique), plus l'arbitre et des juges simulés.

La vérité terrain vient du JSON écrit par le générateur d'arène officiel
(`--verite`, ou FOSA_SIM_VERITE). La vraie pose du robot est lue sur
`/sim/pose` (topic réservé aux tests) : pas de requête Gazebo, donc pas de
ralentissement. Pour chaque visite, les juges vérifient le règlement : centre
à 0,5 m du tag (±5 cm ici) et face au tag à ±12°. Les contacts avec le décor
sont signalés.

Option --quitter-a-la-fin : s'arrête au résumé final de la course.
"""

import argparse
import json
import math
import os
import sys

import rclpy
from geometry_msgs.msg import Pose

from caytu_course.suivi import creer_noeud_suivi, tourner
from parc_msgs.msg import Finish, RunStatus, StartSignal, TagVisit

# Capteurs de contact du modèle Gazebo du robot (pont gz_bridge.yaml de PARC).
TOPICS_CONTACTS = ('/sim/base_collisions', '/sim/top_chassis_collisions',
                   '/sim/left_wheel_collisions', '/sim/right_wheel_collisions')
SOLS = ('ground', 'floor', 'sol')
# Le visuel du tag est 6,6 mm devant l'origine du modèle.
FACE_TAG_M = 0.0066


def juge(noeud, texte):
    print(f'    [juge simulé] {texte}', flush=True)


def charger_verite(chemin):
    """{id: (x, y, cap)} depuis le JSON du générateur d'arène."""
    with open(os.path.expanduser(chemin), encoding='utf-8') as fichier:
        monde = json.load(fichier)
    return {int(tag['id']): (float(tag['x']), float(tag['y']),
                             float(tag['facing_yaw']))
            for tag in monde['tags']}, monde


class Moniteur:
    def __init__(self, noeud, tags):
        self._noeud = noeud
        self._tags = tags
        self._pose = None
        self._dernier_etat = None

    def recevoir_pose(self, message):
        self._pose = message

    def recevoir_statut(self, message):
        etat = getattr(message, 'state', None)
        nom = None
        for candidat in ('IDLE', 'ARMED', 'RUNNING', 'FINISHED', 'TIMED_OUT', 'TERMINATED'):
            if etat == getattr(type(message), candidat, None):
                nom = candidat
                break
        if nom is None or nom == self._dernier_etat:
            return
        self._dernier_etat = nom
        detail = ''
        if hasattr(message, 'elapsed_s'):
            detail = f' ({message.elapsed_s:.0f} s)'
        if getattr(message, 'false_start', False):
            detail += ' — FAUX DÉPART'
        juge(self._noeud, f'arbitre : état {nom}{detail}')

    def recevoir_visite(self, message):
        tag = int(message.tag_id)
        image = getattr(message, 'image', None)
        format_image = getattr(image, 'format', '') if image is not None else ''
        octets = len(image.data) if image is not None else 0
        juge(self._noeud, f'arbitre : visite du tag {tag} reçue, photo '
                          f'{format_image or "ABSENTE"} ({octets} octets)')
        if self._pose is None or tag not in self._tags:
            juge(self._noeud, f'VISITE tag {tag} : pose Gazebo inconnue')
            return
        x, y = self._pose.position.x, self._pose.position.y
        q = self._pose.orientation
        cap_robot = math.atan2(2 * (q.w * q.z + q.x * q.y),
                               1 - 2 * (q.y * q.y + q.z * q.z))
        tx, ty, cap_tag = self._tags[tag]
        tx += FACE_TAG_M * math.cos(cap_tag)
        ty += FACE_TAG_M * math.sin(cap_tag)
        distance = math.hypot(x - tx, y - ty)
        face = cap_tag + math.pi
        erreur = math.degrees(math.atan2(math.sin(cap_robot - face),
                                         math.cos(cap_robot - face)))
        verdict = ('VALIDE' if abs(distance - 0.5) <= 0.05 and abs(erreur) <= 12
                   else 'HORS TOLÉRANCE')
        juge(self._noeud, f'VISITE tag {tag} mesurée dans Gazebo : centre à '
                          f'{distance:.3f} m, erreur de cap {erreur:+.1f}° -> {verdict}')


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
    analyseur = argparse.ArgumentParser(
        description='Récit de course et mesures des visites dans le simulateur PARC.')
    analyseur.add_argument(
        '--verite', default=os.environ.get('FOSA_SIM_VERITE', ''),
        help='JSON de vérité terrain produit pour la graine simulée')
    analyseur.add_argument(
        '--quitter-a-la-fin', action='store_true',
        help='arrêter le moniteur après le résumé final de la course')
    arguments = analyseur.parse_args()
    chemin = arguments.verite
    if not chemin:
        sys.exit('Moniteur : indiquer la vérité terrain (--verite <json>) '
                 'ou FOSA_SIM_VERITE.')
    tags, monde = charger_verite(chemin)
    print(f'Vérité terrain : graine {monde.get("seed")}, tags '
          f'{", ".join(str(i) for i in sorted(tags))}.', flush=True)

    rclpy.init()
    noeud = creer_noeud_suivi('moniteur_essai', technique=True,
                              quitter_a_la_fin=arguments.quitter_a_la_fin)
    moniteur = Moniteur(noeud, tags)
    noeud.create_subscription(Pose, '/sim/pose', moniteur.recevoir_pose, 10)
    noeud.create_subscription(
        RunStatus, '/parc/run_status', moniteur.recevoir_statut, 10)
    noeud.create_subscription(
        StartSignal, '/parc/start',
        lambda m: juge(noeud, f'arbitre : départ reçu (méthode {m.method}, « {m.heard} »)'), 10)
    noeud.create_subscription(TagVisit, '/parc/tag_visit', moniteur.recevoir_visite, 10)
    noeud.create_subscription(
        Finish, '/parc/finish',
        lambda m: juge(noeud, f'arbitre : fin reçue (raison {m.reason})'), 10)
    abonner_contacts(noeud)
    tourner(noeud)


if __name__ == '__main__':
    main()
