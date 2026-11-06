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
import time
from pathlib import Path
import urllib.request

import rclpy
from geometry_msgs.msg import Pose

from caytu_course.suivi import creer_noeud_suivi, tourner
from caytu_course.statut import qos_statut
from caytu_config.loader import config_directory
from parc_msgs.msg import Finish, RunStatus, StartSignal, TagVisit

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
    def __init__(self, noeud, tags, regles, pret):
        self._noeud = noeud
        self._tags = tags
        self._pose = None
        self._instant_pose = None
        self._regles = regles
        self._pret = pret
        self._dernier_etat = None
        self.visites = []
        self.contacts = set()
        self.ordre = []
        self.faux_depart = False
        self.capteurs_contacts = set()

    def recevoir_pose(self, message):
        self._pose = message
        self._instant_pose = time.monotonic()
        if self._pret:
            self._pret.touch()
            self._pret = None

    def recevoir_statut(self, message):
        self.ordre = list(message.tag_order)
        self.faux_depart = self.faux_depart or message.false_start
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
        mesure = {'tag': tag, 'jpeg_octets': octets, 'valide': False}
        self.visites.append(mesure)
        juge(self._noeud, f'arbitre : visite du tag {tag} reçue, photo '
                          f'{format_image or "ABSENTE"} ({octets} octets)')
        regles = self._regles
        if (self._pose is None or tag not in self._tags
                or time.monotonic() - self._instant_pose > regles['age_pose_max_sec']):
            juge(self._noeud, f'VISITE tag {tag} : pose Gazebo inconnue')
            return
        x, y = self._pose.position.x, self._pose.position.y
        q = self._pose.orientation
        cap_robot = math.atan2(2 * (q.w * q.z + q.x * q.y),
                               1 - 2 * (q.y * q.y + q.z * q.z))
        tx, ty, cap_tag = self._tags[tag]
        tx += regles['face_tag_m'] * math.cos(cap_tag)
        ty += regles['face_tag_m'] * math.sin(cap_tag)
        distance = math.hypot(x - tx, y - ty)
        face = cap_tag + math.pi
        erreur = math.degrees(math.atan2(math.sin(cap_robot - face),
                                         math.cos(cap_robot - face)))
        verdict = ('VALIDE' if abs(distance - regles['distance_m']) <= regles['tolerance_distance_m']
                   and abs(erreur) <= regles['tolerance_cap_deg']
                   else 'HORS TOLÉRANCE')
        mesure.update(distance_m=distance, erreur_cap_deg=erreur,
                      valide=verdict == 'VALIDE' and octets > 0)
        juge(self._noeud, f'VISITE tag {tag} mesurée dans Gazebo : centre à '
                          f'{distance:.3f} m, erreur de cap {erreur:+.1f}° -> {verdict}')


def abonner_contacts(noeud, moniteur, regles):
    try:
        from ros_gz_interfaces.msg import Contacts
    except ImportError:
        return
    deja = set()

    def recevoir(message):
        objets = set()
        for contact in message.contacts:
            for nom in (contact.collision1.name, contact.collision2.name):
                if regles['modele_robot'] not in nom and not any(sol in nom for sol in regles['noms_sol']):
                    objets.add(nom.split('::')[0])
        nouveaux = objets - deja
        moniteur.contacts.update(objets)
        if nouveaux:
            deja.update(nouveaux)
            noeud.afficher(noeud.surveillance.contact(nouveaux))

    for topic in regles['topics_contacts']:
        noeud.create_subscription(Contacts, topic, recevoir, 10)
    def verifier_capteurs():
        moniteur.capteurs_contacts.update(
            topic for topic in regles['topics_contacts'] if noeud.count_publishers(topic))
    noeud.create_timer(1.0, verifier_capteurs)


def main():
    analyseur = argparse.ArgumentParser(
        description='Récit de course et mesures des visites dans le simulateur PARC.')
    analyseur.add_argument(
        '--verite', default=os.environ.get('FOSA_SIM_VERITE', ''),
        help='JSON de vérité terrain produit pour la graine simulée')
    analyseur.add_argument(
        '--quitter-a-la-fin', action='store_true',
        help='arrêter le moniteur après le résumé final de la course')
    analyseur.add_argument('--rapport', type=Path)
    analyseur.add_argument('--pret', type=Path)
    analyseur.add_argument('--exiger-visites', action='store_true')
    regles = json.loads((config_directory() / 'exploitation.json').read_text())
    analyseur.add_argument('--port-arbitre', type=int,
                          default=regles['simulation']['port_arbitre'])
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
    jugement = regles['simulation']['jugement']
    moniteur = Moniteur(noeud, tags, jugement, arguments.pret)
    noeud.create_subscription(Pose, jugement['topic_pose'], moniteur.recevoir_pose, 10)
    noeud.create_subscription(
        RunStatus, '/parc/run_status', moniteur.recevoir_statut, qos_statut(1))
    noeud.create_subscription(
        StartSignal, '/parc/start',
        lambda m: juge(noeud, f'arbitre : départ reçu (méthode {m.method}, « {m.heard} »)'), qos_statut())
    noeud.create_subscription(TagVisit, '/parc/tag_visit', moniteur.recevoir_visite, qos_statut())
    noeud.create_subscription(
        Finish, '/parc/finish',
        lambda m: juge(noeud, f'arbitre : fin reçue (raison {m.reason})'), 10)
    abonner_contacts(noeud, moniteur, jugement)
    tourner(noeud)
    rapport = {'ordre': moniteur.ordre, 'visites': moniteur.visites,
                'contacts': sorted(moniteur.contacts), 'faux_depart': moniteur.faux_depart,
                'capteurs_contacts': sorted(moniteur.capteurs_contacts)}
    reussi = noeud.fini
    if arguments.exiger_visites:
        try:
            with urllib.request.urlopen(
                    f'{jugement["url_arbitre"]}:{arguments.port_arbitre}/api/state',
                    timeout=jugement['delai_http_sec']) as flux:
                officiel = json.load(flux)
            rapport['arbitre'] = officiel
            visites = officiel['visits']
            reussi = (reussi and len(visites) == len(moniteur.ordre) == 4
                      and all(v['hold'] == 'ok' and v['image_check'] == 'match'
                              and v['order_status'] == 'in_order' for v in visites))
        except (OSError, ValueError, KeyError) as erreur:
            rapport['erreur_arbitre'] = str(erreur)
            reussi = False
        reussi = (reussi and not moniteur.contacts and not moniteur.faux_depart
                  and moniteur.capteurs_contacts == set(jugement['topics_contacts'])
                  and [v['tag'] for v in moniteur.visites] == moniteur.ordre
                  and all(v['valide'] for v in moniteur.visites))
    rapport['reussi'] = reussi
    if arguments.rapport:
        arguments.rapport.parent.mkdir(parents=True, exist_ok=True)
        arguments.rapport.write_text(json.dumps(rapport, ensure_ascii=False, indent=2))
    return 0 if reussi else 1


if __name__ == '__main__':
    sys.exit(main())
