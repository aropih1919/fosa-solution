#!/usr/bin/env python3
"""Arbitre de la course simulée : juge chaque visite sur la VRAIE pose du robot.

    python3 simulation/outils/verifier_course.py --sortie resultat.json
    python3 simulation/outils/verifier_course.py --verite ~/fosa_maps/X/tags_fictifs/verite.yaml

Sources :
  - vraie pose du robot : /sitoe_robot/pose (Gazebo PARC et simulateur 2D) ;
  - vraie position des tags : --verite (tags fictifs de Gazebo), sinon /sim/tags
    (simulateur 2D) ;
  - obstacles mobiles : /sim/obstacles (simulateur 2D, obstacle_gazebo.py) ;
  - annonces de la course : course.status_topic (JSON de statut_course.py).

Contrôles, comme les juges :
  - le robot ne bouge pas avant le départ ;
  - chaque visite : centre du robot à 0,5 m du tag (± --tolerance-distance),
    face au tag à ± --tolerance-angle degrés ;
  - les tags visités dans l'ordre publié ;
  - la fin annoncée avant la limite de temps ;
  - distance minimale entre le robot et la personne (aucun contact).

Écrit un résumé JSON (--sortie) et rend 0 si tout est réussi, 1 sinon.
"""
import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import yaml


def reglages():
    try:
        from caytu_config import load_config
        course = load_config().course
        return course.status_topic, course.visit_distance_m, course.time_limit_sec
    except Exception:
        return '/fosa/course/statut', 0.5, 600.0


def ecart_angle(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class Arbitre(Node):
    def __init__(self, args, topic_statut, distance_visite, limite):
        super().__init__('arbitre_course')
        self.args = args
        self.distance_visite, self.limite = distance_visite, limite
        self.pose = None
        self.pose_initiale = None
        self.deplacement_avant_depart = 0.0
        self.rotation_avant_depart = 0.0
        self.tags = None
        if args.verite:
            donnees = yaml.safe_load(open(args.verite, encoding='utf-8'))['tags']
            self.tags = {int(i): (t['monde_x'], t['monde_y'], t['monde_cap'])
                         for i, t in donnees.items()}
        self.obstacles = {}
        self.distance_personne_min = math.inf
        self.depart = None            # instant (s) du départ
        self.ordre = []
        self.visites = []
        self.fin = None
        self.annonces = set()
        self.create_subscription(Pose, '/sitoe_robot/pose', self.recevoir_pose, 50)
        self.create_subscription(String, '/sim/tags', self.recevoir_tags, 10)
        self.create_subscription(String, '/sim/obstacles', self.recevoir_obstacles, 10)
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, topic_statut, self.recevoir_statut, qos)
        self.debut = time.monotonic()

    def afficher(self, texte):
        print(texte, flush=True)

    def recevoir_pose(self, message):
        q = message.orientation
        lacet = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (message.position.x, message.position.y, lacet)
        if self.depart is None:
            if self.pose_initiale is None:
                self.pose_initiale = self.pose
            x0, y0, l0 = self.pose_initiale
            self.deplacement_avant_depart = max(
                self.deplacement_avant_depart, math.hypot(self.pose[0] - x0, self.pose[1] - y0))
            self.rotation_avant_depart = max(
                self.rotation_avant_depart, abs(math.degrees(ecart_angle(lacet, l0))))
        for x, y, rayon in self.obstacles.values():
            self.distance_personne_min = min(
                self.distance_personne_min,
                math.hypot(self.pose[0] - x, self.pose[1] - y) - rayon - self.args.rayon_robot)

    def recevoir_tags(self, message):
        if self.args.verite is None:
            self.tags = {int(i): tuple(v) for i, v in json.loads(message.data).items()}

    def recevoir_obstacles(self, message):
        self.obstacles = {nom: tuple(v) for nom, v in json.loads(message.data).items()}

    def recevoir_statut(self, message):
        donnees = json.loads(message.data)
        evenement = donnees.get('evenement')
        cle = (evenement, donnees.get('tag'), donnees.get('temps_s'))
        if cle in self.annonces:
            return
        self.annonces.add(cle)
        if evenement == 'depart':
            self.depart = time.monotonic()
            self.ordre = list(donnees.get('ordre') or [])
            self.afficher(f'DEPART : ordre {self.ordre} ; avant le départ, le robot a bougé de '
                          f'{self.deplacement_avant_depart * 100:.1f} cm et '
                          f'{self.rotation_avant_depart:.1f}°.')
        elif evenement == 'visite':
            self.juger_visite(donnees)
        elif evenement == 'saute':
            self.afficher(f'SAUTE : tag {donnees.get("tag")} à {donnees.get("temps_s", 0):.0f} s.')
        elif evenement == 'fin':
            self.fin = donnees
            self.afficher(f'FIN à {donnees.get("temps_s", 0):.0f} s : visites {donnees.get("visites")}, '
                          f'raison {donnees.get("raison")}.')

    def juger_visite(self, donnees):
        identifiant = donnees.get('tag')
        visite = {'tag': identifiant, 'temps_s': donnees.get('temps_s'),
                  'image': bool(donnees.get('image'))}
        if self.pose is None or not self.tags or identifiant not in self.tags:
            visite.update(valide=False, raison='vérité inconnue (pose ou tags)')
        else:
            x, y, cap = self.tags[identifiant]
            rx, ry, rlacet = self.pose
            distance = math.hypot(rx - x, ry - y)
            alignement = math.degrees(ecart_angle(rlacet, cap + math.pi))
            valide = (abs(distance - self.distance_visite) <= self.args.tolerance_distance
                      and abs(alignement) <= self.args.tolerance_angle)
            visite.update(distance_m=round(distance, 3), alignement_deg=round(alignement, 1),
                          valide=valide)
        self.visites.append(visite)
        self.afficher(
            f'VISITE tag {identifiant} à {visite["temps_s"]:.0f} s : '
            + (f'distance vraie {visite["distance_m"]:.3f} m, alignement '
               f'{visite["alignement_deg"]:+.1f}° ' if 'distance_m' in visite else '')
            + ('-> VALIDE' if visite['valide'] else f'-> HORS REGLE {visite.get("raison", "")}'))

    def termine(self):
        if self.fin is not None:
            return True
        if self.depart is not None:
            return time.monotonic() - self.depart > self.limite + self.args.marge
        return time.monotonic() - self.debut > self.args.attente_depart

    def resume(self):
        visites_valides = [v['tag'] for v in self.visites if v['valide']]
        ordre_respecte = visites_valides == [t for t in self.ordre if t in visites_valides]
        immobile = (self.deplacement_avant_depart <= 0.03 and self.rotation_avant_depart <= 2.0)
        sans_contact = self.distance_personne_min > 0.0
        reussi = (self.depart is not None and immobile and self.fin is not None
                  and all(v['valide'] for v in self.visites)
                  and len(visites_valides) >= self.args.visites_min
                  and ordre_respecte and sans_contact)
        return {
            'reussi': reussi,
            'depart_recu': self.depart is not None,
            'immobile_avant_depart': immobile,
            'deplacement_avant_depart_cm': round(self.deplacement_avant_depart * 100, 1),
            'ordre': self.ordre,
            'visites': self.visites,
            'tags_valides': visites_valides,
            'ordre_respecte': ordre_respecte,
            'fin': self.fin,
            'distance_min_personne_m': (None if math.isinf(self.distance_personne_min)
                                        else round(self.distance_personne_min, 2)),
        }


def main():
    topic_statut, distance_visite, limite = reglages()
    lecteur = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    lecteur.add_argument('--verite', help='verite.yaml de tools/tags_fictifs.py (Gazebo)')
    lecteur.add_argument('--sortie', help='fichier JSON du résumé')
    lecteur.add_argument('--tolerance-distance', type=float, default=0.10)
    lecteur.add_argument('--tolerance-angle', type=float, default=12.0)
    lecteur.add_argument('--visites-min', type=int, default=4,
                         help='visites valides exigées (3 si un tag est retiré)')
    lecteur.add_argument('--rayon-robot', type=float, default=0.30,
                         help='rayon du robot pour la distance à la personne (m)')
    lecteur.add_argument('--attente-depart', type=float, default=900.0,
                         help='abandon si aucun départ (s)')
    lecteur.add_argument('--marge', type=float, default=30.0, help='au-delà de la limite (s)')
    args, reste = lecteur.parse_known_args()
    rclpy.init(args=reste)
    arbitre = Arbitre(args, topic_statut, distance_visite, limite)
    try:
        while rclpy.ok() and not arbitre.termine():
            rclpy.spin_once(arbitre, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    resume = arbitre.resume()
    if arbitre.fin is None:
        arbitre.afficher('PAS DE FIN annoncée dans le temps imparti.')
    if resume['distance_min_personne_m'] is not None:
        arbitre.afficher(f'Distance minimale robot - personne : {resume["distance_min_personne_m"]:.2f} m '
                         f'(rayon robot {args.rayon_robot} m).')
    arbitre.afficher(f'RESULTAT : {"REUSSI" if resume["reussi"] else "ECHEC"}')
    if args.sortie:
        with open(args.sortie, 'w', encoding='utf-8') as fichier:
            json.dump(resume, fichier, ensure_ascii=False, indent=2)
    arbitre.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if resume['reussi'] else 1)


if __name__ == '__main__':
    main()
