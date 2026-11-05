#!/usr/bin/env python3
"""Une « personne » qui marche dans le monde Gazebo, pour tester le garde-fou.

    python3 simulation/outils/obstacle_gazebo.py ~/fosa_maps/cafe_test
    python3 simulation/outils/obstacle_gazebo.py ~/fosa_maps/cafe_test --de 1.0,-1.5 --a 1.0,1.5
    python3 simulation/outils/obstacle_gazebo.py ~/fosa_maps/cafe_test --retirer

Un cylindre (rayon 0,25 m, 1,70 m de haut) fait des allers-retours entre deux
points du repère `map` (par défaut : un segment de 3 m qui coupe le chemin
1,5 m devant la position de départ). Il s'arrête si le robot est sur son
chemin (il ne bouscule jamais le robot), puis fait demi-tour au bout de 3 s,
comme un piéton qui cède le passage. Choisir --de / --a loin des tags : une
personne arrêtée devant un tag le cache.

Publie sa position (repère du monde) sur /sim/obstacles, pour
simulation/outils/verifier_course.py. Ctrl-C : la personne est retirée.
Il faut la commande `gz` et une carte faite dans Gazebo (world_from_map).
"""
import argparse
import json
import math
from pathlib import Path
import sys
import tempfile
import time

import rclpy
from geometry_msgs.msg import Pose
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools'))
from tags_fictifs import Carte, composer, gz, trouver_monde  # noqa: E402

NOM = 'fosa_personne'
RAYON, HAUTEUR = 0.25, 1.70


def sdf_personne():
    materiau = ('<material><ambient>0.8 0.45 0.2 1</ambient><diffuse>0.8 0.45 0.2 1</diffuse>'
                '<specular>0.1 0.1 0.1 1</specular></material>')
    geometrie = f'<geometry><cylinder><radius>{RAYON}</radius><length>{HAUTEUR}</length></cylinder></geometry>'
    return ('<?xml version="1.0"?><sdf version="1.9">'
            f'<model name="{NOM}"><static>true</static><link name="corps">'
            f'<pose>0 0 {HAUTEUR / 2} 0 0 0</pose>'
            f'<visual name="v">{geometrie}{materiau}</visual>'
            f'<collision name="c">{geometrie}</collision>'
            '</link></model></sdf>')


def poser(monde, x, y):
    sortie = gz('service', '-s', f'/world/{monde}/set_pose', '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean', '--timeout', '1000',
                '--req', f'name: "{NOM}" position: {{x: {x:.3f}, y: {y:.3f}, z: 0}}', delai=3)
    return 'data: true' in sortie.stdout


def retirer(monde):
    gz('service', '-s', f'/world/{monde}/remove', '--reqtype', 'gz.msgs.Entity',
       '--reptype', 'gz.msgs.Boolean', '--timeout', '3000',
       '--req', f'name: "{NOM}" type: MODEL')


def point(texte):
    x, y = (float(v) for v in texte.split(','))
    return x, y


def main():
    lecteur = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    lecteur.add_argument('carte', help='dossier de la carte (fosa_map.yaml avec world_from_map)')
    lecteur.add_argument('--de', type=point, help='x,y de départ de la personne (repère map)')
    lecteur.add_argument('--a', type=point, help='x,y d\'arrivée (repère map)')
    lecteur.add_argument('--vitesse', type=float, default=0.4, help='m/s')
    lecteur.add_argument('--monde', help='nom du monde Gazebo')
    lecteur.add_argument('--retirer', action='store_true')
    args, reste = lecteur.parse_known_args()

    monde = trouver_monde(args.monde)
    if args.retirer:
        retirer(monde)
        print('Personne retirée.')
        return
    carte = Carte(args.carte)
    if carte.monde_depuis_carte is None:
        sys.exit('fosa_map.yaml sans world_from_map : carte faite hors de Gazebo.')
    if args.de is None or args.a is None:
        x0, y0, cap = carte.depart
        cx, cy = x0 + 1.5 * math.cos(cap), y0 + 1.5 * math.sin(cap)
        px, py = -math.sin(cap), math.cos(cap)
        args.de = args.de or (cx - 1.5 * px, cy - 1.5 * py)
        args.a = args.a or (cx + 1.5 * px, cy + 1.5 * py)
    de = composer(carte.monde_depuis_carte, (*args.de, 0.0))[:2]
    a = composer(carte.monde_depuis_carte, (*args.a, 0.0))[:2]

    retirer(monde)
    with tempfile.NamedTemporaryFile('w', suffix='.sdf', delete=False) as fichier:
        fichier.write(sdf_personne())
    requete = (f'sdf_filename: "{fichier.name}" name: "{NOM}" '
               f'pose: {{position: {{x: {de[0]:.3f}, y: {de[1]:.3f}, z: 0}}}}')
    sortie = gz('service', '-s', f'/world/{monde}/create', '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', requete)
    if 'data: true' not in sortie.stdout:
        sys.exit(f'Gazebo a refusé la personne (monde « {monde} »).')
    print(f'Personne posée : allers-retours entre {args.de} et {args.a} (map), '
          f'soit ({de[0]:.2f}, {de[1]:.2f}) et ({a[0]:.2f}, {a[1]:.2f}) dans le monde.', flush=True)

    rclpy.init(args=reste)
    noeud = rclpy.create_node('obstacle_gazebo')
    pub = noeud.create_publisher(String, '/sim/obstacles', 10)
    robot = {}

    def recevoir_pose(message):
        robot['xy'] = (message.position.x, message.position.y)
    noeud.create_subscription(Pose, '/sitoe_robot/pose', recevoir_pose, 10)

    x, y = de
    cible = a
    bloquee = 0.0
    dernier = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(noeud, timeout_sec=0.05)
            maintenant = time.monotonic()
            dt, dernier = maintenant - dernier, maintenant
            dx, dy = cible[0] - x, cible[1] - y
            distance = math.hypot(dx, dy)
            pas = args.vitesse * dt
            if distance <= pas:
                x, y = cible
                cible = de if cible == a else a
                time.sleep(1.0)
                dernier = time.monotonic()
            else:
                nx, ny = x + dx / distance * pas, y + dy / distance * pas
                # Le robot est sur le chemin : la personne attend, puis fait demi-tour.
                if 'xy' not in robot or math.hypot(nx - robot['xy'][0], ny - robot['xy'][1]) > RAYON + 0.45:
                    x, y, bloquee = nx, ny, 0.0
                else:
                    bloquee += dt
                    if bloquee > 3.0:
                        cible, bloquee = (de if cible == a else a), 0.0
            poser(monde, x, y)
            if not rclpy.ok():
                break
            pub.publish(String(data=json.dumps({'personne': [x, y, RAYON]})))
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        retirer(monde)
        Path(fichier.name).unlink(missing_ok=True)
        noeud.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
