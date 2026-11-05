#!/usr/bin/env python3
"""Imite les organisateurs : publie l'ordre des tags au départ de la course.

    python3 simulation/outils/ordre_organisateurs.py 7,1,9,4                  # std_msgs/String
    python3 simulation/outils/ordre_organisateurs.py 7,1,9,4 --type entiers   # Int32MultiArray
    python3 simulation/outils/ordre_organisateurs.py 7,1,9,4 --type octets    # UInt8MultiArray
    python3 simulation/outils/ordre_organisateurs.py aleatoire                # ordre tiré au sort
    python3 simulation/outils/ordre_organisateurs.py 7,1,9,4 --retard 20      # publié 20 s plus tard

Le message est « latched » (TRANSIENT_LOCAL, comme un topic publié une fois au
départ) et répété chaque seconde, pour les abonnés qui arrivent en retard.
Topic par défaut : course.order_topic de caytu_config.
"""
import argparse
import random
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32MultiArray, String, UInt8MultiArray


def topic_par_defaut():
    try:
        from caytu_config import load_config
        configuration = load_config().course
        return configuration.order_topic, list(configuration.expected_ids)
    except Exception:
        return '/tag_order', [1, 4, 7, 9]


def main():
    topic, identifiants = topic_par_defaut()
    lecteur = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    lecteur.add_argument('ordre', help='« 7,1,9,4 » ou « aleatoire »')
    lecteur.add_argument('--type', choices=['texte', 'entiers', 'octets'], default='texte')
    lecteur.add_argument('--topic', default=topic)
    lecteur.add_argument('--retard', type=float, default=0.0, help='secondes avant publication')
    args, reste = lecteur.parse_known_args()
    if args.ordre == 'aleatoire':
        ordre = random.sample(identifiants, len(identifiants))
    else:
        ordre = [int(v) for v in args.ordre.replace(' ', '').split(',') if v]

    rclpy.init(args=reste)
    noeud = rclpy.create_node('ordre_organisateurs')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    if args.type == 'texte':
        publicateur = noeud.create_publisher(String, args.topic, qos)
        message = String(data=','.join(map(str, ordre)))
    elif args.type == 'entiers':
        publicateur = noeud.create_publisher(Int32MultiArray, args.topic, qos)
        message = Int32MultiArray(data=ordre)
    else:
        publicateur = noeud.create_publisher(UInt8MultiArray, args.topic, qos)
        message = UInt8MultiArray(data=bytes(ordre))
    if args.retard > 0:
        noeud.get_logger().info(f'Ordre publié dans {args.retard:.0f} s.')
        time.sleep(args.retard)
    noeud.get_logger().info(f'ORDRE {ordre} publié sur {args.topic} ({type(message).__name__}).')
    try:
        while rclpy.ok():
            publicateur.publish(message)
            rclpy.spin_once(noeud, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        noeud.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
