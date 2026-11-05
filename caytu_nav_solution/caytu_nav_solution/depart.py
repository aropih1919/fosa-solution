#!/usr/bin/env python3
"""Départ de la course par commande : `ros2 run caytu_nav_solution depart`.

Publie std_msgs/Empty sur course.start_topic (caytu_config), comme le fait
le démarrage vocal, et le répète jusqu'à ce que course_tags accuse réception
sur course.start_ack_topic (message verrouillé). Sur un PC très chargé (le
simulateur officiel sans carte graphique), un signal envoyé une seule fois
peut se perdre pendant que les deux nœuds se découvrent.
"""

import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String

from caytu_config import load_config

DELAI_MAX = 60.0        # secondes réelles


def main(args=None):
    rclpy.init(args=args)
    course = load_config().course
    noeud = rclpy.create_node('depart_course')
    publicateur = noeud.create_publisher(Empty, course.start_topic, 10)
    accuse = []
    verrouille = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
    noeud.create_subscription(String, course.start_ack_topic,
                              lambda message: accuse.append(message.data), verrouille)
    debut = time.monotonic()
    while not accuse and time.monotonic() - debut < DELAI_MAX:
        publicateur.publish(Empty())
        fin = time.monotonic() + 0.3
        while not accuse and time.monotonic() < fin:
            rclpy.spin_once(noeud, timeout_sec=0.05)
    if accuse:
        print(f'Départ reçu par la course ({accuse[0]}) après '
              f'{time.monotonic() - debut:.1f} s.')
        code = 0
    else:
        print(f'ÉCHEC : aucun accusé sur {course.start_ack_topic} en {DELAI_MAX:.0f} s. '
              f'course_tags est-il lancé, dans le même ROS_DOMAIN_ID ? '
              f'({publicateur.get_subscription_count()} abonné(s) à {course.start_topic})')
        code = 1
    noeud.destroy_node()
    rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
