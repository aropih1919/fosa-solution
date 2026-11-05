#!/usr/bin/env python3
"""Remplace le pilote au clavier dans le simulateur 2D : suit un parcours qui
passe devant les 4 tags (vraie pose /sim/true_pose).

    python3 simulation/sim2d/conduite_auto.py            # robot:=sim
    python3 simulation/sim2d/conduite_auto.py --reel     # robot:=real (commande sur /cmd_vel)
"""
import argparse
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node

# Parcours dans le café (repère du monde), départ au spawn.
PARCOURS = [(-0.2, -6.0), (1.3, -6.0), (-1.6, -6.0), (-1.6, -8.4), (-0.2, -8.0), (-0.2, -7.5)]


class Conduite(Node):
    def __init__(self, args):
        super().__init__('conduite_auto')
        self.vitesse, self.rotation = args.vitesse, args.rotation
        self.pose = None
        self.indice = 0
        self.fini = False
        topic = '/cmd_vel' if args.reel else '/robot_base_controller/cmd_vel_unstamped'
        self.pub = self.create_publisher(Twist, topic, 10)
        self.create_subscription(PoseStamped, '/sim/true_pose', self.recevoir_pose, 10)
        self.create_timer(0.05, self.piloter)

    def recevoir_pose(self, message):
        q = message.pose.orientation
        self.pose = (message.pose.position.x, message.pose.position.y,
                     math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z))

    def piloter(self):
        if self.pose is None or self.fini:
            return
        x, y, lacet = self.pose
        cx, cy = PARCOURS[self.indice]
        if math.hypot(cx - x, cy - y) < 0.15:
            self.indice += 1
            self.get_logger().info(f'Point {self.indice}/{len(PARCOURS)} atteint')
            if self.indice >= len(PARCOURS):
                self.fini = True
                self.pub.publish(Twist())
                return
            cx, cy = PARCOURS[self.indice]
        ecart = math.atan2(cy - y, cx - x) - lacet
        ecart = math.atan2(math.sin(ecart), math.cos(ecart))
        commande = Twist()
        if abs(ecart) > 0.5:
            commande.angular.z = self.rotation * (1 if ecart > 0 else -1)
        else:
            commande.linear.x = self.vitesse
            commande.angular.z = max(-self.rotation, min(self.rotation, 1.5 * ecart))
        self.pub.publish(commande)


def main():
    lecteur = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    lecteur.add_argument('--reel', action='store_true')
    lecteur.add_argument('--vitesse', type=float, default=0.25)
    lecteur.add_argument('--rotation', type=float, default=0.6)
    args, reste = lecteur.parse_known_args()
    rclpy.init(args=reste)
    noeud = Conduite(args)
    while rclpy.ok() and not noeud.fini:
        rclpy.spin_once(noeud, timeout_sec=0.1)
    for _ in range(5):
        noeud.pub.publish(Twist())
        rclpy.spin_once(noeud, timeout_sec=0.05)
    noeud.get_logger().info('Parcours terminé')
    noeud.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
