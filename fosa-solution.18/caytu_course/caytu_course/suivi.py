#!/usr/bin/env python3
"""Affiche en direct ce que fait le robot (`./run.sh suivi`).

Deux niveaux dans le même terminal :
  - le RÉCIT de la course, en phrases (tags repérés, plans, sauts expliqués,
    échecs, visites, résumé final), avec le temps écoulé et restant ;
  - la SURVEILLANCE technique (`surveillance.py`) : trajets Nav2, chemins,
    rotations, arrêts, robot bloqué, et une ligne d'état périodique.

Options : --sans-technique (récit seul), --quitter-a-la-fin.
L'historique du récit est gardé : lancé en retard, on reçoit tout.
"""

import json
import math
import sys

COULEURS = {
    'visite': '\033[1;32m', 'saut': '\033[1;33m', 'echec': '\033[33m',
    'perdu': '\033[33m', 'fin': '\033[1;36m', 'plan': '\033[36m',
    'tag_repere': '\033[35m', 'depart': '\033[1m', 'ordre': '\033[1m',
    'degagement': '\033[33m', 'voix': '\033[34m',
}
COULEUR_TECHNIQUE = '\033[2m'
COULEUR_ALERTE = '\033[1;31m'
ALERTES = ('[BLOQUÉ]', '[OSCILLATION]', '[CONTACT]', '[NAV2] ÉCHEC', '[CHEMIN] aucun')
NEUTRE = '\033[0m'
PERIODE_ETAT = 5.0


def _horloge(secondes):
    secondes = max(0, int(secondes))
    return f'{secondes // 60:02d}:{secondes % 60:02d}'


def formater(evenement, couleurs=True):
    """Lignes à afficher pour un événement du récit (dictionnaire JSON)."""
    genre = evenement.get('type', '')
    if 'temps_ecoule' in evenement:
        entete = (f'[{_horloge(evenement["temps_ecoule"])} | reste '
                  f'{_horloge(evenement.get("temps_restant", 0))}] ')
    else:
        entete = '[micro] '        # nœud du mot de départ : pas de chronomètre
    debut, fin = (COULEURS.get(genre, ''), NEUTRE) if couleurs else ('', '')
    if genre == 'fin' and evenement.get('lignes'):
        lignes = [f'{entete}{debut}=== FIN ==={fin}']
        lignes += [f'    {debut}{ligne}{fin}' for ligne in evenement['lignes']]
        return lignes
    return [f'{entete}{debut}{evenement.get("texte", "")}{fin}']


def formater_technique(ligne, couleurs=True):
    if not couleurs:
        return f'        {ligne}'
    couleur = COULEUR_ALERTE if ligne.startswith(ALERTES) else COULEUR_TECHNIQUE
    return f'        {couleur}{ligne}{NEUTRE}'


def _longueur(poses):
    points = [(p.pose.position.x, p.pose.position.y) for p in poses]
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def _lacet(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def creer_noeud_suivi(nom='suivi_course', technique=True, quitter_a_la_fin=False):
    """Nœud de suivi (abonnements seulement : ne commande jamais le robot)."""
    from action_msgs.msg import GoalStatusArray
    from nav_msgs.msg import Odometry, Path
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener

    from caytu_config import load_config
    from caytu_course.statut import qos_evenements
    from caytu_course.surveillance import Surveillance

    configuration = load_config()

    class NoeudSuivi(Node):

        def __init__(self):
            # Horloge de la simulation si besoin : un Gazebo lent ne doit pas
            # faire croire à un robot bloqué.
            super().__init__(nom, parameter_overrides=[Parameter(
                'use_sim_time', value=configuration.system.use_sim_time)])
            self.couleurs = sys.stdout.isatty() or '--couleurs' in sys.argv
            self.fini = False
            self.quitter_a_la_fin = quitter_a_la_fin
            self.surveillance = Surveillance()
            self._statuts = {}             # but Nav2 -> dernier statut affiché
            self.create_subscription(String, configuration.course.events_topic,
                                     self._recit, qos_evenements())
            if not technique:
                return
            navigation = configuration.navigation
            self._memoire_tf = Buffer()
            self._ecouteur_tf = TransformListener(self._memoire_tf, self)
            self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                     self._statut_nav2, 10)
            self.create_subscription(Path, '/plan', self._chemin, 10)
            self.create_subscription(Twist, navigation.cmd_vel_topic, self._commande, 10)
            self.create_subscription(Odometry, navigation.odom_topic, self._odometrie,
                                     qos_profile_sensor_data)
            self.create_timer(PERIODE_ETAT, self._etat)

        def maintenant(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def afficher(self, lignes):
            for ligne in lignes:
                print(formater_technique(ligne, self.couleurs), flush=True)

        def _recit(self, message):
            try:
                evenement = json.loads(message.data)
            except ValueError:
                return
            for ligne in formater(evenement, self.couleurs):
                print(ligne, flush=True)
            if evenement.get('type') == 'fin':
                self.fini = True

        def _statut_nav2(self, message):
            """La liste contient tous les buts récents, pas forcément dans
            l'ordre : on suit chaque but par son identifiant."""
            for statut in message.status_list:
                but = bytes(statut.goal_info.goal_id.uuid)
                if self._statuts.get(but) == statut.status:
                    continue
                self._statuts[but] = statut.status
                self.afficher(self.surveillance.statut_nav2(
                    statut.status, self.maintenant()))

        def _chemin(self, message):
            self.afficher(self.surveillance.chemin(
                len(message.poses), _longueur(message.poses), self.maintenant()))

        def _commande(self, message):
            self.afficher(self.surveillance.commande(
                message.linear.x, message.angular.z, self.maintenant()))

        def _odometrie(self, message):
            pose = message.pose.pose
            self.afficher(self.surveillance.mouvement(
                pose.position.x, pose.position.y, _lacet(pose.orientation),
                self.maintenant()))

        def _etat(self):
            try:
                tf = self._memoire_tf.lookup_transform(
                    configuration.system.global_frame, configuration.system.base_frame,
                    Time())
                t = tf.transform
                pose = (t.translation.x, t.translation.y, _lacet(t.rotation))
            except TransformException:
                pose = None
            self.afficher([self.surveillance.ligne_etat(pose)])

        def continuer(self):
            return not (self.quitter_a_la_fin and self.fini)

    return NoeudSuivi()


def tourner(noeud):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    try:
        while rclpy.ok() and noeud.continuer():
            rclpy.spin_once(noeud, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        noeud.destroy_node()
        rclpy.try_shutdown()


def main():
    import rclpy
    rclpy.init()
    noeud = creer_noeud_suivi(
        technique='--sans-technique' not in sys.argv,
        quitter_a_la_fin='--quitter-a-la-fin' in sys.argv)
    print('Suivi de la course (Ctrl-C pour quitter, la course continue)...', flush=True)
    tourner(noeud)


if __name__ == '__main__':
    main()
