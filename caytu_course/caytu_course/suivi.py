#!/usr/bin/env python3
"""Affiche en direct, en phrases, ce que fait le robot (`./run.sh suivi`).

Lit le topic des événements de la course : tags repérés, plans de la
stratégie, sauts expliqués, échecs, visites, résumé final. L'historique est
gardé : lancé en retard, l'afficheur reçoit tout depuis le début.
"""

import json
import sys

COULEURS = {
    'visite': '\033[1;32m', 'saut': '\033[1;33m', 'echec': '\033[33m',
    'perdu': '\033[33m', 'fin': '\033[1;36m', 'plan': '\033[36m',
    'tag_repere': '\033[35m', 'depart': '\033[1m', 'ordre': '\033[1m',
}
NEUTRE = '\033[0m'


def _horloge(secondes):
    secondes = max(0, int(secondes))
    return f'{secondes // 60:02d}:{secondes % 60:02d}'


def formater(evenement, couleurs=True):
    """Lignes à afficher pour un événement (dictionnaire JSON de la course)."""
    genre = evenement.get('type', '')
    entete = (f'[{_horloge(evenement.get("temps_ecoule", 0))} | reste '
              f'{_horloge(evenement.get("temps_restant", 0))}] ')
    debut, fin = (COULEURS.get(genre, ''), NEUTRE) if couleurs else ('', '')
    if genre == 'fin' and evenement.get('lignes'):
        lignes = [f'{entete}{debut}=== FIN ==={fin}']
        lignes += [f'    {debut}{ligne}{fin}' for ligne in evenement['lignes']]
        return lignes
    return [f'{entete}{debut}{evenement.get("texte", "")}{fin}']


def main():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from std_msgs.msg import String

    from caytu_config import load_config
    from caytu_course.statut import qos_evenements

    quitter_a_la_fin = '--quitter-a-la-fin' in sys.argv
    couleurs = sys.stdout.isatty() or '--couleurs' in sys.argv
    rclpy.init()
    noeud = rclpy.create_node('suivi_course')
    fini = []

    def recevoir(message):
        try:
            evenement = json.loads(message.data)
        except ValueError:
            return
        for ligne in formater(evenement, couleurs):
            print(ligne, flush=True)
        if evenement.get('type') == 'fin':
            fini.append(True)

    noeud.create_subscription(
        String, load_config().course.events_topic, recevoir, qos_evenements())
    print('Suivi de la course (Ctrl-C pour quitter, la course continue)...', flush=True)
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
