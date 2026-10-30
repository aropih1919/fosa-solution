"""Lecture de task_params.yaml (paquet officiel parc_robot_bringup).

Ce fichier est la seule source du point de départ et du but : task.launch.py
l'utilise pour faire apparaître le robot et le cercle vert. Les valeurs sont
exprimées dans le repère du monde Gazebo. Aucune coordonnée n'est écrite en dur
dans la solution : si le jury modifie le départ ou le but, la solution suit.
"""

import os

import yaml

from caytu_nav_solution.calculs_nav import Pose2D


class ParametresTache:
    """Contient le départ, le but et le chemin du fichier lu."""

    def __init__(self, depart, but_x, but_y, chemin):
        self.depart = depart      # Pose2D : position et cap de départ
        self.but_x = but_x        # le but n'a pas de cap : seulement x et y
        self.but_y = but_y
        self.chemin = chemin      # chemin du fichier lu (utile pour les erreurs)


def chemin_parametres_tache_par_defaut():
    """Chemin de task_params.yaml dans le paquet officiel installé."""
    # L'import est fait ici pour que ce module reste utilisable sans ROS,
    # par exemple dans les tests unitaires.
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(
        get_package_share_directory('parc_robot_bringup'), 'config', 'task_params.yaml')


def charger_parametres_tache(chemin=None):
    """Charge le départ et le but. Lève une erreur claire si le fichier manque."""
    chemin = chemin or chemin_parametres_tache_par_defaut()

    with open(chemin, encoding='utf-8') as flux:
        donnees = yaml.safe_load(flux)

    try:
        # Un fichier de paramètres ROS 2 a toujours cette forme :
        #   /**:
        #     ros__parameters:
        #       x: ...
        # `/**` est un joker qui veut dire « pour tous les nœuds ».
        parametres = donnees['/**']['ros__parameters']
        return ParametresTache(
            depart=Pose2D(
                float(parametres['x']),
                float(parametres['y']),
                float(parametres['yaw']),
            ),
            but_x=float(parametres['goal_x']),
            but_y=float(parametres['goal_y']),
            chemin=chemin,
        )
    except (KeyError, TypeError) as erreur:
        raise ValueError(
            f'task_params.yaml invalide ({chemin}) : {erreur}') from erreur
