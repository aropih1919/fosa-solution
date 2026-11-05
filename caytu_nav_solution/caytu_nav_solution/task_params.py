"""Lecture de task_params.yaml (paquet officiel parc_robot_bringup).

Ce fichier est la seule source du point de départ et du but : task.launch.py
l'utilise pour faire apparaître le robot et le cercle vert. Les valeurs sont
exprimées dans le repère du monde Gazebo. Aucune coordonnée n'est écrite en dur
dans la solution : si le jury modifie le départ ou le but, la solution suit.
"""

import os
from dataclasses import dataclass
from typing import Optional

import yaml

from caytu_nav_solution.nav_math import Pose2D


@dataclass(frozen=True)
class TaskParams:
    """Stores the start pose, goal coordinates, and loaded file path."""

    spawn: Pose2D
    goal_x: float
    goal_y: float
    path: str


def default_task_params_path() -> str:
    """Chemin de task_params.yaml dans le paquet officiel installé."""
    # L'import est fait ici pour que ce module reste utilisable sans ROS,
    # par exemple dans les tests unitaires.
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(
        get_package_share_directory('parc_robot_bringup'), 'config', 'task_params.yaml')


def load_task_params(path: Optional[str] = None) -> TaskParams:
    """Charge le départ et le but, et signale clairement un fichier manquant."""
    path = path or default_task_params_path()

    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)

    try:
        # Un fichier de paramètres ROS 2 a toujours cette forme :
        #   /**:
        #     ros__parameters:
        #       x: ...
        # `/**` est un joker qui veut dire « pour tous les nœuds ».
        params = data['/**']['ros__parameters']
        return TaskParams(
            spawn=Pose2D(
                float(params['x']),
                float(params['y']),
                float(params['yaw']),
            ),
            goal_x=float(params['goal_x']),
            goal_y=float(params['goal_y']),
            path=path,
        )
    except (KeyError, TypeError) as error:
        raise ValueError(
            f'task_params.yaml invalide ({path}) : {error}') from error
