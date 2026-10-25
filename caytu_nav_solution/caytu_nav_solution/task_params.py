"""Lecture de task_params.yaml (package officiel parc_robot_bringup).

Ce fichier est la seule source du point de départ et du but : task.launch.py
l'utilise pour faire apparaître le robot et le cercle vert. Les valeurs sont
exprimées dans le repère du monde Gazebo. Aucune coordonnée n'est écrite en dur
dans la solution : si le jury modifie le spawn ou le but, la solution suit.
"""

import os
from dataclasses import dataclass
from typing import Optional

import yaml

from caytu_nav_solution.nav_math import Pose2D


@dataclass(frozen=True)
class TaskParams:
    spawn: Pose2D
    goal_x: float
    goal_y: float
    path: str


def default_task_params_path() -> str:
    """Chemin de task_params.yaml dans le package officiel installé."""
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(
        get_package_share_directory('parc_robot_bringup'), 'config', 'task_params.yaml')


def load_task_params(path: Optional[str] = None) -> TaskParams:
    """Charge le spawn et le but. Lève une exception explicite si le fichier manque."""
    path = path or default_task_params_path()
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    try:
        params = data['/**']['ros__parameters']
        return TaskParams(
            spawn=Pose2D(float(params['x']), float(params['y']), float(params['yaw'])),
            goal_x=float(params['goal_x']),
            goal_y=float(params['goal_y']),
            path=path,
        )
    except (KeyError, TypeError) as error:
        raise ValueError(f'task_params.yaml invalide ({path}) : {error}') from error
