"""Réglages de conduite modifiables depuis la ligne de commande (sans ROS).

Les valeurs utilisées par défaut sont dans caytu_nav_bringup : nav2_params.yaml
et solution_bringup.launch.py. Ce module sert à COMPARER : il permet de
revenir, pour un essai, aux valeurs de la version mesurée à 2 min 33 s, en
bloc (drive_profile:=reference) ou réglage par réglage.

    ros2 run caytu_nav_solution task_solution.py --ros-args -p drive_profile:=reference
    ros2 run caytu_nav_solution task_solution.py --ros-args -p cruise_speed:=0.5

Le rapport de fin de trajet rappelle les réglages utilisés.
"""

from typing import Dict, List, Optional

PROFILE_DEFAULT = 'default'
PROFILE_REFERENCE = 'reference'

# nom -> (valeur de la version mesurée à 2 min 33 s, minimum, maximum).
# Les bornes viennent du robot : le plugin DiffDrive officiel plafonne les
# commandes à 1,5 m/s et 1 rad/s.
SETTINGS = {
    'cruise_speed': (0.5, 0.1, 1.5),         # vitesse de croisière, m/s
    'turn_speed': (0.8, 0.3, 1.0),           # pivot sur place, rad/s
    'approach_speed': (0.05, 0.02, 0.5),     # vitesse minimale à l'arrivée, m/s
    'approach_distance': (0.6, 0.1, 2.0),    # distance de ralentissement avant le but, m
    'curve_radius': (0.9, 0.2, 2.0),         # rayon sous lequel le robot ralentit en virage, m
    'camera_range': (2.5, 1.0, 8.0),         # portée de marquage de la caméra haute, m
    'axle_offset': (0.0, 0.0, 0.2),          # distance base_footprint -> essieu prise en compte, m
    'keeper_gain': (1.5, 0.1, 10.0),         # gain exigé pour changer de route, m (si path_keeper)
}
# Réglages oui/non : nom -> (valeur par défaut, valeur de la version de
# référence). Toujours transmis au lancement : task_solution.py doit savoir si
# le nœud path_keeper est attendu.
SWITCHES = {
    'path_keeper': (True, False),            # garder le chemin tant qu'un autre n'est pas nettement meilleur
}
NAMES = tuple(SETTINGS) + tuple(SWITCHES)


def _parse_bool(name: str, value) -> bool:
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ('true', '1', 'yes', 'oui'):
        return True
    if text in ('false', '0', 'no', 'non'):
        return False
    raise ValueError(f'{name} : « {value} » n\'est ni true ni false')


def _parse_number(name: str, value) -> float:
    if isinstance(value, bool):
        raise ValueError(f'{name} : un nombre est attendu, pas « {value} »')
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ValueError(f'{name} : « {value} » n\'est pas un nombre') from None
    _, low, high = SETTINGS[name]
    if not (low <= number <= high):       # refuse aussi NaN
        raise ValueError(f'{name} : {value} est hors de l\'intervalle [{low}, {high}]')
    return number


def resolve(profile: str, overrides: Optional[Dict[str, object]] = None) -> Dict[str, object]:
    """Réglages à imposer au lancement ; un réglage numérique absent garde la
    valeur de nav2_params.yaml. Les réglages oui/non sont toujours présents.

    profile   : 'default' ou 'reference' (valeurs de la version mesurée à
                2 min 33 s).
    overrides : {nom: valeur} donnés en ligne de commande ; None ou '' = non donné.
    Lève ValueError si un nom, un profil ou une valeur est invalide.
    """
    profile = str(profile).strip().lower()
    if profile not in (PROFILE_DEFAULT, PROFILE_REFERENCE):
        raise ValueError(
            f'drive_profile : « {profile} » inconnu ({PROFILE_DEFAULT} ou {PROFILE_REFERENCE})')
    reference = profile == PROFILE_REFERENCE
    values: Dict[str, object] = {name: spec[1 if reference else 0]
                                 for name, spec in SWITCHES.items()}
    if reference:
        values.update({name: spec[0] for name, spec in SETTINGS.items()})
    for name, value in (overrides or {}).items():
        if value is None or (isinstance(value, str) and not value.strip()):
            continue
        if name in SETTINGS:
            values[name] = _parse_number(name, value)
        elif name in SWITCHES:
            values[name] = _parse_bool(name, value)
        else:
            raise ValueError(f'réglage inconnu : {name}')
    return values


def _text(value) -> str:
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return f'{value:g}'


def launch_arguments(values: Dict[str, object]) -> List[str]:
    """Arguments `nom:=valeur` à ajouter à la commande `ros2 launch`."""
    return [f'{name}:={_text(values[name])}' for name in NAMES if name in values]


def describe(profile: str, overrides: Optional[Dict[str, object]] = None) -> str:
    """Texte court pour le rapport : profil, puis réglages donnés à la main."""
    profile = str(profile).strip().lower()
    values = resolve(profile, overrides)
    given = [name for name, value in (overrides or {}).items()
             if value is not None and not (isinstance(value, str) and not value.strip())]
    parts = [profile] + [f'{name}={_text(values[name])}' for name in NAMES if name in given]
    return ' + '.join(parts)


# --------------------------------------------------------------------------- #
# Application à la configuration Nav2 (utilisé par les fichiers de lancement)
# --------------------------------------------------------------------------- #
# Réglage -> paramètre du contrôleur (controller_server / FollowPath).
_CONTROLLER_KEYS = {
    'cruise_speed': 'desired_linear_vel',
    'turn_speed': 'rotate_to_heading_angular_vel',
    'approach_speed': 'min_approach_linear_velocity',
    'approach_distance': 'approach_velocity_scaling_dist',
    'curve_radius': 'regulated_linear_scaling_min_radius',
}
# La caméra efface (et son scan porte) un peu plus loin qu'elle ne marque : un
# rayon sans obstacle arrive à range_max et ne doit pas être pris pour un obstacle.
CAMERA_CLEAR_MARGIN = 0.5


def camera_scan_range(camera_range: float) -> float:
    """range_max du scan de la caméra haute pour une portée de marquage donnée."""
    return float(camera_range) + CAMERA_CLEAR_MARGIN


def apply_to_nav2_params(params: dict, values: Dict[str, object]) -> dict:
    """Écrit les réglages imposés dans la configuration Nav2 (modifiée sur place).

    params : contenu de nav2_params.yaml, lu avec yaml.safe_load.
    values : résultat de resolve() ; seuls les réglages présents sont écrits.
    """
    follow = params['controller_server']['ros__parameters']['FollowPath']
    for name, key in _CONTROLLER_KEYS.items():
        if name in values:
            follow[key] = float(values[name])
    if 'camera_range' in values:
        reach = float(values['camera_range'])
        for costmap in ('global_costmap', 'local_costmap'):
            camera = params[costmap][costmap]['ros__parameters']['top_camera_layer']['top_cam']
            camera['obstacle_max_range'] = reach
            camera['raytrace_max_range'] = camera_scan_range(reach)
    return params


def from_launch_text(texts: Dict[str, str]) -> Dict[str, object]:
    """Convertit les arguments de lancement (texte, '' = non donné) en réglages."""
    numbers = resolve(PROFILE_DEFAULT, {name: texts.get(name, '') for name in SETTINGS})
    numbers = {name: value for name, value in numbers.items() if name in SETTINGS}
    switches = {name: _parse_bool(name, texts[name]) for name in SWITCHES
                if str(texts.get(name, '')).strip()}
    return {**numbers, **switches}
