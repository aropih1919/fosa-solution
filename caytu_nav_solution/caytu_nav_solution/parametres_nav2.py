"""YAML Nav2 de la course, généré depuis `caytu_config`.

Base : `nav2_params.yaml` + contrôleur + profil du laboratoire. Tout ce qui
dépend du robot (repères, topics, empreinte, vitesses, scans d'obstacles) est
ensuite imposé par la configuration : le même code sert au robot réel et à
Gazebo, seul le profil change. La carte vient de slam_toolbox (`/map`).
"""

import math
from pathlib import Path

import yaml

from caytu_nav_solution.benchmark_config import (
    _read_yaml,
    _set_sim_time,
    _validate_final_config,
    deep_merge,
)

PLUGIN_OBSTACLES = 'nav2_costmap_2d::ObstacleLayer'


def _charger_base(configuration, racine):
    navigation = configuration.navigation
    controleur = navigation.controller
    parametres = _read_yaml(racine / 'nav2_params.yaml')
    if controleur == 'mppi':
        fichier_controleur = racine / 'controllers' / 'mppi.yaml'
    else:
        fichier_controleur = racine / f'controller_{controleur}.yaml'
    parametres = deep_merge(parametres, _read_yaml(fichier_controleur))
    return deep_merge(parametres, _read_yaml(
        racine / 'profiles' / navigation.profile / f'{controleur}.yaml'))


def _couche_obstacles(source):
    return {
        'plugin': PLUGIN_OBSTACLES,
        'enabled': True,
        'combination_method': 1,
        'observation_sources': source.name,
        source.name: {
            'topic': source.topic,
            'data_type': 'LaserScan',
            'marking': True,
            'clearing': True,
            # Vrai : un rayon sans retour efface jusqu'à raytrace_max_range (le
            # point créé à range_max n'est jamais marqué). Faux pour le lidar :
            # un pied de chaise noir sans retour n'efface pas l'obstacle.
            'inf_is_valid': source.inf_is_valid,
            'obstacle_min_range': source.obstacle_min_range,
            'obstacle_max_range': source.obstacle_max_range,
            'raytrace_min_range': 0.0,
            'raytrace_max_range': source.raytrace_max_range,
            'min_obstacle_height': -0.3,
            'max_obstacle_height': 2.0,
            'observation_persistence': 0.0,
            'expected_update_rate': 0.0,
        },
    }


def _empreinte(navigation):
    return '[' + ', '.join(
        f'[{x:.3f}, {y:.3f}]' for x, y in navigation.footprint) + ']'


def _costmap(base, configuration, repere_global, globale):
    """Remplace repères, empreinte et couches d'une costmap de la base."""
    navigation = configuration.navigation
    sources = [
        source for source in navigation.obstacle_sources
        if source.enabled and (source.global_costmap or not globale)]
    costmap = {
        cle: valeur for cle, valeur in base.items()
        if not (isinstance(valeur, dict) and 'plugin' in valeur)}
    costmap.update({
        'global_frame': repere_global,
        'robot_base_frame': configuration.system.base_frame,
        'footprint': _empreinte(navigation),
        'footprint_padding': navigation.footprint_padding,
        'plugins': (['static_layer'] if globale else [])
        + [f'{source.name}_layer' for source in sources] + ['inflation_layer'],
        'inflation_layer': base['inflation_layer'],
    })
    if globale:
        # Carte publiée en continu par slam_toolbox (transient local).
        costmap['static_layer'] = {
            'plugin': 'nav2_costmap_2d::StaticLayer',
            'map_subscribe_transient_local': True,
            'subscribe_to_updates': False,
        }
    for source in sources:
        costmap[f'{source.name}_layer'] = _couche_obstacles(source)
    return costmap


def _limiter_vitesses(suivi, controleur, navigation):
    lineaire, angulaire = navigation.max_linear_speed, navigation.max_angular_speed
    cles = {
        'rpp': (('desired_linear_vel',), ('rotate_to_heading_angular_vel',)),
        'dwb': (('max_vel_x', 'max_speed_xy'), ('max_vel_theta',)),
        'mppi': (('vx_max',), ('wz_max',)),
    }[controleur]
    for cle in cles[0]:
        if cle in suivi:
            suivi[cle] = min(float(suivi[cle]), lineaire)
    for cle in cles[1]:
        if cle in suivi:
            suivi[cle] = min(float(suivi[cle]), angulaire)


def construire_parametres_course(configuration, config_root=None):
    """Dictionnaire de paramètres complet pour les nœuds Nav2 de la course."""
    if config_root is None:
        from ament_index_python.packages import get_package_share_directory
        config_root = Path(
            get_package_share_directory('caytu_nav_bringup')) / 'config'
    racine = Path(config_root)
    systeme, navigation = configuration.system, configuration.navigation
    course = configuration.course
    base = _charger_base(configuration, racine)
    carte, robot, odom = systeme.global_frame, systeme.base_frame, navigation.odom_frame

    # Le cap du but doit être conservé (face au tag) : Smac ne doit pas le
    # remplacer par l'orientation du dernier segment du chemin.
    base['planner_server']['ros__parameters']['GridBased'][
        'use_final_approach_orientation'] = False
    bt = base['bt_navigator']['ros__parameters']
    bt.update(global_frame=carte, robot_base_frame=robot,
              odom_topic=navigation.odom_topic)
    controleur = base['controller_server']['ros__parameters']
    controleur['odom_topic'] = navigation.odom_topic
    controleur['goal_checker'].update(
        xy_goal_tolerance=course.goal_xy_tolerance_m,
        yaw_goal_tolerance=math.radians(course.goal_yaw_tolerance_deg))
    _limiter_vitesses(controleur['FollowPath'], navigation.controller, navigation)
    comportements = base['behavior_server']['ros__parameters']
    comportements.update(
        local_frame=odom, global_frame=carte, robot_base_frame=robot,
        # Le tour de recherche (behavior Spin) est lent : pas de flou de bougé.
        max_rotational_vel=min(
            float(comportements['max_rotational_vel']),
            navigation.max_angular_speed, course.search_spin_speed_rps))
    for nom, globale, repere in (
            ('global_costmap', True, carte), ('local_costmap', False, odom)):
        parametres = base[nom][nom]['ros__parameters']
        base[nom][nom]['ros__parameters'] = _costmap(
            parametres, configuration, repere, globale)

    _set_sim_time(base, systeme.use_sim_time)
    _validate_final_config(base, navigation.controller, navigation.profile)
    return base


def ecrire_parametres_course(configuration, chemin, config_root=None):
    chemin = Path(chemin)
    chemin.parent.mkdir(parents=True, exist_ok=True)
    with chemin.open('w', encoding='utf-8') as flux:
        yaml.safe_dump(
            construire_parametres_course(configuration, config_root), flux,
            sort_keys=False, allow_unicode=True)
    return chemin
