"""YAML Nav2 de la course, généré depuis `caytu_config`.

Base : `nav2_params.yaml`, `controller_rpp.yaml` et profil `balanced`. Tout ce
qui dépend du robot (repères, topics, empreinte, vitesses, scans d'obstacles)
est ensuite imposé par la configuration : le même code sert au robot réel et à
Gazebo, seul le profil change. La carte vient de slam_toolbox (`/map`).
"""

from copy import deepcopy
import math
from pathlib import Path

import yaml

from caytu_config import ConfigurationError

PLUGIN_OBSTACLES = 'nav2_costmap_2d::ObstacleLayer'
PLUGIN_RPP = (
    'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController')


def _lire_yaml(chemin):
    with Path(chemin).open(encoding='utf-8') as flux:
        valeur = yaml.safe_load(flux)
    if not isinstance(valeur, dict):
        raise ConfigurationError(
            f'Le YAML doit contenir un mapping : {chemin}')
    return valeur


def _fusionner(base, surcharge):
    """Fusionne les mappings récursivement ; listes et valeurs remplacées."""
    resultat = deepcopy(base)
    for cle, valeur in surcharge.items():
        if isinstance(valeur, dict) and isinstance(resultat.get(cle), dict):
            resultat[cle] = _fusionner(resultat[cle], valeur)
        else:
            resultat[cle] = deepcopy(valeur)
    return resultat


def _imposer_sim_time(parametres, use_sim_time):
    """Applique l'horloge à chaque bloc de paramètres ROS de la configuration."""
    for valeur in parametres.values():
        if not isinstance(valeur, dict):
            continue
        if isinstance(valeur.get('ros__parameters'), dict):
            valeur['ros__parameters']['use_sim_time'] = bool(use_sim_time)
        _imposer_sim_time(valeur, use_sim_time)


def _charger_base(configuration, racine):
    navigation = configuration.navigation
    parametres = _lire_yaml(racine / 'nav2_params.yaml')
    parametres = _fusionner(
        parametres, _lire_yaml(racine / 'controller_rpp.yaml'))
    return _fusionner(parametres, _lire_yaml(
        racine / 'profiles' / navigation.profile / 'rpp.yaml'))


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
    carte_statique = globale and navigation.mode == 'carte'
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
        'plugins': (['static_layer'] if carte_statique else [])
        + [f'{source.name}_layer' for source in sources] + ['inflation_layer'],
        'inflation_layer': base['inflation_layer'],
    })
    if carte_statique:
        # Carte publiée en continu par slam_toolbox (transient local).
        costmap['static_layer'] = {
            'plugin': 'nav2_costmap_2d::StaticLayer',
            'map_subscribe_transient_local': True,
            'subscribe_to_updates': False,
        }
    elif globale:
        costmap.update(
            rolling_window=True, track_unknown_space=True,
            width=int(navigation.rolling_window_size_m),
            height=int(navigation.rolling_window_size_m),
            always_send_full_costmap=True)
    for source in sources:
        costmap[f'{source.name}_layer'] = _couche_obstacles(source)
    return costmap


def _limiter_vitesses(suivi, navigation):
    if 'desired_linear_vel' in suivi:
        suivi['desired_linear_vel'] = min(
            float(suivi['desired_linear_vel']), navigation.max_linear_speed)
    if 'rotate_to_heading_angular_vel' in suivi:
        suivi['rotate_to_heading_angular_vel'] = min(
            float(suivi['rotate_to_heading_angular_vel']),
            navigation.max_angular_speed)


def _valider(parametres):
    """Vérifie la cohérence du RPP et des costmaps avant de lancer Nav2."""
    controleur = parametres['controller_server']['ros__parameters']
    if controleur.get('controller_plugins') != ['FollowPath']:
        raise ConfigurationError(
            'Le controller_server doit avoir un seul plugin FollowPath.')
    suivi = controleur['FollowPath']
    if suivi.get('plugin') != PLUGIN_RPP:
        raise ConfigurationError(
            f"Plugin RPP invalide : {suivi.get('plugin')!r}")

    for nom in ('global_costmap', 'local_costmap'):
        bloc = parametres[nom][nom]['ros__parameters']
        if 'inflation_layer' not in bloc['plugins']:
            raise ConfigurationError(
                f'{nom} : la couche inflation_layer est obligatoire.')
        capteurs = [
            cle for cle in bloc['plugins']
            if cle not in ('static_layer', 'inflation_layer')]
        if not capteurs:
            raise ConfigurationError(f'{nom} : aucune couche capteur.')
        if bloc.get('footprint_padding', 0.0) < 0.02:
            raise ConfigurationError(f'{nom} : marge de footprint réduite.')

    inflation = parametres['local_costmap']['local_costmap'][
        'ros__parameters']['inflation_layer']
    if suivi.get('cost_scaling_dist', 0.0) > inflation['inflation_radius']:
        raise ConfigurationError(
            "RPP cost_scaling_dist dépasse le rayon d'inflation local.")
    if not math.isclose(
            suivi.get('inflation_cost_scaling_factor', -1.0),
            inflation['cost_scaling_factor'], rel_tol=1e-6):
        raise ConfigurationError(
            "Le facteur d'inflation de RPP doit correspondre à la costmap "
            'locale.')


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
    carte, robot, odom = (
        systeme.global_frame, systeme.base_frame, navigation.odom_frame)

    # Le cap du but doit être conservé (face au tag) : Smac ne doit pas le
    # remplacer par l'orientation du dernier segment du chemin.
    base['planner_server']['ros__parameters']['GridBased'][
        'use_final_approach_orientation'] = False
    if navigation.mode == 'sans_carte':
        base['planner_server']['ros__parameters']['GridBased']['allow_unknown'] = False
    bt = base['bt_navigator']['ros__parameters']
    bt.update(global_frame=carte, robot_base_frame=robot,
              odom_topic=navigation.odom_topic)
    controleur = base['controller_server']['ros__parameters']
    controleur['odom_topic'] = navigation.odom_topic
    controleur['goal_checker'].update(
        xy_goal_tolerance=course.goal_xy_tolerance_m,
        yaw_goal_tolerance=math.radians(course.goal_yaw_tolerance_deg))
    _limiter_vitesses(controleur['FollowPath'], navigation)
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

    _imposer_sim_time(base, systeme.use_sim_time)
    _valider(base)
    return base


def ecrire_parametres_course(configuration, chemin, config_root=None):
    chemin = Path(chemin)
    chemin.parent.mkdir(parents=True, exist_ok=True)
    with chemin.open('w', encoding='utf-8') as flux:
        yaml.safe_dump(
            construire_parametres_course(configuration, config_root), flux,
            sort_keys=False, allow_unicode=True)
    return chemin
