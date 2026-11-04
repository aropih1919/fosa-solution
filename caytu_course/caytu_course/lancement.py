"""Paramètres calculés au lancement de la course (sans ROS)."""

from pathlib import Path

EXTENSIONS_POSEGRAPH = ('.posegraph', '.data')


def executable_slam(configuration):
    """Le nœud asynchrone ignore `mode` : la localisation sur une carte des
    essais demande le nœud dédié de slam_toolbox."""
    if configuration.course.slam_map_file:
        return 'localization_slam_toolbox_node'
    return 'async_slam_toolbox_node'


def parametres_slam(configuration):
    """Surcharges de slam_toolbox : repères, scan, mode et carte éventuelle.

    Sans `course.slam_map_file`, la carte est construite pendant la course.
    Avec un posegraph sérialisé pendant les essais, slam_toolbox se localise
    dedans en partant de l'origine (même position de départ que lors de la
    cartographie).
    """
    systeme, navigation = configuration.system, configuration.navigation
    parametres = {
        'use_sim_time': systeme.use_sim_time,
        # Transitions pilotées par le lifecycle manager de la course (sans bond).
        'use_lifecycle_manager': False,
        'odom_frame': navigation.odom_frame,
        'map_frame': systeme.global_frame,
        'base_frame': systeme.base_frame,
        'scan_topic': navigation.scan_topic,
        'mode': 'mapping',
    }
    carte = configuration.course.slam_map_file
    if carte:
        chemin = Path(carte).expanduser()
        if chemin.suffix in EXTENSIONS_POSEGRAPH:
            chemin = chemin.with_suffix('')
        parametres.update(
            mode='localization', map_file_name=str(chemin),
            map_start_pose=[0.0, 0.0, 0.0])
    return parametres
