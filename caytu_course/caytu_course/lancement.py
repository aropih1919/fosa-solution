"""Paramètres calculés au lancement de la course (sans ROS).

Trois modes :
  - `preparation` : nouvelle carte, exploration autonome, carte et tags
    enregistrés dans `course.preparation.directory` ;
  - `preparation_manuelle` : SLAM et tags enregistrés pendant que l'équipe
    guide le robot ; aucune cible autonome n'est envoyée ;
  - `course` : le match ; se localise sur la carte de la préparation si elle
    existe, sinon cartographie en direct ;
  - `cartographie` : SLAM, Nav2 et perception sans mission (réglages).
"""

from pathlib import Path

MODES = ('course', 'preparation', 'preparation_manuelle', 'cartographie')
NOM_CARTE = 'carte'            # carte.posegraph + carte.data (slam_toolbox)
NOM_TAGS = 'tags.yaml'


def dossier_preparation(configuration):
    return Path(configuration.course.preparation.directory).expanduser()


def carte_preparee(configuration):
    """Chemin de la carte enregistrée (sans extension), ou None."""
    base = dossier_preparation(configuration) / NOM_CARTE
    if base.with_suffix('.posegraph').is_file() and base.with_suffix('.data').is_file():
        return base
    return None


def fichier_tags(configuration):
    return dossier_preparation(configuration) / NOM_TAGS


def _localiser(configuration, mode):
    return mode == 'course' and carte_preparee(configuration) is not None


def executable_slam(configuration, mode='course'):
    """Le nœud asynchrone ignore `mode` : la localisation sur la carte de la
    préparation demande le nœud dédié de slam_toolbox."""
    if _localiser(configuration, mode):
        return 'localization_slam_toolbox_node'
    return 'async_slam_toolbox_node'


def parametres_slam(configuration, mode='course'):
    """Surcharges de slam_toolbox : repères, scan, mode et carte éventuelle.

    Sur la carte de la préparation, slam_toolbox se localise en partant de
    l'origine : le match doit partir de la position où la préparation a
    commencé (la position de départ officielle).
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
    if _localiser(configuration, mode):
        parametres.update(
            mode='localization', map_file_name=str(carte_preparee(configuration)),
            map_start_pose=[0.0, 0.0, 0.0])
    return parametres
