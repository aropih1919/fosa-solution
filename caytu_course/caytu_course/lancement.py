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

import math
import os
from pathlib import Path

from caytu_config import config_directory

MODES = ('course', 'preparation', 'preparation_manuelle', 'cartographie')
NOM_CARTE = 'carte'            # carte.posegraph + carte.data (slam_toolbox)
NOM_TAGS = 'tags.yaml'


def dossier_preparation(configuration):
    return Path(configuration.course.preparation.directory).expanduser()


def _base_carte(chemin, racine=None):
    """Dossier ou fichier de carte -> préfixe sans extension.

    Un chemin relatif est résolu avant de tester le dossier : sinon
    `cartes/arene_2026_11_06` est vu comme un simple préfixe et la carte
    livrée n'est jamais trouvée depuis la racine du dépôt.
    """
    base = Path(chemin).expanduser()
    if not base.is_absolute():
        base = (Path(racine) if racine is not None else Path.cwd()) / base
    if base.is_dir():
        posegraphs = sorted(base.glob('*.posegraph'))
        # Un seul fichier : son nom est celui de la carte. Plusieurs : le
        # nom conventionnel `carte` tranche, sinon le dossier est ambigu.
        base = (posegraphs[0].with_suffix('') if len(posegraphs) == 1
                else base / NOM_CARTE)
    if base.suffix in ('.posegraph', '.data', '.yaml', '.pgm'):
        base = base.with_suffix('')
    return base


# Taille minimale d'une carte SLAM valide (octets). Une préparation
# interrompue laisse des fichiers partiels non vides mais trop petits pour
# être une carte exploitable : on les rejette pour basculer en sans carte.
TAILLE_MIN_CARTE_OCTETS = 100 * 1024


def _fichier_non_vide(chemin, taille_min=TAILLE_MIN_CARTE_OCTETS):
    try:
        return chemin.is_file() and chemin.stat().st_size >= taille_min
    except OSError:
        return False


def _carte_valide(base):
    return (_fichier_non_vide(base.with_suffix('.posegraph'))
            and _fichier_non_vide(base.with_suffix('.data')))


def _carte_embarquee(configuration):
    """Carte forcée par FOSA_CARTE, sinon carte livrée avec la solution."""
    forcee = os.environ.get('FOSA_CARTE')
    chemin = forcee or configuration.course.preparation.carte_embarquee
    if not chemin:
        return None
    racine = Path.cwd() if forcee else config_directory()
    base = _base_carte(chemin, racine)
    return base if _carte_valide(base) else None


def carte_preparee(configuration):
    """Chemin de la carte utilisée pour le match (sans extension), ou None.

    Une préparation enregistrée sur le robot est prioritaire : elle correspond
    à l'arène du jour. Sinon, la carte embarquée dans la livraison est utilisée.
    `FOSA_CARTE` force une carte précise (simulation ou autre arène).
    """
    if configuration.navigation.mode == 'sans_carte':
        return None
    if os.environ.get('FOSA_CARTE'):
        return _carte_embarquee(configuration)
    base = _base_carte(dossier_preparation(configuration))
    if _carte_valide(base):
        return base
    return _carte_embarquee(configuration)


def fichier_tags(configuration):
    """Fichier écrit pendant une préparation (dossier modifiable du robot)."""
    return dossier_preparation(configuration) / NOM_TAGS


def fichier_tags_match(configuration):
    """Tags associés à la carte réellement utilisée pour le match."""
    carte = carte_preparee(configuration)
    return (carte.parent if carte is not None else dossier_preparation(configuration)) / NOM_TAGS


def pose_depart(configuration):
    """Pose [x, y, lacet] du début de carte, surchargeable par FOSA_POSE_DEPART.

    La variable d'environnement accepte les espaces ou les virgules :
    `FOSA_POSE_DEPART="1.2 -0.4 0.0"`.
    """
    valeur = os.environ.get('FOSA_POSE_DEPART', '').strip()
    if not valeur:
        return list(configuration.course.preparation.map_start_pose)
    morceaux = valeur.replace(',', ' ').split()
    try:
        nombres = [float(morceau) for morceau in morceaux]
    except ValueError as erreur:
        raise ValueError(
            'FOSA_POSE_DEPART doit contenir trois nombres : x y lacet.') from erreur
    if len(nombres) != 3 or not all(math.isfinite(n) for n in nombres):
        raise ValueError(
            'FOSA_POSE_DEPART doit contenir trois nombres finis : x y lacet.')
    return nombres


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
            map_start_pose=pose_depart(configuration))
    return parametres
