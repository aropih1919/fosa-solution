"""Lancement complet de la course (robot réel par défaut, Gazebo par profil).

    ros2 launch caytu_course course.launch.py                   # match
    ros2 launch caytu_course course.launch.py mode:=preparation # carte + tags
    ros2 launch caytu_course course.launch.py mode:=cartographie  # sans mission

Démarre slam_toolbox et Nav2 sous un seul lifecycle manager, un détecteur
AprilTag par caméra, le mot de départ (si activé) et le nœud de course. Le
robot ne bouge qu'après le départ (commande ou voix). Le YAML Nav2 est généré
depuis caytu_config : repères, topics et vitesses suivent le profil actif.
"""

import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from caytu_config import load_config
from caytu_course.lancement import MODES, executable_slam, parametres_slam
from caytu_nav_solution.parametres_nav2 import ecrire_parametres_course
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NOEUDS_GERES = [
    'slam_toolbox', 'controller_server', 'planner_server', 'behavior_server',
    'bt_navigator',
]


def _filtre_laser(configuration, dossier_bringup, temps):
    """Simulation seulement : le lidar Gazebo voit les roues du robot."""
    return Node(
        package='laser_filters', executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain', output='screen',
        parameters=[os.path.join(dossier_bringup, 'config', 'laser_filter_params.yaml'),
                    temps],
        remappings=[('scan', '/scan'),
                    ('scan_filtered', configuration.navigation.scan_topic)])


def _pile_navigation(configuration, fichier_nav2, dossier_bringup, dossier_course, temps,
                     mode):
    vitesse = configuration.navigation.securite.topic_entree
    sans_carte = configuration.navigation.mode == 'sans_carte'
    arbre = os.path.join(
        dossier_bringup, 'behavior_trees', 'navigate_bounded_recovery.xml')
    noeuds = [] if sans_carte else [
        Node(package='slam_toolbox', executable=executable_slam(configuration, mode),
             name='slam_toolbox', output='screen',
             parameters=[os.path.join(dossier_course, 'config', 'slam_toolbox.yaml'),
                          parametres_slam(configuration, mode)])]
    noeuds += [
        Node(package='nav2_controller', executable='controller_server',
             output='screen', parameters=[fichier_nav2, temps],
             remappings=[('cmd_vel', vitesse)]),
        Node(package='nav2_planner', executable='planner_server',
             output='screen', parameters=[fichier_nav2, temps]),
        Node(package='nav2_behaviors', executable='behavior_server',
             output='screen', parameters=[fichier_nav2, temps],
             remappings=[('cmd_vel', vitesse)]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             output='screen', parameters=[
                 fichier_nav2, temps, {'default_nav_to_pose_bt_xml': arbre}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_course', output='screen',
             parameters=[temps, {
                 'autostart': True,
                  'node_names': [nom for nom in NOEUDS_GERES
                                 if not (sans_carte and nom == 'slam_toolbox')],
                 # Pas de bond : slam_toolbox n'en publie pas sous ce manager.
                 'bond_timeout': 0.0,
             }]),
    ]
    return noeuds


def _perception_et_course(configuration, temps, mode):
    noeuds = [
        Node(package='caytu_perception', executable='tag_detector',
             name=f'tag_detector_{camera.name}', output='screen',
             respawn=True, respawn_delay=2.0,
             parameters=[temps, {
                 'camera_name': camera.name,
                 'image_topic': camera.color_topic,
                 'camera_info_topic': camera.info_topic,
             }])
        for camera in configuration.perception.apriltag.cameras]
    # Profondeur -> obstacles hauts (plateaux de table) pour les costmaps.
    noeuds += [
        Node(package='caytu_perception', executable='depth_scan',
             name=f'scan_profondeur_{scan.name}', output='screen',
             respawn=True, respawn_delay=2.0,
             parameters=[temps, {'source_name': scan.name}])
        for scan in configuration.perception.depth_scans]
    noeuds.append(Node(
        package='caytu_course', executable='garde_vitesse', output='screen',
        parameters=[temps, {'mode': mode}]))
    if configuration.start_word.enabled and mode == 'course':
        noeuds.append(Node(
            package='caytu_course', executable='mot_depart', output='screen',
            respawn=True, respawn_delay=2.0, parameters=[temps]))
    if mode != 'cartographie':
        course = Node(
            package='caytu_course', executable='course_parc', output='screen',
            emulate_tty=True, parameters=[temps, {'mode': mode}])
        noeuds += [course, RegisterEventHandler(OnProcessExit(
            target_action=course,
            on_exit=[EmitEvent(event=Shutdown(reason='Le nœud de course est arrêté.'))]))]
    return noeuds


def _generer(context):
    configuration = load_config()
    temps = {'use_sim_time': configuration.system.use_sim_time}
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in MODES:
        raise RuntimeError(f'mode inconnu : {mode} (attendu : {", ".join(MODES)})')
    if configuration.navigation.mode == 'sans_carte' and mode != 'course':
        raise RuntimeError('Le mode sans carte sert au match ; préparation : FOSA_NAVIGATION=carte.')
    dossier_bringup = get_package_share_directory('caytu_nav_bringup')
    dossier_course = get_package_share_directory('caytu_course')
    fichier_nav2 = str(ecrire_parametres_course(
        configuration,
        Path(tempfile.mkdtemp(prefix='fosa_course_')) / 'nav2_course.yaml'))

    noeuds = []
    if configuration.navigation.laser_self_filter:
        noeuds.append(_filtre_laser(configuration, dossier_bringup, temps))
    noeuds += _pile_navigation(
        configuration, fichier_nav2, dossier_bringup, dossier_course, temps, mode)
    noeuds += _perception_et_course(configuration, temps, mode)
    return noeuds


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='course',
            description='course : le match ; preparation : carte et tags '
                        'enregistrés ; preparation_manuelle : carte et tags '
                        'pendant un guidage humain ; cartographie : sans mission.'),
        OpaqueFunction(function=_generer),
    ])
