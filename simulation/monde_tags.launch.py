"""Monde officiel PARC sans interface graphique, robot et 4 AprilTags.

Variables d'environnement :
  FOSA_SIM_TAGS            dossier des modèles générés par generer_tags.py
  FOSA_SIM_ODOM_CORRIGEE=1 écartement des roues du plugin DiffDrive ramené à
                           l'écart réel du modèle (0,393 m au lieu de 0,3409) :
                           sinon l'odométrie surestime les rotations de 15 %.
  FOSA_SIM_GUI=1           afficher Gazebo (plus lent)
"""

import math
import os

import yaml
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

# Position (x, y) dans le monde et direction de la face du tag (rad).
TAGS = {7: (-2.3, -7.3, 0.0), 1: (-1.0, -9.9, math.pi / 2),
        9: (1.0, -6.2, -math.pi / 2), 4: (2.2, -8.3, math.pi)}
ECART_ROUES_OFFICIEL = '<wheel_separation>0.3409</wheel_separation>'
ECART_ROUES_REEL = '<wheel_separation>0.393</wheel_separation>'


def _description(dossier_description):
    urdf = xacro.process_file(
        os.path.join(dossier_description, 'urdf', 'sitoe_robot.urdf.xacro'),
        mappings={'sim_mode': 'true'}).toxml()
    if os.environ.get('FOSA_SIM_ODOM_CORRIGEE') == '1':
        urdf = urdf.replace(ECART_ROUES_OFFICIEL, ECART_ROUES_REEL)
    return urdf


def generate_launch_description():
    bringup = FindPackageShare('parc_robot_bringup').find('parc_robot_bringup')
    description = FindPackageShare('parc_robot_description').find('parc_robot_description')
    gz = FindPackageShare('ros_gz_sim').find('ros_gz_sim')
    with open(os.path.join(bringup, 'config', 'task_params.yaml')) as fichier:
        depart = yaml.safe_load(fichier)['/**']['ros__parameters']
    monde = os.path.join(bringup, 'worlds', 'task_world.sdf')
    options = '-r -v1' if os.environ.get('FOSA_SIM_GUI') == '1' else '-r -s --headless-rendering -v1'
    dossier_tags = os.path.expanduser(
        os.environ.get('FOSA_SIM_TAGS', '~/.fosa_simulation/tags'))

    actions = [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(bringup, 'models')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': f'{options} {monde}',
                              'on_exit_shutdown': 'true'}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'use_sim_time': True,
                          'robot_description': _description(description)}]),
        Node(package='ros_gz_sim', executable='create', output='screen', arguments=[
            '-topic', 'robot_description', '-name', 'sitoe_robot',
            '-x', str(depart['x']), '-y', str(depart['y']), '-z', str(depart['z']),
            '-Y', str(depart['yaw'])]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=['--ros-args', '-p',
                        f"config_file:={os.path.join(bringup, 'config', 'gz_bridge.yaml')}"]),
        Node(package='ros_gz_image', namespace='top_camera', executable='image_bridge',
             arguments=['/top_camera_color/image_raw'], output='screen'),
    ]
    tags = [
        Node(package='ros_gz_sim', executable='create', output='screen', arguments=[
            '-file', os.path.join(dossier_tags, f'tag{tag}', 'model.sdf'),
            '-name', f'tag{tag}', '-x', str(x), '-y', str(y), '-z', '0.0',
            '-Y', str(cap)])
        for tag, (x, y, cap) in TAGS.items()]
    actions.append(TimerAction(period=8.0, actions=tags))
    return LaunchDescription(actions)
