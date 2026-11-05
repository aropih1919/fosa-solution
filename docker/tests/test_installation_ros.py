"""Vérifications de l'installation réelle après colcon, sans lancer la course."""

import importlib
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor

from caytu_interfaces.msg import TagDetectionArray


@pytest.mark.parametrize('module', [
    'caytu_course.noeud_course',
    'caytu_perception.detecteur_tags',
    'caytu_perception.noeud_scan_profondeur',
    'caytu_nav_solution.navigation_runtime',
])
def test_les_modules_ros_sont_importables(module):
    importlib.import_module(module)


def test_les_ressources_de_lancement_sont_installees():
    course = Path(get_package_share_directory('caytu_course'))
    bringup = Path(get_package_share_directory('caytu_nav_bringup'))
    assert (course / 'launch/course.launch.py').is_file()
    assert (course / 'config/slam_toolbox.yaml').is_file()
    assert (bringup / 'config/nav2_params.yaml').is_file()
    ET.parse(bringup / 'behavior_trees/navigate_bounded_recovery.xml')


def test_les_profils_installes_generent_les_parametres_nav2(monkeypatch):
    from caytu_config import load_config
    from caytu_nav_solution.parametres_nav2 import construire_parametres_course

    dossier = Path(get_package_share_directory('caytu_config')) / 'config'
    for profil in (None, dossier / 'profils/simulation.yaml'):
        monkeypatch.delenv('FOSA_CONFIG_OVERRIDE', raising=False)
        if profil:
            monkeypatch.setenv('FOSA_CONFIG_OVERRIDE', str(profil))
        configuration = load_config()
        parametres = construire_parametres_course(configuration)
        assert parametres['bt_navigator']['ros__parameters']['use_sim_time'] == bool(profil)


def test_le_launch_peut_decrire_ses_arguments():
    resultat = subprocess.run(
        ['ros2', 'launch', 'caytu_course', 'course.launch.py', '--show-args'],
        capture_output=True, text=True, timeout=30, check=False)
    assert resultat.returncode == 0, resultat.stdout + resultat.stderr
    assert 'mission' in resultat.stdout


def test_un_message_genere_transite_par_ros():
    """Exerce rclpy et la sérialisation DDS, au lieu d'un faux message Python."""
    rclpy.init()
    emetteur = rclpy.create_node('test_fosa_emetteur')
    recepteur = rclpy.create_node('test_fosa_recepteur')
    executor = SingleThreadedExecutor()
    executor.add_node(emetteur)
    executor.add_node(recepteur)
    recus = []
    publicateur = emetteur.create_publisher(TagDetectionArray, '/fosa/tests/tags', 10)
    abonnement = recepteur.create_subscription(
        TagDetectionArray, '/fosa/tests/tags', recus.append, 10)
    message = TagDetectionArray()
    message.header.frame_id = 'test_docker'
    try:
        fin = time.monotonic() + 10.0
        while not recus and time.monotonic() < fin:
            publicateur.publish(message)
            executor.spin_once(timeout_sec=0.1)
        assert recus, 'Aucun message reçu via DDS dans le conteneur.'
        assert recus[0].header.frame_id == message.header.frame_id
    finally:
        recepteur.destroy_subscription(abonnement)
        executor.shutdown()
        emetteur.destroy_node()
        recepteur.destroy_node()
        rclpy.shutdown()
