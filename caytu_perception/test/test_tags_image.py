"""Tests avec le vrai OpenCV ; lancer séparément des anciens tests mockant cv2."""

import importlib
import sys
import types
from unittest.mock import Mock

import cv2
import pytest

from caytu_config import load_config
from caytu_perception.tags_image import DetecteurImage, generer_scenes


def test_ids_sur_scenes_generees():
    reglages = load_config().perception.apriltag
    detecteur = DetecteurImage(reglages.family)
    scenes = generer_scenes(detecteur.dictionnaire, reglages.test_ids)
    assert reglages.test_ids
    for nom, (image, attendus) in scenes.items():
        _, ids = detecteur.detecter(image)
        trouves = [] if ids is None else sorted(ids.flatten().tolist())
        assert trouves == sorted(attendus), nom


@pytest.mark.parametrize('publier_pose', [False, True])
def test_image_sans_calibration_ni_tf(monkeypatch, tmp_path, publier_pose):
    # Seules les interfaces ROS sont remplacées ; OpenCV traite une vraie image.
    interfaces = {
        'rclpy': {}, 'rclpy.node': {'Node': object},
        'rclpy.time': {'Time': Mock()},
        'rclpy.qos': {'qos_profile_sensor_data': object()},
        'rclpy.executors': {'ExternalShutdownException': RuntimeError},
        'cv_bridge': {'CvBridge': Mock()},
        'sensor_msgs': {}, 'sensor_msgs.msg': {'CameraInfo': Mock(), 'CompressedImage': Mock(),
                                                'Image': Mock()},
        'tf2_ros': {'Buffer': Mock(), 'TransformException': RuntimeError,
                    'TransformListener': Mock()},
        'caytu_interfaces': {},
        'caytu_interfaces.msg': {'TagDetection': Mock(), 'TagDetectionArray': Mock()},
    }
    for nom, attributs in interfaces.items():
        module = types.ModuleType(nom)
        module.__dict__.update(attributs)
        monkeypatch.setitem(sys.modules, nom, module)
    monkeypatch.delitem(sys.modules, 'caytu_perception.detecteur_tags', raising=False)
    module = importlib.import_module('caytu_perception.detecteur_tags')
    noeud = object.__new__(module.DetecteurTags)
    reglages = load_config().perception.apriltag
    noeud._detecteur_image = DetecteurImage(reglages.family)
    image, _ = generer_scenes(noeud._detecteur_image.dictionnaire, [7])['tag_7']
    noeud._bridge = Mock()
    noeud._image_compressee = False
    noeud._en_attente = []
    noeud._bridge.imgmsg_to_cv2.return_value = image
    noeud._camera_matrix = None
    noeud._publier_pose = publier_pose
    noeud._publisher = Mock()
    noeud._dossier_images = tmp_path
    noeud._nombre_images = 0
    noeud._limite_images = 1
    noeud._dernier_enregistrement = None
    noeud._intervalle = 1.0
    noeud.get_logger = Mock()
    message = types.SimpleNamespace(header=types.SimpleNamespace(
        stamp=types.SimpleNamespace(sec=1, nanosec=0)))
    noeud._recevoir_image(message)
    noeud._recevoir_image(message)
    fichiers = list(tmp_path.glob('*.png'))
    assert len(fichiers) == 1
    annotee = cv2.imread(str(fichiers[0]))
    assert (annotee[:, :, 1] != annotee[:, :, 0]).any()  # Contour en couleur.
    noeud._publisher.publish.assert_not_called()  # Pas de pose inventée.
    noeud._bridge.imgmsg_to_cv2.side_effect = ValueError('Image invalide')
    noeud._recevoir_image(message)
    noeud.get_logger.return_value.warning.assert_called_once()
