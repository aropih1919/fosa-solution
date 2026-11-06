#!/usr/bin/env python3
"""Mesures du robot réel pour le diagnostic : caméras, audio, TF, scans.

Affiche, pour chaque caméra de la configuration, la résolution et le repère ;
la hauteur et l'orientation des caméras et du lidar dans base_frame (la caméra
la plus haute voit les tags à 1 m) ; le débit audio et la fréquence
d'échantillonnage estimée ; la présence de base_footprint.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from caytu_config import load_config

DUREE = 4.0


class Mesures(Node):

    def __init__(self, configuration):
        super().__init__('mesures_fosa')
        self.configuration = configuration
        self.infos = {}
        self.scans = {}
        self.audio = {}
        self.tf = Buffer()
        TransformListener(self.tf, self, spin_thread=False)
        for camera in configuration.perception.apriltag.cameras:
            self.create_subscription(
                CameraInfo, camera.info_topic,
                lambda m, nom=camera.name: self.infos.setdefault(nom, m),
                qos_profile_sensor_data)
        for source in configuration.navigation.obstacle_sources:
            self.create_subscription(
                LaserScan, source.topic,
                lambda m, nom=source.name: self.scans.setdefault(nom, m),
                qos_profile_sensor_data)
        try:
            from rosidl_runtime_py.utilities import get_message
            type_audio = get_message('audio_common_msgs/msg/AudioData')
            for topic in ('/audio', configuration.start_word.audio_topic):
                self.audio.setdefault(topic, [])
                self.create_subscription(
                    type_audio, topic,
                    lambda m, t=topic: self.audio[t].append(
                        (time.monotonic(), len(m.data))), 50)
        except (AttributeError, ModuleNotFoundError, ValueError) as erreur:
            print(f'  [ÉCHEC] audio_common_msgs indisponible : {erreur}')

    def position(self, cible, source):
        try:
            t = self.tf.lookup_transform(cible, source, Time()).transform
        except TransformException:
            return None
        q = t.rotation
        tangage = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        lacet = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        return t.translation, math.degrees(tangage), math.degrees(lacet)

    def axe_optique(self, cible, source):
        """Direction de l'axe z (visée) du repère optique dans `cible`."""
        try:
            q = self.tf.lookup_transform(cible, source, Time()).transform.rotation
        except TransformException:
            return None
        return (2 * (q.x * q.z + q.w * q.y), 2 * (q.y * q.z - q.w * q.x),
                1 - 2 * (q.x * q.x + q.y * q.y))


def main():
    rclpy.init()
    noeud = Mesures(load_config())
    fin = time.monotonic() + DUREE
    while time.monotonic() < fin:
        rclpy.spin_once(noeud, timeout_sec=0.05)
    base = noeud.configuration.system.base_frame

    print('\n== Caméras (camera_info)')
    for nom, info in sorted(noeud.infos.items()):
        print(f'  {nom}: {info.width}x{info.height}, fx={info.k[0]:.1f}, '
              f'modèle {info.distortion_model}, repère {info.header.frame_id}')
        mesure = noeud.position(base, info.header.frame_id)
        if mesure:
            p, tangage, lacet = mesure
            print(f'     dans {base} : x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}')
        axe = noeud.axe_optique(base, info.header.frame_id)
        if axe:
            pente = math.degrees(math.asin(max(-1.0, min(1.0, axe[2]))))
            print(f'     visée dans {base} : ({axe[0]:+.2f}, {axe[1]:+.2f}, {axe[2]:+.2f}), '
                  f'pente {pente:+.0f}°'
                  + ('' if axe[0] > 0.7 else
                     f'  [À VÉRIFIER] l\'axe x de {base} n\'est pas la direction de visée'))
    for camera in noeud.configuration.perception.apriltag.cameras:
        if camera.name not in noeud.infos:
            print(f'  [ÉCHEC] {camera.name} : aucune calibration reçue')

    print('\n== Scans')
    for nom, scan in sorted(noeud.scans.items()):
        valides = sum(1 for r in scan.ranges if scan.range_min <= r <= scan.range_max)
        print(f'  {nom}: {len(scan.ranges)} rayons ({valides} valides), '
              f'[{scan.angle_min:.2f}, {scan.angle_max:.2f}] rad, '
              f'portée {scan.range_min:.2f}-{scan.range_max:.2f} m, repère {scan.header.frame_id}')
        mesure = noeud.position(base, scan.header.frame_id)
        if mesure:
            p, tangage, lacet = mesure
            print(f'     dans {base} : x={p.x:.3f} y={p.y:.3f} z={p.z:.3f} '
                  f'tangage={tangage:.1f}° lacet={lacet:.1f}°')

    print('\n== Repères')
    for repere in ('base_footprint', 'odom', 'map'):
        present = noeud.tf.can_transform(base, repere, Time())
        print(f'  {repere} relié à {base} : {"oui" if present else "non"}')

    print('\n== Audio')
    for topic, messages in noeud.audio.items():
        if len(messages) >= 2:
            duree = messages[-1][0] - messages[0][0]
            debit = sum(taille for _, taille in messages[1:]) / max(duree, 1e-3)
            print(f'  {topic} : {len(messages)} messages, {messages[-1][1]} octets/message, '
                  f'{debit:.0f} octets/s -> {debit / 2:.0f} Hz si mono int16')
        elif messages:
            print(f'  {topic} : 1 message de {messages[0][1]} octets '
                  '(une phrase par message ? parler pendant la mesure)')
        else:
            print(f'  {topic} : aucun message (pour /speech_audio, parler pendant la mesure)')
    noeud.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
