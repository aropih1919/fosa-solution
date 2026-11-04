#!/usr/bin/env python3
"""Micro du PC -> /speech_audio : départ vocal en simulation, sans robot.

Sur le robot, le micro ReSpeaker publie déjà l'audio. Ici, `arecord` lit le
micro du PC (PCM 16 bits mono) et publie des morceaux de 0,2 s au format
attendu par le nœud du mot de départ (`audio_common_msgs/AudioData`).
Micro choisi par FOSA_SIM_MICRO (défaut : « default », voir `arecord -L`).
"""

import array
import os
import signal
import subprocess
import sys

import rclpy
from rclpy.executors import ExternalShutdownException

from caytu_config import load_config

DUREE_MORCEAU = 0.2
OCTETS_PAR_ECHANTILLON = 2


def main():
    try:
        from audio_common_msgs.msg import AudioData
    except ImportError:
        sys.exit('audio_common_msgs absent : sudo apt install ros-jazzy-audio-common-msgs')
    voix = load_config().start_word
    rclpy.init()
    noeud = rclpy.create_node('micro_pc')
    publicateur = noeud.create_publisher(AudioData, voix.audio_topic, 20)
    micro = os.environ.get('FOSA_SIM_MICRO', 'default')
    enregistreur = subprocess.Popen(
        ['arecord', '-q', '-D', micro, '-f', 'S16_LE', '-r', str(voix.sample_rate),
         '-c', '1', '-t', 'raw'], stdout=subprocess.PIPE)
    taille = int(voix.sample_rate * DUREE_MORCEAU) * OCTETS_PAR_ECHANTILLON
    print(f'Micro du PC ({micro}) publié sur {voix.audio_topic} à {voix.sample_rate} Hz.',
          flush=True)
    try:
        while rclpy.ok():
            morceau = enregistreur.stdout.read(taille)
            if not morceau:
                noeud.get_logger().error('Micro muet : arecord s\'est arrêté (voir arecord -L).')
                break
            publicateur.publish(AudioData(data=array.array('B', morceau)))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        enregistreur.send_signal(signal.SIGINT)
        noeud.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
