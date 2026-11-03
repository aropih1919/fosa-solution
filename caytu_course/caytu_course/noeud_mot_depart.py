#!/usr/bin/env python3
"""Départ vocal hors ligne : Vosk sur l'audio du ReSpeaker publié par le robot.

Le micro est déjà ouvert par `respeaker_node` : on lit son topic
(`audio_common_msgs/AudioData`, octets PCM) au lieu d'ouvrir le périphérique.
Le nœud n'écoute que lorsque la course attend le départ (ordre reçu) et ne
publie le départ qu'une seule fois. Sans Vosk, sans modèle ou sans paquet
audio, il le signale et la course se lance par commande.
"""

import json
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from caytu_config import load_config
from caytu_config.loader import config_directory
from caytu_course.mot_depart import est_mot_depart, extraire_canal, grammaire
from caytu_course.statut import qos_statut

TYPE_AUDIO = 'audio_common_msgs/msg/AudioData'
ETATS_ARMES = {'ATTENTE_DEPART'}


def _chemin_modele(valeur):
    chemin = Path(valeur).expanduser()
    return chemin if chemin.is_absolute() else config_directory() / chemin


class NoeudMotDepart(Node):

    def __init__(self, configuration):
        super().__init__('mot_depart')
        self._voix = configuration.start_word
        self._arme = False
        self._depart_publie = False
        self._modele = None
        self._flux = None
        self._publicateur = self.create_publisher(
            String, configuration.course.start_topic, 10)
        self.create_subscription(
            String, configuration.course.state_topic, self._recevoir_etat,
            qos_statut(1))
        if self._charger_vosk() and self._abonner_audio():
            self.get_logger().info(
                f'Mot de départ {list(self._voix.words)} écouté sur '
                f'{self._voix.audio_topic} ({self._voix.sample_rate} Hz).')

    def _charger_vosk(self):
        chemin = _chemin_modele(self._voix.model_path)
        try:
            from vosk import KaldiRecognizer, Model, SetLogLevel
        except ImportError:
            self.get_logger().error('Vosk absent : départ par commande uniquement.')
            return False
        if not chemin.is_dir():
            self.get_logger().error(
                f'Modèle Vosk introuvable ({chemin}) : départ par commande uniquement.')
            return False
        SetLogLevel(-1)
        self._modele = Model(str(chemin))
        self._reconnaisseur = KaldiRecognizer
        return True

    def _abonner_audio(self):
        try:
            from rosidl_runtime_py.utilities import get_message
            type_audio = get_message(TYPE_AUDIO)
        except (AttributeError, ModuleNotFoundError, ValueError) as erreur:
            self.get_logger().error(
                f'{TYPE_AUDIO} indisponible ({erreur}) : départ par commande uniquement.')
            return False
        self.create_subscription(
            type_audio, self._voix.audio_topic, self._recevoir_audio, 20)
        return True

    def _nouveau_reconnaisseur(self):
        arguments = [self._modele, float(self._voix.sample_rate)]
        if self._voix.restricted_grammar:
            arguments.append(grammaire(self._voix.words))
        reconnaisseur = self._reconnaisseur(*arguments)
        reconnaisseur.SetWords(True)
        return reconnaisseur

    def _recevoir_etat(self, message):
        try:
            etat = json.loads(message.data).get('etat')
        except (ValueError, AttributeError):
            return
        arme = etat in ETATS_ARMES and not self._depart_publie
        if arme and not self._arme:
            self.get_logger().info('À l\'écoute du mot de départ.')
        if not arme:
            self._flux = None
        self._arme = arme

    def _recevoir_audio(self, message):
        if not self._arme or self._modele is None:
            return
        donnees = extraire_canal(
            bytes(message.data), self._voix.channels, self._voix.sample_width,
            self._voix.channel_index)
        if self._voix.mode == 'utterance':
            reconnaisseur = self._nouveau_reconnaisseur()
            reconnaisseur.AcceptWaveform(donnees)
            self._decider(reconnaisseur.FinalResult())
            return
        if self._flux is None:
            self._flux = self._nouveau_reconnaisseur()
        if self._flux.AcceptWaveform(donnees):
            self._decider(self._flux.Result())

    def _decider(self, resultat):
        if not est_mot_depart(resultat, self._voix.words, self._voix.min_confidence):
            try:
                texte = json.loads(resultat).get('text', '')
            except (TypeError, ValueError, AttributeError):
                texte = ''
            if texte:
                self.get_logger().info(f'Entendu « {texte} » : pas un départ.')
            return
        self._depart_publie = True
        self._arme = False
        self._publicateur.publish(String(data='voix'))
        self.get_logger().info('Mot de départ reconnu : DÉPART.')


def main(args=None):
    rclpy.init(args=args)
    noeud = NoeudMotDepart(load_config())
    try:
        rclpy.spin(noeud)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        noeud.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
