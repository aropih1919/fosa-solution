"""Synthèse locale Piper ou API, avec sortie texte en cas d’indisponibilité."""

import json
import os
import subprocess
import tempfile
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from caytu_config import load_config
from caytu_config.loader import config_directory


class NoeudSortieVocale(Node):
    def __init__(self, configuration=None):
        super().__init__('speech_output')
        self._configuration = configuration or load_config()
        self.declare_parameter(
            'response_topic', self._configuration.dialogue.response_topic)
        self._subscription = self.create_subscription(
            String, self.get_parameter('response_topic').value,
            self._parler, 10)

    def _parler(self, message):
        """Reçoit une réponse textuelle sur le topic ROS et déclenche la synthèse vocale."""
        texte = message.data.strip()
        if not texte:
            return
        tts = self._configuration.dialogue.tts
        fournisseurs = {
            'local': [self._parler_local],
            'api': [self._parler_api],
            'auto': [self._parler_local, self._parler_api],
        }
        erreurs = []
        for fournisseur in fournisseurs.get(tts.provider, [self._parler_local]):
            try:
                fournisseur(texte)
                return
            except Exception as erreur:
                erreurs.append(f'{tts.provider}: {erreur}')
        # Si la voix échoue, on ne bloque pas le robot : on journalise un avertissement
        self.get_logger().warning(
            f'TTS indisponible ({"; ".join(erreurs)}). Texte : {texte}')

    def _parler_local(self, texte):
        """Génère un fichier WAV avec Piper localement puis le joue avec le lecteur configuré (aplay)."""
        voix = self._configuration.dialogue.tts.local_voice
        modele = os.path.expanduser(voix)
        if not os.path.isabs(modele):
            modele = str(config_directory() / modele)
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as output:
            chemin_wav = output.name
        try:
            subprocess.run(
                [self._configuration.audio.piper_executable,
                 '--model', modele, '--output_file', chemin_wav],
                input=texte.encode('utf-8'), check=True,
                timeout=self._configuration.dialogue.tts.timeout_sec)
            device = self._configuration.audio.output_device
            commande = [self._configuration.audio.audio_player]
            if device != 'default':
                commande.extend(['-D', device])
            commande.append(chemin_wav)
            subprocess.run(
                commande, check=True,
                timeout=self._configuration.dialogue.tts.timeout_sec)
        finally:
            if os.path.exists(chemin_wav):
                os.unlink(chemin_wav)

    def _parler_api(self, texte):
        """Envoie le texte à un service TTS distant (OpenAI/ElevenLabs) et joue l'audio reçu."""
        configuration = self._configuration
        tts = configuration.dialogue.tts
        token = (configuration.secrets.fosa_tts_api_key
                 or configuration.secrets.openai_api_key)
        if not token:
            raise RuntimeError('FOSA_TTS_API_KEY ou OPENAI_API_KEY absent.')
        if not tts.api_voice:
            raise RuntimeError('dialogue.tts.api_voice doit être configurée.')
        request = urllib.request.Request(
            tts.api_base.rstrip('/') + '/audio/speech',
            data=json.dumps({
                'model': tts.api_model,
                'voice': tts.api_voice,
                'input': texte,
                'response_format': 'wav',
            }).encode('utf-8'),
            headers={
                'Authorization': f'Bearer {token}',
                'Content-Type': 'application/json',
            })
        with urllib.request.urlopen(
                request, timeout=tts.timeout_sec) as response:
            donnees_audio = response.read()
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as output:
            output.write(donnees_audio)
            chemin_wav = output.name
        try:
            device = configuration.audio.output_device
            commande = [configuration.audio.audio_player]
            if device != 'default':
                commande.extend(['-D', device])
            commande.append(chemin_wav)
            subprocess.run(commande, check=True, timeout=tts.timeout_sec)
        finally:
            if os.path.exists(chemin_wav):
                os.unlink(chemin_wav)

def main(args=None):
    rclpy.init(args=args)
    node = NoeudSortieVocale()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
