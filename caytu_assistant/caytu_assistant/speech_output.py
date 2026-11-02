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
            self._speak, 10)

    def _speak(self, message):
        text = message.data.strip()
        if not text:
            return
        tts = self._configuration.dialogue.tts
        providers = {'local': ['local'], 'api': ['api'], 'auto': ['local', 'api']}
        erreurs = []
        for provider in providers[tts.provider]:
            try:
                if provider == 'local':
                    self._speak_local(text)
                else:
                    self._speak_api(text)
                return
            except Exception as error:
                erreurs.append(f'{provider}: {error}')
        self.get_logger().warning(
            f'TTS indisponible ({"; ".join(erreurs)}). Texte : {text}')

    def _speak_local(self, text):
        voice = self._configuration.dialogue.tts.local_voice
        model = os.path.expanduser(voice)
        if not os.path.isabs(model):
            model = str(config_directory() / model)
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as output:
            wav_path = output.name
        try:
            subprocess.run(
                [self._configuration.audio.piper_executable,
                 '--model', model, '--output_file', wav_path],
                input=text.encode('utf-8'), check=True,
                timeout=self._configuration.dialogue.tts.timeout_sec)
            device = self._configuration.audio.output_device
            command = [self._configuration.audio.audio_player]
            if device != 'default':
                command.extend(['-D', device])
            command.append(wav_path)
            subprocess.run(
                command, check=True,
                timeout=self._configuration.dialogue.tts.timeout_sec)
        finally:
            if os.path.exists(wav_path):
                os.unlink(wav_path)

    def _speak_api(self, text):
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
                'input': text,
                'response_format': 'wav',
            }).encode('utf-8'),
            headers={
                'Authorization': f'Bearer {token}',
                'Content-Type': 'application/json',
            })
        with urllib.request.urlopen(
                request, timeout=tts.timeout_sec) as response:
            audio_data = response.read()
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as output:
            output.write(audio_data)
            wav_path = output.name
        try:
            device = configuration.audio.output_device
            command = [configuration.audio.audio_player]
            if device != 'default':
                command.extend(['-D', device])
            command.append(wav_path)
            subprocess.run(command, check=True, timeout=tts.timeout_sec)
        finally:
            os.unlink(wav_path)


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
