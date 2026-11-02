"""Entrée transcript texte, fichier audio ou micro avec Whisper local/API."""

from array import array
import json
import math
import os
import queue
import tempfile
import threading
import time
import urllib.request
import wave

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from caytu_config import load_config


class NoeudEntreeVocale(Node):
    def __init__(self, configuration=None):
        super().__init__('speech_input')
        self._configuration = configuration or load_config()
        self.declare_parameter('input_mode', 'microphone')
        self.declare_parameter('audio_file', '')
        self.declare_parameter(
            'transcript_topic', self._configuration.dialogue.transcript_topic)
        self._publisher = self.create_publisher(
            String, self.get_parameter('transcript_topic').value, 10)
        mode = self.get_parameter('input_mode').value
        if mode == 'audio_file':
            threading.Thread(target=self._transcribe_file, daemon=True).start()
        elif mode == 'microphone':
            threading.Thread(target=self._listen, daemon=True).start()
        elif mode == 'text':
            threading.Thread(target=self._read_text, daemon=True).start()
        else:
            raise ValueError(f'Mode STT inconnu : {mode}')

    def _publier_texte(self, texte):
        """Publie le texte transcrit sur le topic ROS `/assistant/transcript`."""
        if texte:
            self._publisher.publish(String(data=texte.strip()))

    def _transcrire_audio(self, chemin_audio):
        """Transcrit un enregistrement WAV en texte via Whisper local ou l'API."""
        stt = self._configuration.dialogue.stt
        fournisseurs = {
            'local': ['local'],
            'api': ['api'],
            'auto': ['local', 'api'],
        }
        erreurs = []
        for fournisseur in fournisseurs.get(stt.provider, ['local']):
            try:
                if fournisseur == 'local':
                    from faster_whisper import WhisperModel
                    modele = WhisperModel(
                        stt.local_model, device=stt.local_device,
                        compute_type=stt.local_compute_type)
                    segments, _ = modele.transcribe(chemin_audio)
                    return ' '.join(segment.text.strip() for segment in segments).strip()
                return self._transcrire_api(chemin_audio)
            except Exception as erreur:
                erreurs.append(f'{fournisseur}: {erreur}')
        raise RuntimeError('; '.join(erreurs))

    # Alias de compatibilité
    _publish = _publier_texte
    _transcribe = _transcrire_audio

    def _transcrire_api(self, chemin_audio):
        """Transcrit le fichier audio via une requête POST multipart vers l'API Whisper distante."""
        token = (self._configuration.secrets.fosa_stt_api_key
                 or self._configuration.secrets.openai_api_key)
        if not token:
            raise RuntimeError('FOSA_STT_API_KEY ou OPENAI_API_KEY absent.')
        boundary = f'fosa-{int(time.time() * 1000)}'
        with open(chemin_audio, 'rb') as stream:
            audio_data = stream.read()
        parts = [
            f'--{boundary}\r\nContent-Disposition: form-data; name="model"\r\n\r\n'
            f'{self._configuration.dialogue.stt.api_model}\r\n'.encode(),
            f'--{boundary}\r\nContent-Disposition: form-data; name="file"; '
            f'filename="transcript.wav"\r\nContent-Type: audio/wav\r\n\r\n'.encode(),
            audio_data,
            f'\r\n--{boundary}--\r\n'.encode(),
        ]
        request = urllib.request.Request(
            self._configuration.dialogue.stt.api_base.rstrip('/')
            + '/audio/transcriptions',
            data=b''.join(parts),
            headers={
                'Authorization': f'Bearer {token}',
                'Content-Type': f'multipart/form-data; boundary={boundary}',
            })
        with urllib.request.urlopen(
                request,
                timeout=self._configuration.dialogue.stt.timeout_sec) as response:
            return json.loads(response.read().decode('utf-8'))['text']

    def _transcrire_fichier(self):
        """Lit le fichier audio spécifié en paramètre et publie sa transcription."""
        chemin = self.get_parameter('audio_file').value
        if not chemin:
            self.get_logger().error('Le paramètre audio_file est obligatoire.')
            return
        try:
            self._publier_texte(self._transcrire_audio(chemin))
        except Exception as error:
            self.get_logger().error(f'Échec de transcription : {error}')

    def _lire_texte(self):
        """Lit les phrases tapées au clavier dans le terminal pour les tests sans micro."""
        while rclpy.ok():
            try:
                self._publier_texte(input('Vous> '))
            except EOFError:
                return

    def _ecouter_micro(self):
        """Boucle d'écoute micro avec détection de silence VAD pour isoler les phrases."""
        try:
            import sounddevice as sd
        except ImportError:
            self.get_logger().error(
                'sounddevice absent; installez la dépendance micro ou utilisez '
                'input_mode:=audio_file.')
            return

        audio = self._configuration.audio
        block_frames = max(1, int(audio.sample_rate * audio.vad_frame_ms / 1000.0))
        silence_blocks = max(1, math.ceil(audio.silence_ms / audio.vad_frame_ms))
        threshold = audio.vad_threshold * audio.vad_rms_scale
        queue_audio = queue.Queue()

        def recevoir(indata, frames, time_info, status):
            del frames, time_info
            if status:
                self.get_logger().warning(str(status))
            queue_audio.put(bytes(indata))

        device = None if audio.input_device == 'default' else audio.input_device
        try:
            with sd.RawInputStream(
                    samplerate=audio.sample_rate, blocksize=block_frames,
                    channels=1, dtype='int16', device=device,
                    callback=recevoir):
                actif = False
                silence = 0
                morceaux = []
                while rclpy.ok():
                    morceau = queue_audio.get()
                    samples = array('h')
                    samples.frombytes(morceau)
                    rms = (math.sqrt(sum(sample * sample for sample in samples)
                                     / max(1, len(samples))) / 32768.0)
                    if rms >= threshold:
                        actif = True
                        silence = 0
                        morceaux.append(morceau)
                    elif actif:
                        morceaux.append(morceau)
                        silence += 1
                        if silence >= silence_blocks:
                            self._transcrire_morceaux_audio(morceaux, audio.sample_rate)
                            actif, silence, morceaux = False, 0, []
        except Exception as error:
            self.get_logger().error(f'Échec du micro : {error}')

    def _transcrire_morceaux_audio(self, morceaux, frequence):
        """Écrit les morceaux audio PCM dans un fichier WAV temporaire et déclenche Whisper."""
        if not morceaux:
            return
        chemin = None
        try:
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as stream:
                chemin = stream.name
            with wave.open(chemin, 'wb') as wav:
                wav.setnchannels(1)
                wav.setsampwidth(2)
                wav.setframerate(frequence)
                wav.writeframes(b''.join(morceaux))
            self._publier_texte(self._transcrire_audio(chemin))
        except Exception as error:
            self.get_logger().error(f'Échec de transcription micro : {error}')
        finally:
            if chemin and os.path.exists(chemin):
                os.unlink(chemin)

    # Alias de compatibilité
    _transcribe_api = _transcrire_api
    _transcribe_file = _transcrire_fichier
    _read_text = _lire_texte
    _listen = _ecouter_micro
    _transcribe_chunks = _transcrire_morceaux_audio


def main(args=None):
    rclpy.init(args=args)
    node = NoeudEntreeVocale()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
