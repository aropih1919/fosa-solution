"""Démarrage vocal de la course : mot « allez » (ou « go », « vai »), hors ligne.

Bonus PARC (+5) : la course démarre quand le robot entend le mot de départ.
La reconnaissance tourne sur le robot, sans réseau (faster-whisper, modèle
installé pendant les essais). Le robot ne doit jamais partir sur un bruit du
public : on n'accepte qu'un segment court, assez fort, transcrit en au plus
trois mots dont l'un est le mot de départ, avec une transcription confiante.

Entrée audio (caytu_config, course.voice.audio_topic) :
  /speech_audio  segments de parole du nœud ReSpeaker des organisateurs
                 (audio_common_msgs/AudioData, PCM 16 bits) ;
  ''             micro local avec sounddevice (essais sur un PC).
Sortie : std_msgs/String (le mot reconnu, ex. « allez ») sur course.voice.start_topic,
une seule fois. course_tags prévient alors l'arbitre PARC d'un départ vocal (bonus).
"""

import math
import re
import threading
import time
import unicodedata

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from caytu_config import load_config

# Variantes de transcription admises pour chaque mot de départ (sans accents).
VARIANTES = {
    'allez': {'allez', 'aller', 'alle', 'allez-y', 'alez'},
    'go': {'go'},
    'vai': {'vai', 'vay'},
}


def normaliser_texte(texte):
    decompose = unicodedata.normalize('NFKD', texte)
    sans_accents = ''.join(c for c in decompose if not unicodedata.combining(c))
    return re.findall(r"[a-z]+(?:-[a-z]+)?", sans_accents.casefold())


def trouver_mot_depart(texte, mots_depart, mots_max):
    """Mot de départ reconnu (forme officielle, ex. « allez »), ou None.

    La transcription doit être courte (mots_max mots au plus).
    """
    mots = normaliser_texte(texte)
    if not mots or len(mots) > mots_max:
        return None
    for mot_officiel in mots_depart:
        admis = VARIANTES.get(mot_officiel, {mot_officiel})
        if any(mot in admis for mot in mots):
            return mot_officiel
    return None


def est_mot_depart(texte, mots_depart, mots_max):
    """Vrai si la transcription est courte et contient un mot de départ."""
    return trouver_mot_depart(texte, mots_depart, mots_max) is not None


def pcm16_vers_float(donnees, frequence, frequence_cible=16000):
    """PCM 16 bits petit-boutiste -> float32 à 16 kHz (attendu par Whisper)."""
    signal_audio = np.frombuffer(bytes(donnees), dtype='<i2').astype(np.float32) / 32768.0
    if frequence != frequence_cible and len(signal_audio) > 1:
        duree = len(signal_audio) / float(frequence)
        temps_cible = np.arange(0.0, duree, 1.0 / frequence_cible)
        signal_audio = np.interp(
            temps_cible, np.arange(len(signal_audio)) / float(frequence), signal_audio
        ).astype(np.float32)
    return signal_audio


def niveau_rms(signal_audio):
    if len(signal_audio) == 0:
        return 0.0
    return float(math.sqrt(float(np.mean(np.square(signal_audio)))))


class TranscripteurWhisper:
    """faster-whisper local, chargé une fois au démarrage (jamais en course)."""

    def __init__(self, voix):
        from faster_whisper import WhisperModel
        self._langue = voix.language
        self._modele = WhisperModel(voix.model, device=voix.device,
                                    compute_type=voix.compute_type)

    def transcrire(self, signal_audio):
        segments, _ = self._modele.transcribe(
            signal_audio, language=self._langue, beam_size=1,
            condition_on_previous_text=False, vad_filter=False)
        textes = []
        for segment in segments:
            # Segment douteux (bruit, hallucination) : on n'en tient pas compte.
            if segment.no_speech_prob > 0.6 or segment.avg_logprob < -1.0:
                continue
            textes.append(segment.text.strip())
        return ' '.join(textes).strip()


class NoeudMotDepart(Node):

    def __init__(self, configuration=None, transcripteur=None):
        super().__init__('mot_depart')
        self._configuration = configuration or load_config()
        self._voix = self._configuration.course.voice
        self.declare_parameter('audio_topic', self._voix.audio_topic)
        self._publicateur = self.create_publisher(
            String, self._voix.start_topic, 10)
        self._depart_donne = False
        # Le mot reconnu est répété jusqu'à l'accusé de course_tags (verrouillé),
        # 30 s au plus : sur un PC chargé, un envoi unique peut se perdre.
        self._mot_envoye = None
        self._fin_repetition = 0.0
        self._accuse = False
        verrouille = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, self._configuration.course.start_ack_topic,
                                 self._recevoir_accuse, verrouille)
        self.create_timer(0.3, self._repeter_mot)
        self._verrou = threading.Lock()
        debut = time.monotonic()
        self._transcripteur = transcripteur or TranscripteurWhisper(self._voix)
        self.get_logger().info(
            f'Modèle Whisper « {self._voix.model} » prêt en {time.monotonic() - debut:.1f} s. '
            f'Mots de départ : {", ".join(self._voix.words)}.')

        topic = str(self.get_parameter('audio_topic').value or '')
        if topic:
            from audio_common_msgs.msg import AudioData
            self.create_subscription(AudioData, topic, self.recevoir_segment, 10)
            self.get_logger().info(f'Écoute des segments de parole sur {topic}.')
        else:
            threading.Thread(target=self._ecouter_micro, daemon=True).start()

    # -------------------------------------------------------------- entrées
    def recevoir_segment(self, message):
        self.examiner(pcm16_vers_float(message.data, self._voix.sample_rate))

    def _ecouter_micro(self):
        """Micro local : découpe en segments par niveau sonore (essais sur PC)."""
        import sounddevice
        frequence, bloc = 16000, 1600            # blocs de 0,1 s
        segment, silence = [], 0
        with sounddevice.InputStream(samplerate=frequence, channels=1, dtype='float32',
                                     blocksize=bloc) as flux:
            while rclpy.ok() and not self._depart_donne:
                donnees, _ = flux.read(bloc)
                donnees = donnees[:, 0].copy()
                if niveau_rms(donnees) >= self._voix.min_rms:
                    segment.append(donnees)
                    silence = 0
                elif segment:
                    silence += 1
                    segment.append(donnees)
                    if silence >= 4:             # 0,4 s de silence : fin du mot
                        self.examiner(np.concatenate(segment))
                        segment, silence = [], 0
                if len(segment) * bloc > self._voix.max_segment_sec * frequence:
                    segment, silence = [], 0

    # ------------------------------------------------------------ décision
    def examiner(self, signal_audio):
        """Transcrit un segment et donne le départ si c'est le mot attendu."""
        if self._depart_donne:
            return False
        duree = len(signal_audio) / 16000.0
        rms = niveau_rms(signal_audio)
        if not (self._voix.min_segment_sec <= duree <= self._voix.max_segment_sec):
            return False
        if rms < self._voix.min_rms:
            return False
        with self._verrou:
            texte = self._transcripteur.transcrire(signal_audio)
        self.get_logger().info(f'Entendu : « {texte} » ({duree:.1f} s).')
        mot = trouver_mot_depart(texte, self._voix.words, self._voix.max_words)
        if mot is None:
            return False
        self._depart_donne = True
        self._mot_envoye = mot
        self._fin_repetition = time.monotonic() + 30.0
        self._publicateur.publish(String(data=mot))
        self.get_logger().info(f'MOT DE DÉPART reconnu (« {mot} ») : départ de la course.')
        return True

    def _recevoir_accuse(self, message):
        if not self._accuse:
            self._accuse = True
            self.get_logger().info(f'La course a reçu le départ ({message.data}).')

    def _repeter_mot(self):
        if self._mot_envoye is None or self._accuse:
            return
        if time.monotonic() > self._fin_repetition:
            self.get_logger().error(
                'Aucun accusé de la course en 30 s : course_tags est-il lancé ?')
            self._mot_envoye = None
            return
        self._publicateur.publish(String(data=self._mot_envoye))


def main(args=None):
    rclpy.init(args=args)
    configuration = load_config()
    if not configuration.course.voice.enabled:
        print('mot_depart : démarrage vocal désactivé (course.voice.enabled).')
        return
    noeud = NoeudMotDepart(configuration)
    try:
        rclpy.spin(noeud)
    except KeyboardInterrupt:
        pass
    finally:
        noeud.destroy_node()
        rclpy.try_shutdown()
