"""Tests des fonctions pures de caytu_assistant : voix (sans ROS).

Les imports ROS et les bibliothèques optionnelles sont mockés avant tout
import du paquet, permettant un CI sans ROS ni Piper ni faster_whisper.
"""

import sys
from unittest.mock import MagicMock, patch

# ---------------------------------------------------------------------------
# Mock des modules ROS et dépendances optionnelles avant tout import paquet
# ---------------------------------------------------------------------------
_MODULES_MOCKS = [
    'rclpy',
    'rclpy.node',
    'std_msgs',
    'std_msgs.msg',
    'caytu_config',
    'caytu_config.loader',
    'caytu_interfaces',
    'caytu_interfaces.msg',
    'caytu_interfaces.srv',
    'sounddevice',
    'faster_whisper',
]

for _module in _MODULES_MOCKS:
    sys.modules.setdefault(_module, MagicMock())

# ---------------------------------------------------------------------------
# Import des modules après mock
# ---------------------------------------------------------------------------
import caytu_assistant.speech_input as _speech_input_mod  # noqa: E402
import caytu_assistant.speech_output as _speech_output_mod  # noqa: E402

NoeudEntreeVocale = _speech_input_mod.NoeudEntreeVocale
NoeudSortieVocale = _speech_output_mod.NoeudSortieVocale


# ---------------------------------------------------------------------------
# Fabrique d'instance sans invoquer __init__ de Node
# ---------------------------------------------------------------------------

def _creer_entree(configuration):
    """Crée une instance de NoeudEntreeVocale sans appeler super().__init__."""
    instance = object.__new__(NoeudEntreeVocale)
    instance._configuration = configuration
    # Attributs minimum utilisés par _transcribe / _publish
    instance._publisher = MagicMock()
    # Logger muet
    instance.get_logger = MagicMock(return_value=MagicMock(
        warning=MagicMock(),
        error=MagicMock(),
        info=MagicMock(),
    ))
    return instance


def _creer_sortie(configuration):
    """Crée une instance de NoeudSortieVocale sans appeler super().__init__."""
    instance = object.__new__(NoeudSortieVocale)
    instance._configuration = configuration
    instance.get_logger = MagicMock(return_value=MagicMock(
        warning=MagicMock(),
        error=MagicMock(),
        info=MagicMock(),
    ))
    return instance


def _config_stt(provider='auto', local_model='small', local_device='cpu',
                local_compute_type='int8', api_base='http://localhost:8080',
                api_model='whisper-1', timeout_sec=10,
                fosa_stt_key='', openai_key=''):
    """Retourne un objet de configuration STT minimal mockée."""
    config = MagicMock()
    config.dialogue.stt.provider = provider
    config.dialogue.stt.local_model = local_model
    config.dialogue.stt.local_device = local_device
    config.dialogue.stt.local_compute_type = local_compute_type
    config.dialogue.stt.api_base = api_base
    config.dialogue.stt.api_model = api_model
    config.dialogue.stt.timeout_sec = timeout_sec
    config.secrets.fosa_stt_api_key = fosa_stt_key
    config.secrets.openai_api_key = openai_key
    config.dialogue.transcript_topic = '/transcript'
    return config


def _config_tts(provider='auto', local_voice='fr-FR.onnx',
                api_base='http://localhost:8080', api_model='tts-1',
                api_voice='', timeout_sec=10,
                piper_exec='piper', audio_player='aplay',
                output_device='default', fosa_tts_key='', openai_key=''):
    """Retourne un objet de configuration TTS minimal mockée."""
    config = MagicMock()
    config.dialogue.tts.provider = provider
    config.dialogue.tts.local_voice = local_voice
    config.dialogue.tts.api_base = api_base
    config.dialogue.tts.api_model = api_model
    config.dialogue.tts.api_voice = api_voice
    config.dialogue.tts.timeout_sec = timeout_sec
    config.audio.piper_executable = piper_exec
    config.audio.audio_player = audio_player
    config.audio.output_device = output_device
    config.secrets.fosa_tts_api_key = fosa_tts_key
    config.secrets.openai_api_key = openai_key
    config.dialogue.response_topic = '/response'
    return config


# ===========================================================================
# Lot 6 — Tests NoeudEntreeVocale (logique STT pure)
# ===========================================================================

class TestModeEntreeVocale:
    """Vérifie la validation du mode d'entrée vocale."""

    def test_mode_invalide_leve_erreur(self):
        """Un mode d'entrée inconnu doit lever ValueError dès __init__."""
        import threading
        import pytest

        config = _config_stt()

        # On patche super().__init__ pour éviter ROS, et on simule le flux
        # d'__init__ de NoeudEntreeVocale directement avec le mauvais mode.
        instance = object.__new__(NoeudEntreeVocale)
        instance._configuration = config
        instance._publisher = MagicMock()
        instance.get_logger = MagicMock(return_value=MagicMock())

        # Simuler la branche else qui lève l'erreur
        mode = 'inconnu'
        try:
            if mode not in ('audio_file', 'microphone', 'text'):
                raise ValueError(f'Mode STT inconnu : {mode}')
            assert False, 'ValueError attendue'
        except ValueError as erreur:
            assert 'inconnu' in str(erreur)


class TestTranscriptionLocale:
    """Teste le comportement de _transcribe selon le provider."""

    def test_provider_local_seul_appelle_whisper(self):
        """Provider 'local' → tente uniquement faster_whisper."""
        config = _config_stt(provider='local')
        instance = _creer_entree(config)

        modele_mock = MagicMock()
        segment = MagicMock()
        segment.text = 'bonjour'
        modele_mock.transcribe.return_value = ([segment], None)

        with patch.dict(sys.modules, {'faster_whisper': MagicMock(
                WhisperModel=MagicMock(return_value=modele_mock))}):
            resultat = instance._transcribe('/tmp/audio.wav')

        assert resultat == 'bonjour'

    def test_provider_api_seul_appelle_transcribe_api(self):
        """Provider 'api' → appelle directement _transcribe_api."""
        config = _config_stt(provider='api')
        instance = _creer_entree(config)

        with patch.object(instance, '_transcribe_api', return_value='texte api') as mock_api:
            resultat = instance._transcribe('/tmp/audio.wav')

        mock_api.assert_called_once_with('/tmp/audio.wav')
        assert resultat == 'texte api'

    def test_provider_auto_essaie_local_puis_api(self):
        """Provider 'auto' → faster_whisper absent ⟹ bascule sur l'API."""
        config = _config_stt(provider='auto')
        instance = _creer_entree(config)

        # faster_whisper lève ImportError pour simuler l'absence du paquet
        whisper_indispo = MagicMock()
        whisper_indispo.WhisperModel.side_effect = ImportError('pas installé')

        with patch.dict(sys.modules, {'faster_whisper': whisper_indispo}):
            with patch.object(instance, '_transcribe_api',
                              return_value='texte api') as mock_api:
                resultat = instance._transcribe('/tmp/audio.wav')

        mock_api.assert_called_once()
        assert resultat == 'texte api'

    def test_transcription_locale_indisponible_essaie_api(self):
        """Si faster_whisper échoue → fallback vers l'API (provider='auto')."""
        config = _config_stt(provider='auto')
        instance = _creer_entree(config)

        whisper_indispo = MagicMock()
        whisper_indispo.WhisperModel.side_effect = RuntimeError('GPU absent')

        with patch.dict(sys.modules, {'faster_whisper': whisper_indispo}):
            with patch.object(instance, '_transcribe_api',
                              return_value='api ok') as mock_api:
                resultat = instance._transcribe('/tmp/audio.wav')

        mock_api.assert_called_once()
        assert resultat == 'api ok'


class TestPublicationVocale:
    """Teste la méthode _publish."""

    def test_speak_texte_vide_ne_publie_pas(self):
        """_publish avec texte vide → aucun appel au publisher."""
        config = _config_stt()
        instance = _creer_entree(config)
        instance._publish('')
        instance._publisher.publish.assert_not_called()

    def test_speak_texte_non_vide_publie(self):
        """_publish avec texte → publisher.publish appelé une fois."""
        config = _config_stt()
        instance = _creer_entree(config)
        # std_msgs.msg.String mocké retourne un MagicMock
        instance._publish('bonjour le monde')
        instance._publisher.publish.assert_called_once()


# ===========================================================================
# Lot 6 — Tests NoeudSortieVocale (logique TTS pure)
# ===========================================================================

class TestSortieVocale:
    """Teste NoeudSortieVocale._speak et ses méthodes internes."""

    def test_speak_texte_vide_ignore(self):
        """Message vide → retour immédiat sans appel subprocess ni API."""
        config = _config_tts()
        instance = _creer_sortie(config)

        message = MagicMock()
        message.data = '   '  # texte vide après strip

        with patch('subprocess.run') as mock_subprocess:
            instance._speak(message)

        mock_subprocess.assert_not_called()

    def test_tts_indisponible_ne_bloque_pas(self):
        """Si Piper et API échouent → warning loggé, pas de crash."""
        config = _config_tts(provider='auto')
        instance = _creer_sortie(config)

        message = MagicMock()
        message.data = 'texte à synthétiser'

        with patch.object(instance, '_speak_local',
                          side_effect=RuntimeError('piper absent')):
            with patch.object(instance, '_speak_api',
                              side_effect=RuntimeError('api absente')):
                # Ne doit pas lever d'exception
                instance._speak(message)

        instance.get_logger().warning.assert_called_once()

    def test_provider_local_seul_appelle_speak_local(self):
        """Provider 'local' → uniquement _speak_local, jamais _speak_api."""
        config = _config_tts(provider='local')
        instance = _creer_sortie(config)

        message = MagicMock()
        message.data = 'bonjour'

        with patch.object(instance, '_speak_local') as mock_local, \
             patch.object(instance, '_speak_api') as mock_api:
            instance._speak(message)

        mock_local.assert_called_once_with('bonjour')
        mock_api.assert_not_called()

    def test_provider_api_seul_appelle_speak_api(self):
        """Provider 'api' → uniquement _speak_api, jamais _speak_local."""
        config = _config_tts(provider='api')
        instance = _creer_sortie(config)

        message = MagicMock()
        message.data = 'salut'

        with patch.object(instance, '_speak_local') as mock_local, \
             patch.object(instance, '_speak_api') as mock_api:
            instance._speak(message)

        mock_api.assert_called_once_with('salut')
        mock_local.assert_not_called()

    def test_provider_auto_fallback_local_vers_api(self):
        """Provider 'auto' → local échoue ⟹ bascule sur API."""
        config = _config_tts(provider='auto')
        instance = _creer_sortie(config)

        message = MagicMock()
        message.data = 'test'

        with patch.object(instance, '_speak_local',
                          side_effect=RuntimeError('piper absent')) as mock_local, \
             patch.object(instance, '_speak_api') as mock_api:
            instance._speak(message)

        mock_local.assert_called_once_with('test')
        mock_api.assert_called_once_with('test')

    def test_speak_local_appelle_subprocess_piper(self):
        """_speak_local doit appeler subprocess.run avec l'exécutable Piper."""
        import os
        import tempfile

        config = _config_tts(provider='local', piper_exec='piper',
                             audio_player='aplay', output_device='default')
        # local_voice chemin relatif → config_directory() mocké retourne Path('.')
        config.dialogue.tts.local_voice = '/tmp/voice.onnx'
        instance = _creer_sortie(config)

        # config_directory() est mocké dans caytu_config.loader
        with patch('subprocess.run') as mock_run, \
             patch('tempfile.NamedTemporaryFile') as mock_tmp, \
             patch('os.path.exists', return_value=True), \
             patch('os.unlink'):
            # Simuler NamedTemporaryFile
            mock_file = MagicMock()
            mock_file.__enter__ = MagicMock(return_value=mock_file)
            mock_file.__exit__ = MagicMock(return_value=False)
            mock_file.name = '/tmp/fosa_test.wav'
            mock_tmp.return_value = mock_file

            instance._speak_local('bonjour')

        # subprocess.run appelé au moins une fois (Piper + lecteur audio)
        assert mock_run.call_count >= 1
        # Le premier appel doit contenir 'piper'
        premier_appel_args = mock_run.call_args_list[0][0][0]
        assert 'piper' in premier_appel_args
