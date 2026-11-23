from io import BytesIO
from unittest.mock import Mock, patch
import wave

import pytest
import speech_recognition as sr
from speech_recognition.recognizers.google_cloud import _build_config

from nira_communication.stt_online import SpeechToTextOnline


@pytest.mark.parametrize("transcript", ["stampf", "stomp", "steh auf", "stand up", "dreh nach links", "turn left"])
def test_online_recognition_uses_one_bilingual_cloud_request(transcript):
    callback = Mock()
    audio = sr.AudioData(b'\x00\x00' * 160, 16000, 2)

    def recognize(audio_data, **kwargs):
        # Exercise the installed library's config builder to catch invalid fields.
        config = _build_config(audio_data, kwargs)
        assert config.language_code == 'de-DE'
        assert list(config.alternative_language_codes) == ['en-US']
        assert config.model == 'command_and_search'
        assert {'stomp', 'stampf'} <= set(config.speech_contexts[0].phrases)
        return transcript

    with patch('nira_communication.stt_online.sr.Recognizer') as recognizer, \
            patch('nira_communication.stt_online.sr.Microphone'):
        recognizer.return_value.listen.return_value = audio
        recognizer.return_value.recognize_google_cloud.side_effect = recognize
        SpeechToTextOnline(callback, logger=Mock()).run()

    recognizer.return_value.recognize_google_cloud.assert_called_once()
    callback.assert_called_once_with(transcript)


def test_short_command_audio_reaches_transcription():
    # Model 192 ms of voiced input between calibration silence and a full pause.
    pcm = b'\x00\x10' * (3 * 1024)
    wav = BytesIO()
    with wave.open(wav, 'wb') as writer:
        writer.setparams((1, 2, 16000, 0, 'NONE', 'not compressed'))
        writer.writeframes(bytes(64000) + pcm + bytes(64000))
    wav.seek(0)

    class RecordedMicrophone(sr.AudioFile):
        def __enter__(self):
            source = super().__enter__()
            source.CHUNK = 1024  # sr.Microphone's normal buffer size
            return source

    callback = Mock()
    with patch('nira_communication.stt_online.sr.Microphone', return_value=RecordedMicrophone(wav)), \
            patch.object(sr.Recognizer, 'recognize_google_cloud', return_value='stop') as transcribe:
        SpeechToTextOnline(callback, logger=Mock()).run()

    assert pcm in transcribe.call_args.args[0].get_raw_data()
    callback.assert_called_once_with('stop')


def test_cloud_request_failure_completes_with_fallback():
    callback = Mock()
    with patch('nira_communication.stt_online.sr.Recognizer') as recognizer, \
            patch('nira_communication.stt_online.sr.Microphone'):
        recognizer.return_value.recognize_google_cloud.side_effect = sr.RequestError('offline')
        SpeechToTextOnline(callback, logger=Mock()).run()

    callback.assert_called_once_with('das habe ich nicht verstanden')


def test_microphone_failure_completes_request():
    callback = Mock()
    with patch('nira_communication.stt_online.sr.Microphone', side_effect=OSError('no microphone')):
        SpeechToTextOnline(callback, logger=Mock()).run()
    callback.assert_called_once_with('das habe ich nicht verstanden')


def test_cancel_after_capture_skips_network_and_callback():
    callback = Mock()
    worker = SpeechToTextOnline(callback, logger=Mock())
    with patch('nira_communication.stt_online.sr.Recognizer') as recognizer, \
            patch('nira_communication.stt_online.sr.Microphone'):
        recognizer.return_value.listen.side_effect = lambda *args, **kwargs: worker.stop_listening()
        worker.run()
    recognizer.return_value.recognize_google_cloud.assert_not_called()
    callback.assert_not_called()
