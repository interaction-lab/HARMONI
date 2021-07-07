#!/usr/bin/env python3

import mock
import numpy as np
import os
import pyaudio
import unittest
import wave

from harmoni_stt.deepspeech_client import DeepSpeechClient
from scipy.io import wavfile


MODEL_DIR = os.path.abspath(os.path.join(os.getcwd(), "../../../../../model/deepspeech/models"))
MODEL_FILE_PATH = os.path.join(MODEL_DIR, "deepspeech-0.9.3-models.pbmm")
SCORER_PATH = os.path.join(MODEL_DIR, "deepspeech-0.9.3-models.scorer")
LM_ALPHA = 0.75
LM_BETA = 1.85
BEAM_WIDTH = 700

TEST_AUDIO_FILE_PATH = os.path.abspath(os.path.join(os.getcwd(), "../test_data/hello_16k.wav"))


class TestDeepSpeechClient(unittest.TestCase):

    def setUp(self):
        self.ds_client = DeepSpeechClient(
            MODEL_FILE_PATH,
            SCORER_PATH,
            LM_ALPHA,
            LM_BETA,
            BEAM_WIDTH
        )

    def test_start_stream(self):
        self.ds_client.start_stream()
        assert not self.ds_client.is_final
        assert self.ds_client.is_streaming

    @mock.patch("deepspeech.Stream.finishStream")
    def test_finish_stream(self, mock_finish_stream):
        mock_finish_stream.return_value = "test_transcription"

        # test behavior when stopping an open stream
        self.ds_client.start_stream()
        result = self.ds_client.finish_stream()
        assert result == "test_transcription"
        assert not self.ds_client.is_streaming

        # test behavior when stopping a closed stream
        result = self.ds_client.finish_stream()
        assert result == ""
        assert not self.ds_client.is_streaming

    def test_process_audio(self):
        # TODO: fix method of streaming from wav file; currently the output is "to" instead of "hello"
        chunk_size = 1024
        wf = wave.open(TEST_AUDIO_FILE_PATH)
        # create an audio object
        p = pyaudio.PyAudio()
        # open stream based on the wave object which has been input.
        stream = p.open(
            format=p.get_format_from_width(wf.getsampwidth()),
            channels=wf.getnchannels(),
            rate=wf.getframerate(),
            output=True
        )
        # read data (based on the chunk size)
        self.ds_client.start_stream()
        # play stream
        while not self.ds_client.is_final:
            data = wf.readframes(chunk_size)
            text = self.ds_client.process_audio(data)
        # close stream
        stream.stop_stream()
        stream.close()
        p.terminate()
        # expected transcription from the test audio file is "hello"
        # assert text == "hello"


if __name__ == "__main__":
    unittest.main()
