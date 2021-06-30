#!/usr/bin/env python3

import deepspeech
import mock
import numpy as np
import os
import unittest

from scipy.io import wavfile
from harmoni_stt.deepspeech_client import DeepSpeechClient

dir_name = os.path.dirname(__file__)

MODEL_DIR = os.path.join(
    dir_name, "../../..", "model", "deepspeech", "models"
)
MODEL_FILE_PATH = os.path.join(MODEL_DIR, "deepspeech-0.9.3-models.pbmm")
SCORER_PATH = os.path.join(MODEL_DIR, "deepspeech-0.9.3-models.scorer")
LM_ALPHA = 0.75
LM_BETA = 1.85
BEAM_WIDTH = 700

TEST_AUDIO_FILE_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "test_data",
    "hello.wav"
)


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

        # test behavior when trying to stop a closed stream
        result = self.ds_client.finish_stream()
        assert result == ""
        assert not self.ds_client.is_streaming


if __name__ == "__main__":
    unittest.main()
