import os
import unittest

from harmoni_tts.local_tts_client import TtsClient

tts_dir = os.path.abspath(os.path.join(os.getcwd(), ".."))
content_dir = os.path.abspath(os.path.join(tts_dir, "../../../../model/tts"))
tts_config = os.path.join(content_dir, "config.json")
tts_model = os.path.join(content_dir, "tts_model.pth.tar")
vocoder_config = os.path.join(content_dir, "config_vocoder.json")
vocoder_model = os.path.join(content_dir, "vocoder_model.pth.tar")
scale_stats_path = os.path.join(tts_dir, "scale_stats.npy")


class TestTtsClient(unittest.TestCase):

    def setUp(self):
        self.client = TtsClient(
            tts_config,
            tts_model,
            vocoder_config,
            vocoder_model,
            scale_stats_path
        )

    def test_get_audio(self):
        _, _, _, waveform = self.client.get_audio("Hello")
        assert len(waveform) > 0


if __name__ == "__main__":
    unittest.main()
