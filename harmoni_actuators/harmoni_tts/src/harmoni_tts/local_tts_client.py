#!/usr/bin/env python3

import os
import torch
import time

from TTS.vocoder.utils.generic_utils import setup_generator
from TTS.utils.generic_utils import setup_model
from TTS.utils.io import load_config
from TTS.utils.text.symbols import symbols, phonemes
from TTS.utils.audio import AudioProcessor
from TTS.utils.synthesis import synthesis

import sounddevice as sd


class TtsClient:

    def __init__(
        self,
        tts_config,
        tts_model,
        vocoder_config,
        vocoder_model,
        scale_stats_path,
        use_cuda=False,
        verbose=False,
        speedup=1.1
    ):
        self._use_cuda = use_cuda
        self._verbose = verbose
        self._speedup = speedup

        self._check_files_exist([
            tts_config, tts_model, vocoder_config, vocoder_model
        ])

        # Load configs
        self._tts_config = load_config(tts_config)
        self._vocoder_config = load_config(vocoder_config)

        # Make sure scale_stats.npy path is correct when using with roslaunch
        self._tts_config.audio["stats_path"] = scale_stats_path
        self._vocoder_config.audio["stats_path"] = scale_stats_path

        # Load audio processor
        self._ap = AudioProcessor(**self._tts_config.audio)

        # LOAD TTS MODEL
        self._speaker_id = None
        speakers = []
        num_chars = len(phonemes) if self._tts_config.use_phonemes else len(symbols)
        self._tts_model = setup_model(num_chars, len(speakers), self._tts_config)

        # Load model state
        cp = torch.load(tts_model, map_location=torch.device('cpu'))
        self._tts_model.load_state_dict(cp['model'])
        if self._use_cuda:
            self._tts_model.cuda()
        self._tts_model.eval()

        # Set model stepsize
        if 'r' in cp:
            self._tts_model.decoder.set_r(cp['r'])

        # LOAD VOCODER MODEL
        self._vocoder_model = setup_generator(self._vocoder_config)
        self._vocoder_model.load_state_dict(torch.load(vocoder_model, map_location="cpu")["model"])
        self._vocoder_model.remove_weight_norm()
        self._vocoder_model.inference_padding = 0

        if self._use_cuda:
            self._vocoder_model.cuda()
        self._vocoder_model.eval()

    def _check_files_exist(self, list_of_files):
        for file in list_of_files:
            if not os.path.exists(file):
                raise FileNotFoundError(f"Invalid file path: {file}")

    def get_audio(self, text, use_gl=False):
        t_1 = time.time()
        waveform, alignment, mel_spec, mel_postnet_spec, stop_tokens, inputs = synthesis(
            self._tts_model,
            text,
            self._tts_config,
            self._use_cuda,
            self._ap,
            self._speaker_id,
            truncated=False,
            enable_eos_bos_chars=self._tts_config.enable_eos_bos_chars
        )
        if not use_gl:
            waveform = self._vocoder_model.inference(torch.FloatTensor(mel_postnet_spec.T).unsqueeze(0))
            waveform = waveform.flatten()
        if self._use_cuda:
            waveform = waveform.cpu()
        waveform = waveform.numpy()
        rtf = (time.time() - t_1) / (len(waveform) / self._ap.sample_rate)
        tps = (time.time() - t_1) / len(waveform)
        if self._verbose:
            print(" > Run-time: {}".format(time.time() - t_1))
            print(" > Real-time factor: {}".format(rtf))
            print(" > Time per step: {}".format(tps))
        return alignment, mel_postnet_spec, stop_tokens, waveform


if __name__ == "__main__":
    import argparse

    # parser = argparse.ArgumentParser()
    # parser.add_argument("tts_config")
    # parser.add_argument("tts_model")
    # parser.add_argument("vocoder_config")
    # parser.add_argument("vocoder_model")
    #
    # args = parser.parse_args()
    #
    # tts_config = args.tts_config
    # tts_model = args.tts_model
    # vocoder_config = args.vocoder_config
    # vocoder_model = args.vocoder_model

    content_dir = os.path.join(
        "/root", "harmoni_catkin_ws", "src", "HARMONI", "harmoni_actuators", "harmoni_tts", "content"
    )
    tts_config = os.path.join(content_dir, "config.json")
    tts_model = os.path.join(content_dir, "tts_model.pth.tar")
    vocoder_config = os.path.join(content_dir, "config_vocoder.json")
    vocoder_model = os.path.join(content_dir, "vocoder_model.pth.tar")

    tts_client = TtsClient(
        tts_config,
        tts_model,
        vocoder_config,
        vocoder_model
    )

    sentence = "How are you doing today?"
    align, spec, stop_tokens, wav = tts_client.get_audio(sentence, use_gl=False)
    sd.play(wav, 22050 * 1.1, blocking=True)
