#!/usr/bin/env python3.8
import deepspeech
import logging
import numpy as np
import os
import pyaudio
import time
import wave

logging.basicConfig(level=logging.INFO)


class DeepSpeechClient:

    def __init__(
        self,
        model_file_path,
        scorer_path,
        alpha,
        beta,
        beam_width,
        t_wait=0.5
    ):
        if not os.path.exists(model_file_path):
            raise FileNotFoundError(f"Invalid model path: {model_file_path}")
        self._model = deepspeech.Model(model_file_path)

        if not os.path.exists(scorer_path):
            raise FileNotFoundError(f"Invalid scorer path: {scorer_path}")
        self._model.enableExternalScorer(scorer_path)

        self._model.setScorerAlphaBeta(alpha, beta)
        self._model.setBeamWidth(beam_width)

        self._is_streaming = False

        # Create a Streaming session
        self._ds_stream = self._model.createStream()
        self._text = ''
        self._t_start = time.time()
        self._t_wait = t_wait
        self._is_final = False

    def process_audio(self, in_data):
        self._is_final = False
        data16 = np.frombuffer(in_data, dtype=np.int16)
        self._ds_stream.feedAudioContent(data16)
        text = self._ds_stream.intermediateDecode()

        try:
            if text != self._text:
                if text not in ["i ", "he ", "the "]:
                    logging.info('Interim text = {};'.format(text))
                self._text = text
                self._t_start = time.time()

            elif text != '' and (time.time() - self._t_start > self._t_wait):
                if text not in ["i ", "he ", "the "]:
                    self._text = ""
                    logging.info(f"Final text = {text}\n")
                    self._is_final = True

        except Exception as e:
            logging.info(f"Text: '{text}'; So far: '{self._text}")
            logging.info(self._t_start)
            raise e

        return text

    def start_stream(self):
        self._is_final = False
        """Starts DeepSpeech streaming inference state if it is not already open."""
        if not self._is_streaming:
            self._ds_stream = self._model.createStream()
            self._is_streaming = True
        else:
            logging.info("Tried to start stream when DeepSpeech client already streaming.")

    def finish_stream(self):
        """Ends DeepSpeech streaming inference state if it is still open.
        Returns the transcrbed text,
        """
        if self._is_streaming:
            text = self._ds_stream.finishStream()
            self._is_streaming = False
            self._is_final = False
        else:
            logging.info("Tried to end stream when DeepSpeech client currently not streaming")
            text = ""
        return text

    def restart_stream(self):
        text = self.finish_stream()
        logging.info(f"Restarting stream; last transcript: {text}")
        self.start_stream()

    def transcribe_from_file(self, audio_file):
        chunk = 1024
        wf = wave.open(audio_file, 'rb')
        data = wf.readframes(chunk)
        while data != '':
            self.process_audio(data)
            data = wf.readframes(chunk)
        return

    @property
    def is_final(self):
        return self._is_final

    @is_final.setter
    def is_final(self, is_final):
        self._is_final = is_final

    @property
    def is_streaming(self):
        return self._is_streaming


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("model_file_path")
    parser.add_argument("scorer_path")

    args = parser.parse_args()

    model_file_path = args.model_file_path
    scorer_path = args.scorer_path

    lm_alpha = 0.75
    lm_beta = 1.85
    beam_width = 700

    ds_client = DeepSpeechClient(
        model_file_path,
        scorer_path,
        lm_alpha,
        lm_beta,
        beam_width,
        t_wait=2
    )
