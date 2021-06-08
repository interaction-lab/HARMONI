#!/usr/bin/env python3
import deepspeech
import numpy as np
import os
import pyaudio
import time


class DeepSpeechClient:

    def __init__(
            self, 
            model,
            scorer_path,
            alpha,
            beta,
            beam_width
    ):
        self._model = model
        if not os.path.exists(scorer_path):
            raise FileNotFoundError(f"Invalid scorer path: {scorer_path}")
        self._model.enableExternalScorer(scorer_path)
        self.model.setScorerAlphaBeta(alpha, beta)
        self.model.setBeamWidth(beam_width)
        
        # Create a Streaming session
        self.ds_stream = self.model.createStream()

        # Encapsulate DeepSpeech audio feeding into a callback for PyAudio
        self.text_so_far = ''
        self.t_start = time.time()
        self.t_wait = .5
        self.final_text = None

    def listen(self):
        # Feed audio to DeepSpeech in a callback to PyAudio
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=1024,
            stream_callback=self.process_audio
        )

        print('Please start speaking, when done press Ctrl-C ...')
        self.stream.start_stream()
        print("listening now")

        return

    def process_audio(self, in_data, frame_count, time_info, status):
        data16 = np.frombuffer(in_data, dtype=np.int16)
        self.ds_stream.feedAudioContent(data16)
        text = self.ds_stream.intermediateDecode()
        try:
            if text != self.text_so_far:
                if text not in ["i ", "he ", "the "]:
                    print('Interim text = {};'.format(text))
                self.text_so_far = text
                self.t_start = time.time()
            elif text != '' and (time.time() - self.t_start > self.t_wait):
                if text not in ["i ", "he ", "the "]:
                    print("Finishing stream")
                    text = self.ds_stream.finishStream()
                    print('Final text = {}.\n'.format(text))
                    self.final_text = text
                    self.ds_stream = self.model.createStream()

        except Exception as e:
            print(f"Text: '{text}'; So far: '{self.text_so_far}")
            print(self.t_start)
            raise e

        return (in_data, pyaudio.paContinue)

    def transcribe_from_file(self, audio_file):
        pass
