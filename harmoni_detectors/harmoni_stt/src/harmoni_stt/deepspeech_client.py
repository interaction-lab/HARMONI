#!/usr/bin/env python3
import deepspeech
import logging
import numpy as np
import os
import pyaudio
import sys
import time

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
        
        # Create a Streaming session
		self._ds_stream = self._model.createStream()

		self._text = ''
		self._t_start = time.time()
		self._t_wait = t_wait
		self._is_final = False

	def process_audio(self, data16):
		self._is_final = False
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
					logging.info("Finishing stream")
					text = self._ds_stream.finishStream()
					logging.info('Final text = {}.\n'.format(text))
					self._text = text
					self._is_final = True

		except Exception as e:
			logging.info(f"Text: '{text}'; So far: '{self.text_so_far}")
			logging.info(self._t_start)
			raise e

		return text

	def finish_stream(self):
		text = self._ds_client.finishStream()
		return text

	def transcribe_from_file(self, audio_file):
		pass
	
	@property
	def is_final(self):
		return self._is_final


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

