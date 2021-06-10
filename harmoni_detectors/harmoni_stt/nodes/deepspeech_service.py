#!/usr/bin/env python3

#Common Imports
import rospy
import roslib 

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

# Specific Imports
from harmoni_common_lib.constants import State, DetectorNameSpace, SensorNameSpace
from harmoni_stt.deepspeech_client import DeepSpeechClient
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
import io
import numpy as np
import os


class SpeechToTextService(HarmoniServiceManager):
	"""
	Speech to text service using DeepSpeech
	"""

	def __init__(self, name, param):
		""" Initialization of variables and deepspeech parameters """
		super().__init__(name)
		# self.model_file_path = param["model_file_path"]
		# self.scorer_path = param["scorer_path"]
		self.model_file_path = os.path.join(
			"/root", "harmoni_catkin_ws", "src", "speechbot", "models", "deepspeech-0.9.3-models.pbmm"
		)
		self.scorer_path = os.path.join(
			"/root", "harmoni_catkin_ws", "src", "speechbot", "models", "deepspeech-0.9.3-models.scorer"
		)
		self.lm_alpha = param["lm_alpha"]
		self.lm_beta = param["lm_beta"]
		self.beam_width = param["beam_width"]
		self.t_wait = param["t_wait"]
		self.subscriber_id = param["subscriber_id"]

		self.service_id = hf.get_child_id(self.name)

		self.ds_client = DeepSpeechClient(
			self.model_file_path,
			self.scorer_path,
			self.lm_alpha,
			self.lm_beta,
			self.beam_width,
			t_wait=2
		)

		"""Setup publishers and subscribers"""
		rospy.Subscriber("/audio/audio", AudioData, self.playing_sound_pause_callback)
		rospy.Subscriber(
			SensorNameSpace.microphone.value + self.subscriber_id,
			AudioData,
			self.sound_data_callback,
		)
		self.text_pub = rospy.Publisher(
			DetectorNameSpace.stt.value + self.service_id, String, queue_size=10
		)

		self.state = State.INIT
		return

	def start(self, rate=""):
		rospy.loginfo("Start the %s service" % self.name)
		if self.state == State.INIT:
			self.state = State.START
		else:
			self.state = State.START
		return

	def stop(self):
		rospy.loginfo("Stop the %s service" % self.name)
		try:
			text = self.ds_client.finish_stream()
			rospy.loginfo(f"Final text: {text}")
			self.state = State.SUCCESS
		except Exception:
			self.state = State.FAILED
		return

	def pause(self):
		rospy.loginfo("Pause the %s service" % self.name)
		self.state = State.SUCCESS
		return

	def sound_data_callback(self, data):
		"""Callback function subscribing to the microphone topic
		Passes audio data to DeepSpeech client
		"""
		data = np.fromstring(data.data, np.uint8)
		if self.state == State.START:
			self.transcribe_stream_request(data)
		# else:
			# rospy.loginfo(f"Not Transcribing data because state is {self.state}")

	def transcribe_stream_request(self, data):
		stream = data
		text = self.ds_client.process_audio(data)
		rospy.loginfo(f"I heard: {text}")
		if self.ds_client.is_final:
			rospy.loginfo(f"Final text: {text}")
			self.text_pub.publish(text)
		return

	def request(self, input_data):
		rospy.loginfo("Start the %s request" % self.name)
		self.state = State.START
		self.transcribe_stream_request(input_data)
		return

	def playing_sound_pause_callback(self, data):
		"""Sleeps when data is being published to the speaker"""
		rospy.loginfo(f"pausing for data: {len(data.data)}")
		self.pause()
		rospy.sleep(int(len(data.data) / 22040))
		self.start()
		return


def main():
	"""Set names, collect params, and give service to server"""

	service_name = DetectorNameSpace.stt.name  # "stt"
	instance_id = rospy.get_param("instance_id")  # "default"
	service_id = f"{service_name}_{instance_id}"

	try:
		rospy.init_node(service_name, log_level=rospy.INFO)

        # stt/default_param/[all your params]
		params = rospy.get_param(service_name + "/" + instance_id + "_param/")
        
		s = SpeechToTextService(service_id, params)

		service_server = HarmoniServiceServer(service_id, s)

		service_server.start_sending_feedback()
		rospy.spin()

	except rospy.ROSInterruptException:
		pass


if __name__ == "__main__":
	main()

