#!/usr/bin/env python

import rospy
import os
import sys
import pyaudio
import wave
import struct
import json
import time
import random
import numpy as np
import actionlib
from std_msgs.msg import String
from cordial_tts import CoRDialTTS
from cordial_behavior.msg import Behavior
from cordial_manager.msg import *
import soundfile as sf


WAV_HEADER_LENGTH = 24

class SynthesizeServer():
	_feedback = CordialFeedback()
	_result = CordialResult()
	def __init__(self, name, manager):
		self.controller_manager = manager
		self.controller_manager.dialogue_message = ''
		self.controller_manager.synthesize_done = False
		self.controller_manager.interaction_message = ''
		self.controller_manager.interaction_continue = True
		self.action_name = name
		self.action = actionlib.SimpleActionServer(self.action_name, CordialAction, self.execute_goal, False)
		self.action.start()

	def execute_goal(self, goal):
		goal_name = goal.action
		success = True
		if goal.optional_data != '':
			self.controller_manager.dialogue_message = goal.optional_data
		while self.controller_manager.dialogue_message == '':
			#print("Wait until dialogue message is updated")
			rospy.Rate(10)
		self.controller_manager.handle_tts_realtime(self.controller_manager.dialogue_message)
		print("The dialogue message is:" + self.controller_manager.dialogue_message)
		self._feedback.action = goal_name
		#self._feedback.state = controller state
		## Decide when to send the feedback
		# self.action.publish_feedback(self._feedback)
		while not self.controller_manager.synthesize_done:
			if self.action.is_preempt_requested():
					self.action.set_preempted()
					success = False
			rospy.Rate(10)
		if success:
			self._result.do_continue = self.controller_manager.interaction_continue
			self._result.action = goal_name
			self._result.message = self.controller_manager.interaction_message
			self.controller_manager.dialogue_message = ''
			self.controller_manager.synthesize_done = False
			self.controller_manager.interaction_message = ''
			self.controller_manager.interaction_continue = True
			self.action.set_succeeded(self._result)


class TTSManager():
	def __init__(self):
		# Initialize variables useful for the Server
		self.dialogue_message = ""
		self.synthesize_done = False
		self.interaction_message = ""
		self.interaction_continue = True
		# Declare subscribers and publishers
		rospy.Subscriber('cordial/dialogue/script', String, self.handle_dialogue_message, queue_size=1)
		self.behavior_publisher = rospy.Publisher("cordial/behavior", Behavior, queue_size=1)
		

	def handle_dialogue_message(self, data):
		self.dialogue_message = data.data
		print("TTS Heard from Lex: " + self.dialogue_message)
		return

	def handle_tts_realtime(self, data):
		print("The TTS message received")
		#outdir = os.path.dirname(os.path.abspath("home		
		outdir = "/home/qtrobot/catkin_ws/src/cordial-public/cordial_dialogue/scripts/data" #TODO: absolute path
		voice = "Justin"	
		tts = CoRDialTTS(voice)
		phraseID = "1"
		text_content = data
		print("The text content is: " + text_content)
		if "." in text_content:
			text_array = text_content.split(".")
		else:
			text_array = []
			text_array.append(text_content)
		behavior_array = []
		audio_frame_array = []
		audio_data_array = []
		for text in text_array:
			if text == "":
				rospy.loginfo("Empty string")
			else:
				file_saved = tts.phrase_to_file(phraseID, text, outdir)
				#print("The output from the TTS is: ", file_saved)
				behaviours = sorted(file_saved["behaviors"], key = lambda i: i['start']) # sorting the behaviours
				path_audio_file = file_saved["file"] # path of audiofile.ogg
				data, samplerate = sf.read(outdir + '/'+phraseID+'.ogg')
				sf.write(outdir + '/'+phraseID+'.wav', data, samplerate)
				file_handle =outdir + '/'+phraseID+'.wav'
				data = np.fromfile(file_handle, np.uint8)[WAV_HEADER_LENGTH:] #Loading wav file
				data = data.astype(np.uint8).tostring()
				data_array = data
				audio_frame = samplerate
				behavior_array.append(str(behaviours))
				audio_data_array.append(data_array)
				audio_frame_array.append(audio_frame)
		behavior_msg = Behavior()
		behavior_msg.audio_frame = audio_frame_array
		behavior_msg.audio_data =  audio_data_array
		behavior_msg.behavior_json = behavior_array
		print("The behavior message is sent")
		self.behavior_publisher.publish(behavior_msg)
		self.synthesize_done = True

if __name__ == '__main__':
		rospy.init_node("tts_node", anonymous=True)
		controller_manager = TTSManager()
		SynthesizeServer("synthesizing", controller_manager)
		rospy.spin()
