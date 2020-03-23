#!/usr/bin/env python
from sys import byteorder
from array import array
from struct import pack
import sys
import pyaudio
import wave
import rospy
import actionlib
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
from cordial_manager.msg import *
import boto3
import json
from datetime import datetime


USER_ID = "micol_testing"
BOT_NAME = "QTRobotBot"
BOT_ALIAS = "qt_robot_demo" 


#USER_ID = "micol_testing"
#BOT_NAME = "QTDemo"
#BOT_ALIAS = "qt_sample_demo" 

SAMPLERATE = 16000
FORMAT_SIZE = pyaudio.paInt16
CHUNK_SIZE = 1024


class DialogueServer():
	_feedback = CordialFeedback()
	_result = CordialResult()
	def __init__(self, name, manager):
		self.controller_manager = manager
		self.controller_manager.prompt_message = ''
		self.controller_manager.dialogue_process_done = False
		self.controller_manager.interaction_message = ''
		self.controller_manager.interaction_continue = True
		self.action_name = name
		self.action = actionlib.SimpleActionServer(self.action_name, CordialAction, self.execute_goal, False)
		self.action.start()

	def execute_goal(self, goal):
		print(goal)
		goal_name = goal.action
		success = True
		if goal.optional_data != '':
			self.controller_manager.prompt_message = goal.optional_data
			print("Dialogue has heard: " + self.controller_manager.prompt_message)
			self.controller_manager.send_textToAWS(self.controller_manager.prompt_message)
		else:
			while self.controller_manager.prompt_message == '':
				rospy.logdebug("Waiting from the microphone input data")
				rospy.Rate(1)
			self.controller_manager.send_audioToAWS_client(self.controller_manager.prompt_message)
		self._feedback.action = goal_name
		#self._feedback.state = #Controller state
		## Decide when to send the feedback
		# self.action.publish_feedback(self._feedback)
		while not self.controller_manager.dialogue_process_done:
			if self.action.is_preempt_requested():
					print("action preempted")
					self.action.set_preempted()
					success = False
			rospy.Rate(10)
		if success:
			print("enter if success")
			self._result.do_continue = self.controller_manager.interaction_continue
			self._result.action = goal_name
			self._result.message = self.controller_manager.interaction_message
			self.controller_manager.prompt_message = ''
			self.controller_manager.dialogue_process_done = False
			self.controller_manager.interaction_message = ''
			self.controller_manager.interaction_continue = True
			self.action.set_succeeded(self._result)


class DialogueManager():
	def __init__(self):
		# Initialize variables useful for the Server
		self.prompt_message = ""
		self.dialogue_process_done = False
		self.interaction_message = ""
		self.interaction_continue = True
		self.lex_client = boto3.client('lex-runtime', region_name='us-west-2')
		# Declare subscribers and publishers
	   	rospy.Subscriber('/cordial/microphone/audio', AudioData, self.handle_audio_data, queue_size=1)
		self.text_publisher = rospy.Publisher('cordial/dialogue/script', String, queue_size=1)
		self.dialogue_data_publisher = rospy.Publisher('cordial/dialogue/data', String, queue_size=1)
		

	def handle_audio_data(self, data):
		self.prompt_message = data.data

	def handle_lex_response(self,lex_response):
		if len(lex_response["message"]) > 0:
			print("The lex response is: ", lex_response["message"])
			#Stored the user data
			self.dialogue_data_publisher.publish(str(lex_response["sessionAttributes"]))
			#When lex failed in understanding the user
			if "intentName" in lex_response:
				if lex_response["dialogState"] == 'Failed':
					self.interaction_message = 'failed_understanding'
					self.interaction_continue = False
					print("In Failed dialogue state")
				elif lex_response["dialogState"] == 'Fulfilled':
					self.interaction_message = 'success'
					self.interaction_continue = False
					print("In Fulfilled dialogue state, the response is:" + lex_response["message"])
					self.text_publisher.publish(lex_response["message"])
				else:
					print("In general dialogue state, the response is:" + lex_response["message"])
					self.text_publisher.publish(lex_response["message"])
			else:
				self.interaction_message = 'failed_understanding'
				self.interaction_continue = False
				print("Empty string")
				self.text_publisher.publish(lex_response["message"])
			self.dialogue_process_done = True

	def send_audioToAWS_client(self,audiodatarequest):
		print("Starting client request..")
		#print("The audio data request is: ", audiodatarequest)
		audiodata = audiodatarequest
		p = pyaudio.PyAudio()
		file_name = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
		file_name = "audio_user"
		outdir = "/home/qtrobot/catkin_ws/src/cordial-public/cordial_logger/scripts/data/audio_user"
		wf = wave.open(outdir + "/"+ file_name + ".wav", 'wb')
		wf.setnchannels(1)
		wf.setsampwidth(p.get_sample_size(FORMAT_SIZE))
		wf.setframerate(SAMPLERATE)
		wf.setnframes(CHUNK_SIZE)
		wf.writeframes(b''.join(audiodata))
		wf.close()
		wav_file = wave.open(outdir + "/"+ file_name + ".wav", 'rb')
		try:
			lex_response = self.lex_client.post_content(botName = BOT_NAME,
														botAlias = BOT_ALIAS,
														userId = USER_ID,
														contentType = 'audio/x-l16; sample-rate=16000; channel-count=1',
														accept = 'text/plain; charset=utf-8',
														inputStream = audiodata)
			self.handle_lex_response(lex_response)
		except rospy.ServiceException, e:
			print "Service call failed: %s" %e

	def send_textToAWS(self,textdatarequest):
		print("Starting client request..")
		textdata = textdatarequest
		try:
			lex_response = self.lex_client.post_content(botName = BOT_NAME,
														botAlias = BOT_ALIAS,
														userId = USER_ID,
														contentType = 'text/plain; charset=utf-8',
														accept = 'text/plain; charset=utf-8',
														inputStream = textdata)
			self.handle_lex_response(lex_response)
		except rospy.ServiceException, e:
			print "Service call failed: %s" %e
	

if __name__ == '__main__':
		rospy.init_node("dialogue_node", anonymous=True)
		controller_manager = DialogueManager()
		DialogueServer("dialoging", controller_manager)
		rospy.spin()
    
    




