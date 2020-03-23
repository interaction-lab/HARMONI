#!/usr/bin/env python

import rospy
import os
import sys
import json
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from audio_common_msgs.msg import AudioData
from qt_robot_speaker.msg import PlayRequest
from qt_nuitrack_app.msg import Skeletons
from datetime import datetime
import wave
import pyaudio
import ast

CHANNEL = 1
SAMPLERATE = 16000
SAMPLERATE_MIC_EXT = 44100
FORMAT_SIZE = pyaudio.paInt16
CHUNK_SIZE = 1024
FPS = 24

class RecordingManager():

	def __init__(self):
		self.recorded_audio_common_frames = []
		self.recorded_audio_respeaker_frames = []
		self.script_data = []
		self.first_video_frame = True
		self.first_video_head_frame = True
		self.first_audio_frame = True
		self.first_audio_ext_frame = True
		self.is_video_recording = False
		self.is_audio_recording = False
		self.is_data_recording = False
		self.cv_bridge = CvBridge()
		self.fps = FPS
		rospy.Subscriber("/cordial/chest_camera/video", Image, self.handle_recorded_video, queue_size=1)
		rospy.Subscriber("/audio/channel0", AudioData, self.handle_recorded_audio_respeaker, queue_size=1)
		rospy.Subscriber("/audio", AudioData, self.handle_recorded_audio_common, queue_size=1)
		#rospy.Subscriber("/cordial/recording/audio/data", PlayRequest, self.handle_recorded_audio_common, queue_size=1)
		rospy.Subscriber("/cordial/dialogue/data", String, self.handle_user_prompt, queue_size=1)
		rospy.Subscriber("/camera/color/image_raw", Image, self.handle_recorded_video_head, queue_size=1)
		rospy.Subscriber("/qt_nuitrack_app/skeletons", Skeletons, self.handle_recorded_skeleton_position, queue_size=1)

		rospy.Subscriber("/cordial/recording/audio", Bool, self.handle_trigger_recorded_audio, queue_size=1)
		rospy.Subscriber("/cordial/recording/video", Bool, self.handle_trigger_recorded_video, queue_size=1)
		rospy.Subscriber("/cordial/recording/data", Bool, self.handle_trigger_recorded_data, queue_size=1)
		

	def handle_recorded_skeleton_position(self, data):
		return
	
	def handle_user_prompt(self,data):
		if self.is_data_recording:
			transcript = ast.literal_eval(data.data)
			self.script_data.append(transcript)
		return

	def handle_trigger_recorded_data(self, data):
		if not data.data:
			self.is_data_recording = False
			file_name = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
			file_name = "transcript_" + file_name
			outdir = "/media/qtrobot/Micol-Study/EmpiricalStudy/logfile/script/"
			with open(outdir + file_name+'.json', 'w') as outfile:
				json.dump(self.script_data, outfile)
		else:
			self.is_data_recording = True


	def handle_recorded_video_head(self,data):
		if self.is_video_recording:
			if self.first_video_head_frame:
				print("First frame")
				# Create the writer for the video
				fourcc = cv2.VideoWriter_fourcc(*'MJPG')
				file_name = datetime.now().strftime('%Y-%m-%d_%H-%M-%S') 
				file_name = "video_head_" + file_name
				path = "/media/qtrobot/Micol-Study/EmpiricalStudy/video_head/"
				fps = FPS
				self.video_head_data = cv2.VideoWriter(path + file_name + ".avi", fourcc , fps, (data.width, data.height), True)
				frame = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
				self.first_video_head_frame = False
			else:
				frame = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
			try:
				self.video_head_data.write(frame)
			except:
				print("Not writing!!")

	def handle_trigger_recorded_video(self, data):
		if not data.data:
			print("Stop video recording")
			self.is_video_recording = False
			self.video_data.release()
			self.video_head_data.release()
			cv2.destroyAllWindows()
		elif data.data:
			self.is_video_recording = True


	def handle_recorded_video(self, data):
		if self.is_video_recording:
			if self.first_video_frame:
				print("First frame")
				# Create the writer for the video
				#self.fps = data.header.stamp.secs
				fps = FPS
				fourcc = cv2.VideoWriter_fourcc(*'MJPG')
				file_name = datetime.now().strftime('%Y-%m-%d_%H-%M-%S') 
				file_name = "video_chest_" + file_name
				path = "/media/qtrobot/Micol-Study/EmpiricalStudy/video/"
				self.video_data = cv2.VideoWriter(path + file_name + ".avi", fourcc , fps, (data.width, data.height), True)
				frame = self.cv_bridge.imgmsg_to_cv2(data, "rgb8")
				self.first_video_frame = False
			
			else:
				frame = self.cv_bridge.imgmsg_to_cv2(data, "rgb8")
			try:
				self.video_data.write(frame)
			except:
				print("Not writing!!")

	def handle_recorded_audio_respeaker(self,data):
		if self.is_audio_recording:
			#self.recorded_audio_respeaker_frames.append(data.data)
			if self.first_audio_frame:
				file_name_date = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
				p = pyaudio.PyAudio()
				file_name = "audio_head_" + file_name_date
				outdir = "/media/qtrobot/Micol-Study/EmpiricalStudy/audio/"
				self.wf = wave.open(outdir + "/"+ file_name + ".wav", 'wb')
				self.wf.setnchannels(CHANNEL)
				self.wf.setsampwidth(p.get_sample_size(FORMAT_SIZE))
				self.wf.setframerate(SAMPLERATE)
				self.wf.setnframes(CHUNK_SIZE)
				self.wf.writeframes(b''.join(data.data))
				self.first_audio_frame = False
			else:
				self.wf.writeframes(b''.join(data.data))

	def handle_recorded_audio_common(self,data):
		if self.is_audio_recording:
			#self.recorded_audio_common_frames.append(data.data)
			if self.first_audio_ext_frame:
				file_name_date = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
				p_ext = pyaudio.PyAudio()
				outdir = "/media/qtrobot/Micol-Study/EmpiricalStudy/audio_ext/"
				file_name = "audio_ext_" + file_name_date
				self.wf_ext = wave.open(outdir + "/"+ file_name + ".wav", 'wb')
				self.wf_ext.setnchannels(CHANNEL)
				self.wf_ext.setsampwidth(p_ext.get_sample_size(FORMAT_SIZE))
				self.wf_ext.setframerate(SAMPLERATE)
				self.wf_ext.setnframes(CHUNK_SIZE)
				self.first_audio_ext_frame = False
				self.wf_ext.writeframes(b''.join(data.data))
			else:
				self.wf_ext.writeframes(b''.join(data.data))

	def handle_trigger_recorded_audio(self, data):
		if not data.data:
			self.is_audio_recording = False
			self.wf.close()
			self.wf_ext.close()
		elif data.data:
			self.is_audio_recording = True




if __name__ == '__main__':
	rospy.init_node("recorded_node", anonymous=True)
	controller_manager = RecordingManager()
	rospy.spin()