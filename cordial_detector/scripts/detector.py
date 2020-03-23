#!/usr/bin/env python

import rospy
import os
import sys
import json
import time
import actionlib
from std_msgs.msg import String, Bool
from cordial_manager.msg import *

class DetectorServer():
	_feedback = CordialFeedback()
	_result = CordialResult()
	def __init__(self, name, manager):
		self.controller_manager = manager
		self.controller_manager.face_detector_message = ''
		self.controller_manager.detecting_done = False
		self.controller_manager.interaction_message = ''
		self.controller_manager.interaction_continue = True
		self.action_name = name
		self.action = actionlib.SimpleActionServer(self.action_name, CordialAction, self.execute_goal, False)
		self.action.start()

	def execute_goal(self, goal):
		goal_name = goal.action
		success = True
		if goal.optional_data != '':
			self.controller_manager.face_detector_message = goal.optional_data
		self.controller_manager.handle_face_detecting_start(self.controller_manager.face_detector_message)
		self._feedback.action = goal_name
		# self._feedback.state = Detector status
		## Decide when to send the feedback
		self.action.publish_feedback(self._feedback)
		while not self.controller_manager.detecting_done:
			if self.action.is_preempt_requested():
					self.action.set_preempted()
					success = False
			rospy.Rate(10)
		if success:
			self._result.do_continue = self.controller_manager.interaction_continue
			self._result.action = goal_name
			self._result.message = self.controller_manager.interaction_message
			self.controller_manager.face_detector_message = ''
			self.controller_manager.detecting_done = False
			self.controller_manager.interaction_message = ''
			self.controller_manager.interaction_continue = True
			self.action.set_succeeded(self._result)


class DetectorManager():
	def __init__(self):
		# Initialize variables useful for the Server
		self.face_detector_message = ""
		self.detecting_done = False
		self.interaction_continue = ""
		self.interaction_message = True
		# Declare subscribers and publishers
		self.face_detector_publisher = rospy.Publisher("cordial/detector/faces/detecting", Bool, queue_size=1)
		
	def handle_face_detecting_start(self, data):
		global INTERACTION_CONTINUE
		rospy.loginfo("Starting the detection")
		self.face_detector_publisher.publish(True)
		self.interaction_continue = False
		self.interaction_message = "success"
		self.detecting_done = True
		

if __name__ == '__main__':
		rospy.init_node("detector_node", anonymous=True)
		controller_manager = DetectorManager()
		DetectorServer("detecting", controller_manager)
		rospy.spin()

