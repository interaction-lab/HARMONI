#!/usr/bin/env python

# 1. import libraries
# 2. class name
## a. class attributes
## b. variables initialization
## c. class operations

# 1. import libraries
import rospy
import actionlib

from harmoni_common_msgs.msg import *


# 2. class name
class ActionServer():

    def __init__(self, action_topic):
        # variables initializations
        self.__feedback = harmoniFeedback()
        self.__result = harmoniResult()
        self.action_topic = action_topic
		self.action = actionlib.SimpleActionServer(self.action_topic, harmoniAction, self.execute_goal_callback, False)
		self.action.start()

    def execute_goal_callback(self, goal):
        # action request
        action_name = goal.action
        optional_data = goal.optional_data
        child = goal.child
        condition = goal.condition
        
        #execute the goal


