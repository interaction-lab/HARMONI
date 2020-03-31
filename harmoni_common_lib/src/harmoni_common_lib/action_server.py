#!/usr/bin/env python

# Importing the libraries
import rospy
import actionlib
from harmoni_common_msgs.msg import *


class ActionServer():

    def __init__(self):
        # Initialization of the variables
        self.__feedback = harmoniFeedback()
        self.__result = harmoniResult()
        self.goal_received = False
        
    def setup_server(self, action_topic):
        # Setup the server
        self.action_topic = action_topic
        self.action = actionlib.SimpleActionServer(self.action_topic, harmoniAction, self.goal_received_callback, False)
        self.action.start()
        return

    def goal_received_callback(self, goal):
        # Initialize request body variables
        self.action_goal = goal.action ## action request
        self.optional_data = goal.optional_data ## input data for the module
        self.child = goal.child ## external module that will accomplish the task 
        self.condition = goal.condition ## event condition to wait before starting the action
        # Set goal received
        self.goal_received = True
        
    def check_if_goal_received(self):
        if self.goal_received:
            received = True
        else:
            received = False
        return received

    def get_request_data(self):
        # Get the data of the action request, when the goal has been received successfully
        return(self.optional_data, self.child, self.condition)

    def check_if_preempt(self):
        success = True
        if self.action_goal.is_preempt_requested():
            rospy.loginfo(self.action_goal + " Action Preempted")
            self.action_goal.set_preempted()
            success = False
        return success

    def send_feedback(self, state):
        self.__feedback.action = self.action_goal
        self.__feedback.state = state
        # Send the feedback
        self.action_goal.publish_feedback(self._feedback)
        return
        
    def send_result(self, do_continue, message):
        self.__result.action = self.action_goal
        self.__result.do_continue = do_continue
        self.__result.message = message
        # Action set to succeded
        self.action_goal.set_succeeded(self.__result)
        # Received goal set to False
        self.goal_received = False
        return
