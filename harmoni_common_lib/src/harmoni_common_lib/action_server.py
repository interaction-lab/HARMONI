#!/usr/bin/env python

# Importing the libraries
import rospy
import roslib
import actionlib
from harmoni_common_msgs.msg import harmoniAction, harmoniFeedback, harmoniResult


class HarmoniActionServer():
    """
    Most nodes (both controllers and children) are servers.
    This class provides basic server functionality which controllers and children extend,
    including basic type checking, warnings, interrupts, etc.

    """

    def __init__(self):
        self.init_check_variables()

    def init_check_variables(self):
        """Reset initialized variables"""
        self.goal_received = False
        return

    def setup_server(self, action_topic):
        """You must know the action name to set up a server"""
        self.__feedback = harmoniFeedback()
        self.__result = harmoniResult()
        self.action_topic = action_topic
        self.action = actionlib.SimpleActionServer(self.action_topic, harmoniAction, self.goal_received_callback, auto_start=False)
        self.action.start()
        return

    def goal_received_callback(self, goal):
        # Initialize request body variables
        self.action_goal = goal.action  # action request
        self.optional_data = goal.optional_data  # input data for the module
        self.child = goal.child  # external module that will accomplish the task
        self.condition = goal.condition  # event condition to wait before starting the action
        # Set goal received
        rospy.loginfo("The goal is: " + goal.action)
        self.goal_received = True
        return

    def preemption_status(self):
        if self.action_goal.is_preempt_requested():
            rospy.loginfo(self.action_goal + " Action Preemepted")
            self.action_goal.set_preempted()
            preempted = True
        return preempted

    def check_if_goal_received(self):
        if self.goal_received:
            received = True
        else:
            received = False
        rospy.loginfo("The goal has been received:" + str(received))
        return received

    def request_data(self):
        """Return Request Data"""
        request_data = {}
        request_data["optional_data"] = self.optional_data
        request_data["child"] = self.child
        request_data["condition"] = self.condition
        return(request_data)

    def send_feedback(self, state):
        self.__feedback.action = self.action_goal
        self.__feedback.state = state
        # Send the feedback
        self.action_goal.publish_feedback(self.__feedback)
        rospy.loginfo("The feedback is:" + self.__feedback.state)
        return

    def send_result(self, do_continue, message):
        self.__result.action = self.action_goal
        self.__result.do_continue = do_continue
        self.__result.message = message
        # Action set to succeded
        self.action_goal.set_succeeded(self.__result)
        rospy.loginfo("The action " + self.__result.action + " have been set to succeded")
        return
