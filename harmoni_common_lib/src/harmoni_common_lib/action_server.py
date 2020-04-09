#!/usr/bin/env python

# Importing the libraries
import rospy
import roslib
import actionlib
from harmoni_common_msgs.msg import harmoniAction, harmoniFeedback, harmoniResult


class HarmoniActionServer(object):
    """
    Most nodes (both controllers and children) are servers.
    This class provides basic server functionality which controllers and children extend,
    including basic type checking, warnings, interrupts, etc.

    """

    def __init__(self):
        """Initialization"""

    def setup_server(self, action_topic, execute_goal_received_callback):
        """You must know the action name to set up a server"""
        self.__feedback = harmoniFeedback()
        self.__result = harmoniResult()
        self.action_topic = action_topic
        self.__action_server = actionlib.SimpleActionServer(self.action_topic, harmoniAction, self.goal_received_callback, auto_start=False)
        self.__action_server.start()
        rospy.loginfo("Server starts")
        self.execute_goal_received_callback = execute_goal_received_callback
        return

    def goal_received_callback(self, goal):
        """ Callback function, initialize the variables and set the goal to received"""
        self.action_goal = goal.action  # action request
        self.optional_data = goal.optional_data  # input data for the module
        self.child = goal.child  # external module that will accomplish the task
        self.condition = goal.condition  # event condition to wait before starting the action
        rospy.loginfo("The goal is: " + goal.action)
        self.goal_received = True
        self.execute_goal_received_callback(goal)
        return

    def get_preemption_status(self):
        preempted = False
        if self.__action_server.is_preempt_requested():
            rospy.loginfo(self.action_goal + " Action Preemepted")
            self.__action_server.set_preempted()
            preempted = True
        return preempted

    def get_goal_received(self):
        if self.goal_received:
            received = True
            rospy.loginfo("The goal has been received:" + str(received))
        else:
            received = False
        return received

    def get_request_data(self):
        """Return Request Data"""
        request_data = {}
        request_data["optional_data"] = self.optional_data
        request_data["child"] = self.child
        request_data["condition"] = self.condition
        return(request_data)

    def send_feedback(self, state):
        """ Send the feedback"""
        self.__feedback.action = self.action_goal
        self.__feedback.state = state
        self.__action_server.publish_feedback(self.__feedback)
        rospy.logdebug("The feedback is " + str(self.__feedback.state))
        return

    def send_result(self, do_action, message):
        """Send the result and action set to succeded"""
        self.__result.action = self.action_goal
        self.__result.do_action = do_action
        self.__result.message = message
        self.__action_server.set_succeeded(self.__result)
        rospy.loginfo("The action " + self.__result.action + " have been set to succeded")
        return
