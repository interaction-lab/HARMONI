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
        self.init_check_variables_server()

    def setup_server(self, action_topic, execute_goal_received_callback):
        """You must know the action name to set up a server"""
        self.__feedback = harmoniFeedback()
        self.__result = harmoniResult()
        self.action_topic = action_topic
        self.action = actionlib.SimpleActionServer(self.action_topic, harmoniAction, self.goal_received_callback, auto_start=False)
        self.action.start()
        rospy.loginfo("Server starts")
        self.execute_goal_received_callback = execute_goal_received_callback
        return

    def goal_received_callback(self, goal):
        """ Callback function, initialize the variables and set the goal to received"""
        self.action_goal = goal.action  # action request
        self.optional_data = goal.optional_data  # input data for the module
        self.child = goal.child  # external module that will accomplish the task
        self.condition = goal.condition  # event condition to wait before starting the action
        print(goal)
        rospy.loginfo("The goal is: " + goal.action)
        self.goal_received = True
        self.execute_goal_received_callback(goal)
        return

    def preemption_status(self):
        if self.action_goal.is_preempt_requested():
            rospy.loginfo(self.action_goal + " Action Preemepted")
            self.action_goal.set_preempted()
            preempted = True
        return preempted

    def request_data(self):
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
        self.action_goal.publish_feedback(self.__feedback)
        rospy.loginfo("The feedback is:" + self.__feedback.state)
        return

    def send_result(self, do_continue, message):
        """Send the result and action set to succeded"""
        self.__result.action = self.action_goal
        self.__result.do_continue = do_continue
        self.__result.message = message
        self.action_goal.set_succeeded(self.__result)
        rospy.loginfo("The action " + self.__result.action + " have been set to succeded")
        return
