#!/usr/bin/env python

# Importing the libraries
import rospy
import roslib
import actionlib
from harmoni_common_msgs.msg import harmoniAction, harmoniFeedback, harmoniResult
from action_client import HarmoniActionClient



class HarmoniActionServer(object):
    """
    Most nodes (both controllers and children) are servers.
    This class provides basic server functionality which controllers and children extend,
    including basic type checking, warnings, interrupts, etc.

    """

    def __init__(self):
        """Initialization"""

    def setup_server(self, action_topic):
        """You must know the action name to set up a server"""
        self.__feedback = harmoniFeedback()
        self.__result = harmoniResult()
        self.action_topic = action_topic
        self.__action_server = actionlib.SimpleActionServer(self.action_topic, harmoniAction, self.goal_received_callback, auto_start=False)
        self.__action_server.start()
        rospy.loginfo("Server starts")
        return

    def goal_received_callback(self, goal):
        """ Callback function, initialize the variables and set the goal to received"""
        self.action_goal = goal.action  # action request
        self.optional_data = goal.optional_data  # input data for the module
        self.child = goal.child  # external module that will accomplish the task
        self.condition = goal.condition  # event condition to wait before starting the action
        rospy.loginfo("The goal is: " + goal.action)
        return

    def send_feedback(self, state):
        """ Send the feedback"""
        self.__feedback.action = self.action_goal
        self.__feedback.state = state
        self.__action_server.publish_feedback(self.__feedback)
        rospy.loginfo("The feedback is:" + self.__feedback.state)
        return

    def send_result(self, do_continue, message):
        """Send the result and action set to succeded"""
        self.__result.action = self.action_goal
        self.__result.do_continue = do_continue
        self.__result.message = message
        self.__action_server.set_succeeded(self.__result)
        rospy.loginfo("The action " + self.__result.action + " have been set to succeded")
        return

class Controller(HarmoniActionServer):
    """
    Testing the callback function
    """

    def __init__(self):
        print("Init Controller")
        return

    def callback_controller(self, request_data):
        print("Handle controller")
        super(Controller, self).goal_received_callback()
        return

    def setup(self):
        print("Setup")
        self.setup_server("controller")
        return

    def callback_result(self):
        print("Result")

    def callback_feedback(self):
        print("Feedback")


if __name__ == "__main__":
    rospy.init_node("test_node")
    contr = Controller()
    contr.setup()
    client = HarmoniActionClient()
    client.setup_client("controller", contr.callback_result(), contr.callback_feedback())
    client.send_goal(action_goal="test", optional_data="ciao", child="", condition="", time_out=20)
    rospy.spin()
    pass
