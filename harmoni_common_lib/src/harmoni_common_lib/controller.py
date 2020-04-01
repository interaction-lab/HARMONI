#!/usr/bin/env python

# Importing the libraries
import rospy
import roslib
from action_client import ActionClient
from action_server import ActionServer


class HarmoniController(ActionClient, ActionServer):
    """
    A control provider receives some request from the manager and send the corresponding 
    request action to the child.
    This class provides basic controller functionality which the subclasses of controller can exploit
    """

    def __init__(self, controller, subclasses):
        """ Initialization of the variables """
        self.controller = controller
        self.subclasses_array = subclasses  # array of the subclasses of the single controller

    def setup_actions(self):
        """ Setup clients of each subclass and the server of the controller"""
        for i in range(0, len(self.subclasses_array)):
            super().setup_client(self.subclasses_array[i])
        super().setup_server(self.controller)
        return

    def setup_conditional_startup(self, condition_event, checked_event):
        """ Set condition for starting the action """
        while condition_event != checked_event:
            rospy.loginfo("Waiting for event to finish")
            rospy.Rate(1)
        rospy.loginfo("Conditional event ended successfully")
        return

    def send_state(self, state):
        """ Send feedback """
        super().send_feedback(state)
        return

    def send_result(self, do_continue, message):
        """ Send result """
        super().send_result(do_continue, message)
        return

    def handle_controller(self, time_out, checked_event):
        """ 
        Receiving the request (server role)
        Check if the goal has been received, if the server received the request
        Get the data from the parent request
        Check if setting up a conditional startup or not
        Sending the goal request to the server (client role)
        """
        while not super()s.check_if_goal_received():
            rospy.loginfo("Waiting for receiving a request goal")
            rospy.Rate(1)
        request_data = super().request_data()
        rospy.loginfo("The request data are:" + str(request_data))
        if request_data.condition != "uncondition":  # check if the action is conditioned by another event or not
            self.setup_conditional_startup(request_data.condition, checked_event)

        rospy.loginfo("Start a goal request to the child")
        super().send_goal(action_goal=request_data.child, optional_data=request_data.optional_data, condition="", time_out=time_out)
        return

    def handle_state(self, handle_function):
        """ Check the feedback state received. Get the data and reset the variables"""
        if super().check_if_feedback_received():
            rospy.loginfo("Received state feedback")
            feedback_data = super().feedback_data()
            super().init_check_variables_client()
            handle_function()
        else:
            return False

    def handle_response(self, handle_function):
        """ Check the response received. Get the data and reset the variables """
        if super().check_if_result_received():
            rospy.loginfo("Received result")
            result_data = super().result_data()
            super().init_check_variables_client()
            handle_function()
        else:
            return False
