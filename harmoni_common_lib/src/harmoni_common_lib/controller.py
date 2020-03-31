#!/usr/bin/env python

# Importing the libraries
import rospy
import roslib
from action_client import ActionClient
from action_server import ActionServer

class Controller():

    def __init__(self, controller, subclasses):
        # Initialization of the inherite classes
        self.client = ActionClient() 
        self.server = ActionServer()
        # Initialization of the variables
        self.controller = controller
        self.subclasses_array = subclasses # array of the subclasses of the single controller

    def setup_actions(self):
        # Setup clients of each subclass
        for i in range(0, len(self.subclasses_array)):
            self.client.setup_client(self.subclasses_array[i])
        # Setup server of the controller
        self.server.setup_server(self.controller)
        return

    def setup_conditional_startup(self, condition):
        # Set condition for starting the action
        return

    def handle_controller(self, time_out):
        # Check if the goal has been received
        while not self.server.check_if_goal_received():
            rospy.loginfo("Waiting for receiving a goal")
            rospy.Rate(1)
        # Get the data from the parent request
        request_data = self.server.get_request_data
        rospy.loginfo("The request data are:" + str(request_data))
        # Check if setting up a conditional start up or not
        if request_data.condition != "uncondition": # check if the action is conditioned by another event or not
            self.setup_conditional_startup(condition)
        # Send the goal request to the client
        self.client.send_goal(action_goal = request_data.child, optional_data = request_data.optional_data, condition = "", time_out = time_out)
        return

    