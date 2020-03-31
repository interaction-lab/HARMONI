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

    def setup_conditional_startup(self, condition_event, checked_event):
        # Set condition for starting the action
        while condition_event != checked_event:
            rospy.loginfo("Waiting for event to finish")
            rospy.Rate(1)  
        rospy.loginfo("Conditional event ended successfully")      
        return

    def send_state(self, state):
        # Send feedback
        self.server.send_feedback(state)
        
        
    def send_response(self, do_continue, message):
        # Send response
        self.server.send_result(do_continue, message)

    def handle_controller(self, time_out, checked_event):
        # Receiving the request (server role)
        ## Check if the goal has been received, if the server received the request
        while not self.server.check_if_goal_received():
            rospy.loginfo("Waiting for receiving a request goal")
            rospy.Rate(1)
        ## Get the data from the parent request
        request_data = self.server.get_request_data
        rospy.loginfo("The request data are:" + str(request_data))
        ## Check if setting up a conditional start up or not
        if request_data.condition != "uncondition": # check if the action is conditioned by another event or not
            self.setup_conditional_startup(request_data.condition, checked_event)
        
        # Sending the request (client role)
        ## Send the goal request to the client
        rospy.loginfo("Start a goal request to the child")
        self.client.send_goal(action_goal = request_data.child, optional_data = request_data.optional_data, condition = "", time_out = time_out)
        return

    def handle_state(self, handle_function):
        # Check the feedback state received 
        if self.client.check_if_feedback_received():
            rospy.loginfo("Received state feedback")
            # Get the feedback data
            feedback_data = self.client.get_feedback_data()
            # Reset feedback variable
            self.client.init_check_variables()
            handle_function()
        else:
            return False

    def handle_response(self, handle_function):
        # Check the response received
        if self.client.check_if_result_received():
            rospy.loginfo("Received result")
            # Get the result data
            result_data = self.client.get_result_data()
            # Reset result variable
            self.client.init_check_variables()
            handle_function()
        else:
            return False
            