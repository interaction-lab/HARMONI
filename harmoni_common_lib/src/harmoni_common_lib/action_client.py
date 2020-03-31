#!/usr/bin/env python

# Importing the libraries
import rospy
import roslib
import actionlib
from harmoni_common_msgs.msg import harmoniGoal, harmoniAction

class ActionClient():

    def __init__(self):
        # Initialization of the variables
        self.init_check_variables()
       
    def init_check_variables(self):
        # Initizalization or Reset of check variables
        self.result_received = False
        self.feedback_received = False
        return

    def init_action_variables(self):
        # Initizalization or Reset variables
        self.action_result = {}
        self.action_feedback = {}
        self.action_result = {
            "do_continue": False,
            "message": ""}
        }
        self.action_feedback = {
            "state":""
        }

    def setup_client(self, action_goal):
        # Init client action variables
        self.init_action_variables()
        # Setup clients
        self.action_client = actionlib.SimpleActionClient(action_topic, harmoniAction)
        self.action_client.wait_for_server()
        return

    def goal_result_callback(self, terminal_stale, result):
        # Save the action result
        rospy.loginfo("Heard back result from: " + result.action)
        self.action_result["do_continue"] = result.do_continue
        self.action_result["message"] = result.message
        self.result_received = True
        return
        

    def goal_feedback_callback(self, feedback):
        # Save the action feedback
        rospy.loginfo("Heard back feedback from: " + feedback.action)
        self.action_feedback["state"] = feedback.state
        self.feedback_received = True
        return

    def check_if_feedback_received(self):
        # Check if feedback received
        if self.feedback_received = True:
            feedback = True
            self.feedback_received = False
        else:
            feedback = False
        return feedback  

    def check_if_result_received(self):
        # Check if result received
        if self.result_received = True:
            result = True
        else:
            result = False
        return result

    def get_feedback_data(self):
        return(self.action_feedback) 

    def get_result_data(self):
        return(self.action_result) 

    def send_goal(self, action_goal, optional_data, child, condition, time_out):
        # Reset of check variables
        self.result_received = False
        self.feedback_received = False
        # Send goal and the time_out 
        goal = harmoniGoal(action = action_goal, optional_data= optional_data, child = child, condition = condition)
        self.action_client.send_goal(goal, done_cb = self.goal_result_callback, feedback_cb = self.goal_feedback_callback)
        self.action_client.wait_for_result(time_out)
        return

        



    