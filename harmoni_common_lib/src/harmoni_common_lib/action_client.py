#!/usr/bin/env python

# Importing the libraries
import rospy
import actionlib
from harmoni_common_msgs.msg import *

class ActionClient():

    def __init__(self):
        # Initialization of the variables
        self.action_result = {}
        self.action_feedback = {}

    def setup_client(self, action_goal):
        # Setup clients
        self.action_client = actionlib.SimpleActionClient(action_topic, harmoniAction)
        self.action_client.wait_for_server()
        self.action_result = {
            "do_continue": False,
            "message": ""}
        }
        self.action_feedback = {
            "state":""
        }
        return

    def goal_result_callback(self, terminal_stale, result):
        # Save the action result
        rospy.loginfo("Heard back result from: " + result.action)
        self.action_result["do_continue"] = result.do_continue
        self.action_result["message"] = result.message
        return

    def goal_feedback_callback(self, feedback):
        # Save the action feedback
        rospy.loginfo("Heard back feedback from: " + feedback.action)
        self.action_feedback["state"] = feedback.state
        return

    def send_goal(self, action_goal, optional_data, child, condition, time_out):
        # Send goal and the time_out 
        goal = harmoniGoal(action = action_goal, optional_data= optional_data, child = child, condition = condition)
        self.action_client.send_goal(goal, done_cb = self.goal_result_callback, feedback_cb = self.goal_feedback_callback)
        self.action_client.wait_for_result(time_out)
        return

    