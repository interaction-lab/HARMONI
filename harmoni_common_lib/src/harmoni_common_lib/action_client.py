#!/usr/bin/env python

# Importing the libraries
import rospy
import roslib
import actionlib
from harmoni_common_msgs.msg import harmoniGoal, harmoniAction


class HarmoniActionClient():
    """
    Controllers and manager are clients.
    This class provides basic client functionality which controller and manager extend,
    including basic type checking, warnings, interrupts, etc.

    """
    def __init__(self):
        """ Initialization of the variables """
        self.init_check_variables_client()

    def init_action_variables(self):
        """ Initizalization or Reset variables """
        self.action_result = {}
        self.action_feedback = {}
        self.action_result = {
            "do_continue": False,
            "message": ""
        }
        self.action_feedback = {
            "state": ""
        }

    def setup_client(self, action_topic, timeout, execute_goal_result_callback, execute_goal_feedback_callback):
        """ Init client action variables and setup clients"""
        timeout = rospy.Duration.from_sec(timeout)
        self.init_action_variables()
        self.action_client = actionlib.SimpleActionClient(action_topic, harmoniAction)
        self.action_client.wait_for_server(timeout)
        self.execute_goal_result_callback = execute_goal_result_callback
        self.execute_goal_feedback_callback = execute_goal_feedback_callback
        return

    def goal_result_callback(self, terminal_state, result):
        """ Save the action result """
        rospy.loginfo("Heard back result from: " + result.action)
        self.action_result["do_continue"] = result.do_continue
        self.action_result["message"] = result.message
        self.execute_goal_result_callback(self.action_result)
        return

    def goal_feedback_callback(self, feedback):
        """ Save the action feedback """
        rospy.loginfo("Heard back feedback from: " + feedback.action)
        self.action_feedback["state"] = feedback.state
        self.execute_goal_feedback_callback(self.action_feedback)
        return

    def feedback_data(self):
        """Return Feedback Data"""
        return(self.action_feedback)

    def result_data(self):
        """Return Result Data"""
        return(self.action_result)

    def send_goal(self, action_goal, optional_data, child, condition, timeout):
        """ Reset of check variables. Send goal and set the time out """
        self.init_check_variables_client()
        goal = harmoniGoal(action=action_goal, optional_data=optional_data, child=child, condition=condition)
        self.action_client.send_goal(goal, done_cb=self.goal_result_callback, feedback_cb=self.goal_feedback_callback)
        self.action_client.wait_for_result(rospy.Duration.from_sec(timeout))
        return
