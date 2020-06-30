#!/usr/bin/env python3

import rospy
import roslib
import actionlib
from harmoni_common_msgs.msg import harmoniAction, harmoniFeedback, harmoniResult
from harmoni_common_lib.simple_action_server import SimpleActionServer


class HarmoniActionServer(object):
    """A wrapper around SimpleActionServer that is structured for HARMONI architecture.

    Most nodes (both routers and children) are servers.
    This class provides basic server functionality which routers and children extend,
    including basic type checking, warnings, interrupts, etc.py

    TODO: Implement priority handling for received goals instead of preempting current goals.

    """

    def _init_(self):
        """No initialization required, server setup must be called explicitly """

    # You must know the action name to set up a server
    def setup_server(self, action_topic, execute_goal_received_callback):
        """ Set up action server with appropriate action topic"""
        self._feedback = harmoniFeedback()
        self._result = harmoniResult()
        self.action_topic = action_topic
        self._action_server = SimpleActionServer(
            self.action_topic,
            harmoniAction,
            self._goal_received_callback,
            auto_start=False,
        )
        self._action_server.start()
        rospy.loginfo("(Server) Startup")
        self.execute_goal_received_callback = execute_goal_received_callback
        rospy.loginfo(f"{action_topic} server started")
        return

    def _goal_received_callback(self, goal):
        """ Save the goal data, set the goal to received, and execute the child callback """
        self.optional_data = goal.optional_data  # input data for the module
        self.action_goal = goal.action_type  # action request
        self.child = goal.child_server  # external module that will accomplish the task
        self.condition = (
            goal.condition
        )  # event condition to wait before starting the action
        rospy.loginfo(
            f"(Server) The goal is a {goal.action_type} request for {goal.child_server}"
        )
        self.goal_received = True
        rospy.loginfo("The goal is: %i" % goal.action_type)
        # Perform the callback set by child
        self.execute_goal_received_callback(goal)
        return

    def get_preemption_status(self):
        preempted = False
        if self._action_server.is_preempt_requested():
            rospy.loginfo(f"(Server) {self.action_goal} Action Preemepted")
            self._action_server.set_preempted()
            preempted = True
        return preempted

    def get_goal_received(self):
        if self.goal_received:
            received = True
            rospy.loginfo("(Server) The goal has been received:" + str(received))
        else:
            received = False
        return received

    def get_request_data(self):
        """Return Request Data"""
        request_data = {}
        request_data["optional_data"] = self.optional_data
        request_data["child_server"] = self.child
        request_data["condition"] = self.condition
        return request_data

    def send_feedback(self, state):
        """ Send the state as feedback"""
        self._feedback.action_type = self.action_goal
        self._feedback.state = state
        self._action_server.publish_feedback(self._feedback)
        rospy.logdebug("(Server) The feedback is " + str(self._feedback.state))
        return

    def send_result(self, do_action, message):
        """Send the result and action set to succeded"""
        self._result.action_type = self.action_goal
        self._result.do_action = do_action
        self._result.message = message
        self._action_server.set_succeeded(self._result)
        rospy.loginfo(
            f"(Server) sending result {do_action} to actiontype {self.action_goal}"
        )
        return

    # is this useful?
    def get_preemption_status(self):
        """ Check preemption status on action server and log if preempted"""
        if self._action_server.is_preempt_requested():
            rospy.loginfo(self.action_goal + " Action Preemepted")
            self._action_server.set_preempted()
            return True
        else:
            return False

    # is this useful?
    def get_goal_received(self):
        """ Check if goal has been recieved and log if true"""
        if self.goal_received:
            rospy.loginfo(f"The goal for {self.action_goal} has been received")
            return True
        else:
            return False
