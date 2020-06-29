#!/usr/bin/env python3

import rospy
import roslib
from actionlib import SimpleActionClient
from harmoni_common_msgs.msg import harmoniGoal, harmoniAction
from harmoni_common_lib.constants import State


class HarmoniActionClient(object):
    """A wrapper around SimpleActionClient that is structured for HARMONI architecture.

    Routers and managers are clients.
    This class provides basic client functionality which routers and managers extend,
    including basic type checking, warnings, interrupts, etc.
    """

    def __init__(self):
        """ Initialization of the variables """
        self._init_action_variables()
        self._set_com_flag_variables()

    def _set_com_flag_variables(self):
        """ Set communication flag variables to default: False """
        self.feedback_received = False
        self.result_received = False
        return

    def _init_action_variables(self):
        """ Initizalize or reset action msg variables """
        self.action_result = {}
        self.action_feedback = {}
        self.action_result = {"do_action": False, "message": ""}
        self.action_feedback = {"state": ""}

    def _result_callback(self, terminal_state, result):
        """Save the action result """
        rospy.loginfo(f"(Client) Heard back result on actiontype: {result.action_type}")
        rospy.loginfo(f"(Client) Result was : {result.do_action}")
        if len(result.message) < 500:
            rospy.loginfo(f"(Client) Message was : {result.message}")
        else:
            rospy.loginfo(f"(Client) Message was : (too long to display)")
        if self.result_received:
            rospy.loginfo(f"(Client) Result was already recieved")
        self.action_result["do_action"] = result.do_action
        self.action_result["message"] = result.message
        if self.execute_goal_result_callback:
            self.execute_goal_result_callback(self.action_result)
        self.result_received = True
        return

    def _feedback_callback(self, feedback):
        """Save the action feedback """
        rospy.logdebug(
            f"(Client) Heard back feedback from type: {feedback.action_type}"
        )
        rospy.logdebug(f"(Client) State was: {feedback.state}")
        self.action_feedback["state"] = feedback.state
        if self.execute_goal_feedback_callback:
            self.execute_goal_feedback_callback(self.action_feedback)
        self.feedback_received = True
        return

    def setup_client(
        self,
        action_type_name,
        execute_goal_result_callback=None,
        execute_goal_feedback_callback=None,
        wait=True,
    ):
        """Init client action variables and setup client

        Args:
            action_type_name (str): The name of the action server the client should connect to. #TODO rename. this is just the server name.
            execute_goal_result_callback (func): [Optional] A callback function
                handle for action results (done state).
            execute_goal_feedback_callback (func): [Optional] A callback function
                handle for action feedback.
            wait (bool): Indicates whether
        """

        self._init_action_variables()

        self.action_client = SimpleActionClient(action_type_name, harmoniAction)
        if wait:
            rospy.loginfo(
                "action_client waiting for {} server to connect.".format(
                    action_type_name
                )
            )
            self.action_client.wait_for_server()

        self.execute_goal_result_callback = execute_goal_result_callback
        self.execute_goal_feedback_callback = execute_goal_feedback_callback
        rospy.loginfo("action_client {} server connected.".format(action_type_name))
        return

    def get_feedback_data(self):
        """Return Feedback Data

        Returns:
            dictionary: feedback message content
        """
        return self.action_feedback

    def get_result_data(self):
        """Return Result Data

        Returns:
            dictionary: result message content
        """
        return self.action_result

    def send_goal(
        self,
        action_goal,
        optional_data="",
        child_server="",
        condition="",
        time_out=600,
        wait=True,
    ):
        """Sends a goal to the action server tied to this client.

        Args:
            action_goal (harmoniGoal): The goal object given to the action server
            optional_data:
        Reset of check variables. Send goal and set the time out
        """

        self._set_com_flag_variables()
        goal = harmoniGoal(
            action_type=action_goal,
            optional_data=optional_data,
            child_server=child_server,
            condition=condition,
        )
        rospy.loginfo("(Client) Sending Goal.")
        self.action_client.send_goal(
            goal, done_cb=self._result_callback, feedback_cb=self._feedback_callback
        )
        rospy.loginfo("(Client) Goal Sent")
        if wait:
            rospy.loginfo("(Client) Waiting for return.")
            self.action_client.wait_for_result(rospy.Duration.from_sec(time_out))
            rospy.loginfo("(Client) done waiting.")
        else:
            rospy.loginfo("(Client) Not waiting for result.")

        return

    def _result_callback(self, terminal_state, result):
        """ Save the action result and perform the callback if set """
        rospy.loginfo("Heard result from: %i" % result.action_type)
        self._action_result["do_action"] = result.do_action
        self._action_result["message"] = result.message

        if self.execute_goal_result_callback:
            self.execute_goal_result_callback(self._action_result)

        self.result_received = True
        return

    def _feedback_callback(self, feedback):
        """ Save the action feedback and perform the callback if set """
        rospy.logdebug("Heard back feedback from: %i" % feedback.action_type)
        self._action_feedback["state"] = feedback.state

        if self.execute_goal_feedback_callback:
            self.execute_goal_feedback_callback(self._action_feedback)

        self.feedback_received = True
        return

    def _result_callback(self, terminal_state, result):
        """ Save the action result and perform the callback if set """
        rospy.loginfo("Heard result from: %i" % result.action_type)
        self._action_result["do_action"] = result.do_action
        self._action_result["message"] = result.message

        if self.execute_goal_result_callback:
            self.execute_goal_result_callback(self._action_result)

        self.result_received = True
        return

    def _feedback_callback(self, feedback):
        """ Save the action feedback and perform the callback if set """
        rospy.logdebug("Heard back feedback from: %i" % feedback.action_type)
        self._action_feedback["state"] = feedback.state

        if self.execute_goal_feedback_callback:
            self.execute_goal_feedback_callback(self._action_feedback)

        self.feedback_received = True
        return
