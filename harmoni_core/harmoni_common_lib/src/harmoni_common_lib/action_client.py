#!/usr/bin/env python3

import rospy
import threading

from actionlib_msgs.msg import GoalStatus
from actionlib.action_client import ActionClient, CommState, get_name_of_constant

import roslib

from harmoni_common_msgs.msg import harmoniGoal, harmoniAction
from harmoni_common_lib.constants import State


class SimpleGoalState:
    PENDING = 0
    ACTIVE = 1
    DONE = 2


SimpleGoalState.to_string = classmethod(get_name_of_constant)


class HarmoniActionClient(object):
    """A wrapper around SimpleActionClient that is structured for HARMONI architecture.

    Credit goes to https://github.com/ros/actionlib/blob/noetic-devel/actionlib/src/actionlib/simple_action_client.py
    for the simple_action_client this is based on.

    Routers and managers are clients.
    This class provides basic client functionality which routers and managers extend,
    including basic type checking, warnings, interrupts, etc.
    """

    def __init__(self, name):
        # @brief Constructs a SimpleActionClient and opens connections to an ActionServer.
        #
        # @param ns The namespace in which to access the action.  For
        # example, the "goal" topic should occur under ns/goal
        #
        # @param ActionSpec The *Action message type.  The SimpleActionClient
        # will grab the other message types from this type.
        self.name = name
        self.action_result = {"do_action": None, "message": None}
        self.action_feedback = {"state": None}
        self.active_cb = None
        return

    # @brief Sends a goal to the ActionServer, and also registers callbacks
    #
    # If a previous goal is already active when this is called. We simply forget
    # about that goal and start tracking the new goal. No cancel requests are made.
    #
    # @param done_cb Callback that gets called on transitions to
    # Done.  The callback should take two parameters: the terminal
    # state (as an integer from actionlib_msgs/GoalStatus) and the
    # result.
    #
    # @param active_cb   No-parameter callback that gets called on transitions to Active.
    #
    # @param feedback_cb Callback that gets called whenever feedback
    # for this goal is received.  Takes one parameter: the feedback.
    # destroys the old goal handle
    def send_goal(
        self, action_goal, optional_data="", condition="", time_out=60, wait=True,
    ):
        """Sends a goal to the action server tied to this client.

        Args:
            action_goal (harmoniGoal): The goal object given to the action server
            optional_data:
        Reset of check variables. Send goal and set the time out
        """
        if len(optional_data) < 50:
            rospy.loginfo(f"Client ({self.name}) Sending goal. Data: {optional_data}")
        else:
            rospy.loginfo(f"Client ({self.name}) Sending Goal.")

        self.stop_tracking_goal()
        goal = harmoniGoal(
            action_type=action_goal, optional_data=optional_data, condition=condition,
        )

        self.simple_state = SimpleGoalState.PENDING
        self.goal_handler = self.action_client.send_goal(
            goal, self._handle_transition, self._handle_feedback
        )

        rospy.loginfo(f"Client ({self.name}) Goal Sent")
        if wait:
            rospy.loginfo(f"Client ({self.name}) Waiting for return: {time_out}")
            self.wait_for_result(rospy.Duration(time_out))
            # self.wait_for_result(rospy.Duration.from_sec(time_out))
            rospy.loginfo(f"Client ({self.name}) done waiting.")

        else:
            rospy.loginfo(f"Client ({self.name}) Not waiting for result.")
        return

    def setup_client(
        self, action_type, result_cb_fnc=None, feedback_cb_fnc=None, wait=True,
    ):
        """Init client action variables and setup client

        Args:
            action_type (str): The name of the action server the client should connect to. #TODO rename. this is just the server name.
            result_cb_fnc (func): [Optional] A callback function
                handle for action results (done state).
            feedback_cb_fnc (func): [Optional] A callback function
                handle for action feedback.
            wait (bool): Indicates whether or not to wait
        """

        self.action_client = ActionClient(action_type, harmoniAction)
        self.simple_state = SimpleGoalState.DONE
        self.goal_handler = None
        self.done_condition = threading.Condition()

        if wait:
            rospy.loginfo(f"action_client waiting for {action_type} server to connect.")
            self.action_client.wait_for_server()

        self.result_cb_fnc = result_cb_fnc
        self.feedback_cb_fnc = feedback_cb_fnc
        rospy.loginfo(f"action_client {action_type} server connected.")
        return

    def wait_for_server(self, timeout=rospy.Duration()):
        # @brief Blocks until the action server connects to this client
        #
        # @param timeout Max time to block before returning. A zero
        # timeout is interpreted as an infinite timeout.
        #
        # @return True if the server connected in the allocated time. False on timeout
        return self.action_client.wait_for_server(timeout)

    def send_goal_and_wait(
        self, goal, execute_timeout=rospy.Duration(), preempt_timeout=rospy.Duration()
    ):
        # @brief Sends a goal to the ActionServer, waits for the goal to complete, and
        # preempts goal is necessary
        #
        # If a previous goal is already active when this is called. We simply forget
        # about that goal and start tracking the new goal. No cancel requests are made.
        #
        # If the goal does not complete within the execute_timeout, the goal gets preempted
        #
        # If preemption of the goal does not complete withing the preempt_timeout, this
        # method simply returns
        #
        # @param execute_timeout The time to wait for the goal to complete
        #
        # @param preempt_timeout The time to wait for preemption to complete
        #
        # @return The goal's state.
        self.send_goal(goal)
        if not self.wait_for_result(execute_timeout):
            # preempt action
            rospy.logdebug("Canceling goal")
            self.cancel_goal()
            if self.wait_for_result(preempt_timeout):
                rospy.logdebug(
                    "Preempt finished within specified preempt_timeout [%.2f]",
                    preempt_timeout.to_sec(),
                )
            else:
                rospy.logdebug(
                    "Preempt didn't finish specified preempt_timeout [%.2f]",
                    preempt_timeout.to_sec(),
                )
        return self.get_state()

    def wait_for_result(self, timeout=rospy.Duration()):
        # @brief Blocks until this goal transitions to done
        #
        # @param timeout Max time to block before returning. A zero timeout is
        # interpreted as an infinite timeout.
        #
        # @return True if the goal finished. False if the goal didn't finish within
        # the allocated timeout

        self.simple_state == SimpleGoalState.PENDING
        if not self.goal_handler:
            rospy.logerr("Called wait_for_result when no goal exists")
            return False

        timeout_time = rospy.get_rostime() + timeout
        loop_period = rospy.Duration(0.1)
        with self.done_condition:
            while not rospy.is_shutdown():
                time_left = timeout_time - rospy.get_rostime()
                if timeout > rospy.Duration(0.0) and time_left <= rospy.Duration(0.0):
                    rospy.loginfo("Time break")
                    break

                if self.simple_state == SimpleGoalState.DONE:
                    rospy.loginfo("state break")
                    break

                if time_left > loop_period or timeout == rospy.Duration():
                    time_left = loop_period

                self.done_condition.wait(time_left.to_sec())

        return self.simple_state == SimpleGoalState.DONE

    def get_result(self):
        # @brief Gets the Result of the current goal
        if not self.goal_handler:
            rospy.logerr("Called get_result when no goal is running")
            return None

        return self.goal_handler.get_result()

    def get_state(self):
        # @brief Get the state information for this goal
        #
        # Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED,
        # PREEMPTED, ABORTED, SUCCEEDED, LOST.
        #
        # @return The goal's state. Returns LOST if this
        # SimpleActionClient isn't tracking a goal.
        if not self.goal_handler:
            return GoalStatus.LOST
        status = self.goal_handler.get_goal_status()

        if status == GoalStatus.RECALLING:
            status = GoalStatus.PENDING
        elif status == GoalStatus.PREEMPTING:
            status = GoalStatus.ACTIVE

        return status

    def get_goal_status_text(self):
        # @brief Returns the current status text of the goal.
        #
        # The text is sent by the action server. It is designed to
        # help debugging issues on the server side.
        #
        # @return The current status text of the goal.
        if not self.goal_handler:
            rospy.logerr("Called get_goal_status_text when no goal is running")
            return "ERROR: Called get_goal_status_text when no goal is running"

        return self.goal_handler.get_goal_status_text()

    def cancel_all_goals(self):
        # @brief Cancels all goals currently running on the action server
        #
        # This preempts all goals running on the action server at the point that
        # this message is serviced by the ActionServer.
        self.action_client.cancel_all_goals()

    def cancel_goals_at_and_before_time(self, time):
        # @brief Cancels all goals prior to a given timestamp
        #
        # This preempts all goals running on the action server for which the
        # time stamp is earlier than the specified time stamp
        # this message is serviced by the ActionServer.
        self.action_client.cancel_goals_at_and_before_time(time)

    def cancel_goal(self):
        # @brief Cancels the goal that we are currently pursuing
        if self.goal_handler:
            self.goal_handler.cancel()

    def stop_tracking_goal(self):
        # @brief Stops tracking the state of the current goal. Unregisters this goal's callbacks
        #
        # This is useful if we want to make sure we stop calling our callbacks before sending a new goal.
        # Note that this does not cancel the goal, it simply stops looking for status info about this goal.
        self.goal_handler = None

    def _handle_transition(self, gh):
        comm_state = gh.get_comm_state()

        error_msg = (
            "Received comm state %s when in simple state %s with SimpleActionClient in NS %s"
            % (
                CommState.to_string(comm_state),
                SimpleGoalState.to_string(self.simple_state),
                rospy.resolve_name(self.action_client.ns),
            )
        )

        if comm_state == CommState.ACTIVE:
            if self.simple_state == SimpleGoalState.PENDING:
                self.simple_state = SimpleGoalState.ACTIVE
                if self.active_cb:
                    self.active_cb()
            elif self.simple_state == SimpleGoalState.DONE:
                rospy.logerr(error_msg)
        elif comm_state == CommState.RECALLING:
            if self.simple_state != SimpleGoalState.PENDING:
                rospy.logerr(error_msg)
        elif comm_state == CommState.PREEMPTING:
            if self.simple_state == SimpleGoalState.PENDING:
                self.simple_state = SimpleGoalState.ACTIVE
                if self.active_cb:
                    self.active_cb()
            elif self.simple_state == SimpleGoalState.DONE:
                rospy.logerr(error_msg)
        elif comm_state == CommState.DONE:
            if self.simple_state in [SimpleGoalState.PENDING, SimpleGoalState.ACTIVE]:
                self.simple_state = SimpleGoalState.DONE
                if self._result_cb:
                    self._result_cb(gh.get_goal_status(), gh.get_result())
                with self.done_condition:
                    self.done_condition.notifyAll()
            elif self.simple_state == SimpleGoalState.DONE:
                rospy.logerr("SimpleActionClient received DONE twice")

    def _handle_feedback(self, gh, feedback):
        if not self.goal_handler:
            # this is not actually an error - there can be a small window in which old feedback
            # can be received between the time this variable is reset and a new goal is
            # sent and confirmed
            return
        if gh != self.goal_handler:
            rospy.logerr(
                "Got a feedback callback on a goal handle that we're not tracking. %s vs %s"
                % (
                    self.goal_handler.comm_state_machine.action_goal.goal_id.id,
                    gh.comm_state_machine.action_goal.goal_id.id,
                )
            )
            return
        rospy.logdebug(f"Client ({self.name}) Heard back feedback")
        rospy.logdebug(f"Client ({self.name}) State was: {feedback.state}")
        self.action_feedback["state"] = feedback.state
        if self.feedback_cb_fnc:
            self.feedback_cb_fnc(self.action_feedback)
        return

    def _result_cb(self, terminal_state, result):
        """Save the action result """
        rospy.loginfo(f"Client ({self.name}) Heard back result")
        rospy.loginfo(f"Client ({self.name}) Do action?... {result.do_action}")
        if result.message:
            if len(result.message) < 500:
                rospy.loginfo(f"Client ({self.name}) Message was : {result.message}")
            else:
                rospy.loginfo(f"Client ({self.name}) Message was : (too long to display)")

        self.action_result["service"] = self.name
        self.action_result["do_action"] = result.do_action
        self.action_result["message"] = result.message
        if self.result_cb_fnc:
            self.result_cb_fnc(self.action_result)
        return

