#!/usr/bin/env python3

import rospy
import threading

from actionlib_msgs.msg import GoalStatus
from actionlib.action_client import ActionClient, CommState, get_name_of_constant

import roslib

# from actionlib import SimpleActionClient
from harmoni_common_lib.simple_action_client import SimpleActionClient
from harmoni_common_msgs.msg import harmoniGoal, harmoniAction
from harmoni_common_lib.constants import State


class SimpleGoalState:
    PENDING = 0
    ACTIVE = 1
    DONE = 2


SimpleGoalState.to_string = classmethod(get_name_of_constant)


class HarmoniActionClient(object):
    """A wrapper around SimpleActionClient that is structured for HARMONI architecture.

    Routers and managers are clients.
    This class provides basic client functionality which routers and managers extend,
    including basic type checking, warnings, interrupts, etc.
    """

    ## @brief Constructs a SimpleActionClient and opens connections to an ActionServer.
    ##
    ## @param ns The namespace in which to access the action.  For
    ## example, the "goal" topic should occur under ns/goal
    ##
    ## @param ActionSpec The *Action message type.  The SimpleActionClient
    ## will grab the other message types from this type.
    def __init__(self):
        return

    def _result_callback(self, terminal_state, result):
        """Save the action result """
        rospy.loginfo(f"(Client) Heard back result")
        rospy.loginfo(f"(Client) Result was : {result.do_action}")
        if len(result.message) < 500:
            rospy.loginfo(f"(Client) Message was : {result.message}")
        else:
            rospy.loginfo(f"(Client) Message was : (too long to display)")
        # if self.result_received:
        #     rospy.loginfo(f"(Client) Result was already recieved")
        self.action_result["do_action"] = result.do_action
        self.action_result["message"] = result.message
        if self.execute_goal_result_callback:
            self.execute_goal_result_callback(self.action_result)
        # self.result_received = True
        return

    def _feedback_callback(self, feedback):
        """Save the action feedback """
        rospy.logdebug(f"(Client) Heard back feedback")
        rospy.logdebug(f"(Client) State was: {feedback.state}")
        self.action_feedback["state"] = feedback.state
        if self.execute_goal_feedback_callback:
            self.execute_goal_feedback_callback(self.action_feedback)
        return

    def send_goal(
        self,
        action_goal,
        optional_data="",
        resource="",
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

        goal = harmoniGoal(
            action_type=action_goal,
            optional_data=optional_data,
            resource=resource,
            condition=condition,
        )
        rospy.loginfo("(Client) Sending Goal.")
        self._send_goal(
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
            wait (bool): Indicates whether or not to wait
        """

        self.action_client = ActionClient(action_type_name, harmoniAction)
        self.simple_state = SimpleGoalState.DONE
        self.gh = None
        self.done_condition = threading.Condition()

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

    ## @brief Blocks until the action server connects to this client
    ##
    ## @param timeout Max time to block before returning. A zero
    ## timeout is interpreted as an infinite timeout.
    ##
    ## @return True if the server connected in the allocated time. False on timeout
    def wait_for_server(self, timeout=rospy.Duration()):
        return self.action_client.wait_for_server(timeout)

    ## @brief Sends a goal to the ActionServer, and also registers callbacks
    ##
    ## If a previous goal is already active when this is called. We simply forget
    ## about that goal and start tracking the new goal. No cancel requests are made.
    ##
    ## @param done_cb Callback that gets called on transitions to
    ## Done.  The callback should take two parameters: the terminal
    ## state (as an integer from actionlib_msgs/GoalStatus) and the
    ## result.
    ##
    ## @param active_cb   No-parameter callback that gets called on transitions to Active.
    ##
    ## @param feedback_cb Callback that gets called whenever feedback
    ## for this goal is received.  Takes one parameter: the feedback.
    def _send_goal(self, goal, done_cb=None, active_cb=None, feedback_cb=None):
        # destroys the old goal handle
        self.stop_tracking_goal()

        self.done_cb = done_cb
        self.active_cb = active_cb
        self.feedback_cb = feedback_cb

        self.simple_state = SimpleGoalState.PENDING
        self.gh = self.action_client.send_goal(
            goal, self._handle_transition, self._handle_feedback
        )

    ## @brief Sends a goal to the ActionServer, waits for the goal to complete, and preempts goal is necessary
    ##
    ## If a previous goal is already active when this is called. We simply forget
    ## about that goal and start tracking the new goal. No cancel requests are made.
    ##
    ## If the goal does not complete within the execute_timeout, the goal gets preempted
    ##
    ## If preemption of the goal does not complete withing the preempt_timeout, this
    ## method simply returns
    ##
    ## @param execute_timeout The time to wait for the goal to complete
    ##
    ## @param preempt_timeout The time to wait for preemption to complete
    ##
    ## @return The goal's state.
    def send_goal_and_wait(
        self, goal, execute_timeout=rospy.Duration(), preempt_timeout=rospy.Duration()
    ):
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

    ## @brief Blocks until this goal transitions to done
    ## @param timeout Max time to block before returning. A zero timeout is interpreted as an infinite timeout.
    ## @return True if the goal finished. False if the goal didn't finish within the allocated timeout
    def wait_for_result(self, timeout=rospy.Duration()):
        if not self.gh:
            rospy.logerr("Called wait_for_result when no goal exists")
            return False

        timeout_time = rospy.get_rostime() + timeout
        loop_period = rospy.Duration(0.1)
        with self.done_condition:
            while not rospy.is_shutdown():
                time_left = timeout_time - rospy.get_rostime()
                if timeout > rospy.Duration(0.0) and time_left <= rospy.Duration(0.0):
                    break

                if self.simple_state == SimpleGoalState.DONE:
                    break

                if time_left > loop_period or timeout == rospy.Duration():
                    time_left = loop_period

                self.done_condition.wait(time_left.to_sec())

        return self.simple_state == SimpleGoalState.DONE

    ## @brief Gets the Result of the current goal
    def get_result(self):
        if not self.gh:
            rospy.logerr("Called get_result when no goal is running")
            return None

        return self.gh.get_result()

    ## @brief Get the state information for this goal
    ##
    ## Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED,
    ## PREEMPTED, ABORTED, SUCCEEDED, LOST.
    ##
    ## @return The goal's state. Returns LOST if this
    ## SimpleActionClient isn't tracking a goal.
    def get_state(self):
        if not self.gh:
            return GoalStatus.LOST
        status = self.gh.get_goal_status()

        if status == GoalStatus.RECALLING:
            status = GoalStatus.PENDING
        elif status == GoalStatus.PREEMPTING:
            status = GoalStatus.ACTIVE

        return status

    ## @brief Returns the current status text of the goal.
    ##
    ## The text is sent by the action server. It is designed to
    ## help debugging issues on the server side.
    ##
    ## @return The current status text of the goal.
    def get_goal_status_text(self):
        if not self.gh:
            rospy.logerr("Called get_goal_status_text when no goal is running")
            return "ERROR: Called get_goal_status_text when no goal is running"

        return self.gh.get_goal_status_text()

    ## @brief Cancels all goals currently running on the action server
    ##
    ## This preempts all goals running on the action server at the point that
    ## this message is serviced by the ActionServer.
    def cancel_all_goals(self):
        self.action_client.cancel_all_goals()

    ## @brief Cancels all goals prior to a given timestamp
    ##
    ## This preempts all goals running on the action server for which the
    ## time stamp is earlier than the specified time stamp
    ## this message is serviced by the ActionServer.
    def cancel_goals_at_and_before_time(self, time):
        self.action_client.cancel_goals_at_and_before_time(time)

    ## @brief Cancels the goal that we are currently pursuing
    def cancel_goal(self):
        if self.gh:
            self.gh.cancel()

    ## @brief Stops tracking the state of the current goal. Unregisters this goal's callbacks
    ##
    ## This is useful if we want to make sure we stop calling our callbacks before sending a new goal.
    ## Note that this does not cancel the goal, it simply stops looking for status info about this goal.
    def stop_tracking_goal(self):
        self.gh = None

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
                self._set_simple_state(SimpleGoalState.ACTIVE)
                if self.active_cb:
                    self.active_cb()
            elif self.simple_state == SimpleGoalState.DONE:
                rospy.logerr(error_msg)
        elif comm_state == CommState.RECALLING:
            if self.simple_state != SimpleGoalState.PENDING:
                rospy.logerr(error_msg)
        elif comm_state == CommState.PREEMPTING:
            if self.simple_state == SimpleGoalState.PENDING:
                self._set_simple_state(SimpleGoalState.ACTIVE)
                if self.active_cb:
                    self.active_cb()
            elif self.simple_state == SimpleGoalState.DONE:
                rospy.logerr(error_msg)
        elif comm_state == CommState.DONE:
            if self.simple_state in [SimpleGoalState.PENDING, SimpleGoalState.ACTIVE]:
                self._set_simple_state(SimpleGoalState.DONE)
                if self.done_cb:
                    self.done_cb(gh.get_goal_status(), gh.get_result())
                with self.done_condition:
                    self.done_condition.notifyAll()
            elif self.simple_state == SimpleGoalState.DONE:
                rospy.logerr("SimpleActionClient received DONE twice")

    def _handle_feedback(self, gh, feedback):
        if not self.gh:
            # this is not actually an error - there can be a small window in which old feedback
            # can be received between the time this variable is reset and a new goal is
            # sent and confirmed
            return
        if gh != self.gh:
            rospy.logerr(
                "Got a feedback callback on a goal handle that we're not tracking. %s vs %s"
                % (
                    self.gh.comm_state_machine.action_goal.goal_id.id,
                    gh.comm_state_machine.action_goal.goal_id.id,
                )
            )
            return
        if self.feedback_cb:
            self.feedback_cb(feedback)

    def _set_simple_state(self, state):
        self.simple_state = state

