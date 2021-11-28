#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
from harmoni_common_lib.constants import State, ActionType
from harmoni_common_lib.action_server import HarmoniActionServer
import threading


class HarmoniServiceServer(HarmoniActionServer, object):
    """
    The service server is responsible for exposing the functionality of the service
    to clients. As a server it has a callback for when it recieves action requests from the
    clients. In this callback it uses the handle it has on the service to control the service
    according to the requests of the client.

    ActionServer functions used include:
    publish_feedback
    set_succeeded
    get_preemption_status
    """

    def __init__(self, name, service_manager, premption_rate=10):
        """Initialize the service and test that the service manager has been set up

        Args:
            name (str): name of the service, used for logging/debugging
            service_manager (HarmoniServiceManager): provides the functionality of the service
        """

        self.name = name
        self.service_manager = service_manager
        self.premption_rate = premption_rate

        if self.service_manager.test():
            rospy.loginfo(f"Service Server {self.name} has been successfully set up")
        else:
            rospy.logwarn(f"Service Server {self.name} has not been started")
        super().__init__(
            name, self._execute_goal_received_callback, self._preempt_callback
        )

        return

    def start_sending_feedback(self, rate=0.2):
        """Provides feedback at a constant rate on the status of the server
            FIXME: this is a blocking function, so it probably needs to be threaded.
        Args:
            rate (float, optional): Rate feedback on state is sent (hz). Defaults to .2.
        """
        r = rospy.Rate(rate)
        rospy.loginfo(f"{self.name} Service Server sending feedback at {rate}/sec")
        while not rospy.is_shutdown():
            if self.service_manager.state != State.FAILED:
                if self.service_manager.state != State.INIT:
                    self.publish_feedback(self.service_manager.state)
                r.sleep()
            else:
                # if state has failed the node should be restarted
                """FIXME: in general it is unlikely we can detect an unrecoverable
                node failure in low level code (e.g. segfault), so the failure state should be more of
                an indicator of wrong input or opportunity to retry than of node restart"""
                break
        return

    def send_result(self, do_action, message):
        """Send the result and action set to succeded"""
        self._result.do_action = do_action
        self._result.message = message
        self.set_succeeded(self._result)
        rospy.loginfo(
            f"(Server) sending result {do_action} to actiontype {self.action_goal}"
        )
        return

    def get_preemption_status(self):
        preempted = False
        if self.is_preempt_requested():
            rospy.loginfo(f"(Server) {self.action_goal} Action Preemepted")
            self.set_preempted()
            preempted = True
        return preempted

    def _preempt_callback(self):
        """Used to signal a cancel/pause to the currently running service so
        that a new goal can be received.
        """
        self.service_manager.stop()



    def _execute_goal_received_callback(self, goal):
        """Turns action goals into calls to the service manager. Is passed to the
        parent class to be used directly by the action server.

        Args:
            goal (HarmoniAction):
        """
        pr = rospy.Rate(self.premption_rate)

        # Create a thread
        # t = threading.Thread(target=self.handle_step, args=(optional_data))

        if goal.action_type == ActionType.ON:
            rospy.loginfo(f"(Server {self.name}) Received goal. Starting")
            t = threading.Thread(target=self.service_manager.start)
            t.start()

            # self.service_manager.start()

        elif goal.action_type == ActionType.PAUSE:
            rospy.loginfo(f"(Server {self.name}) Received goal. Pausing")

            t = threading.Thread(target=self.service_manager.pause)
            t.start()
            # self.service_manager.pause()

        elif goal.action_type == ActionType.OFF:
            rospy.loginfo(f"(Server {self.name}) Received goal. Stopping")
            t = threading.Thread(target=self.service_manager.stop)
            t.start()
            # self.service_manager.stop()
            # self.service_manager.reset_init()
            # TODO: implement reset init

        elif goal.action_type == ActionType.DO:
            # For 'do' type actions, we want to start the action and then
            # monitor for a preemption from the client or the action completion

            rospy.loginfo(f"(Server {self.name}) Received goal. Doing")

            self.service_manager.actuation_completed = False
            preempted = False

            # self.service_manager.do(goal.optional_data)
            t = threading.Thread(
                target=self.service_manager.do, args=(goal.optional_data,)
            )
            t.start()

            while not self.service_manager.actuation_completed:
                if self.get_preemption_status():
                    preempted = True
                pr.sleep()

            # Once an action has completed or been preempted, we need to let
            # the client know
            if not hasattr(self.service_manager, "result_msg"):
                self.service_manager.result_msg = ""

            if preempted or self.service_manager.state == State.FAILED:
                self.send_result(
                    do_action=False, message=self.service_manager.result_msg
                )

            elif self.service_manager.state == State.SUCCESS:
                self.send_result(
                    do_action=True, message=self.service_manager.result_msg
                )

            # To make sure we are ready for the next action, when we have completed
            # the prior action we should reset the initialization
            self.service_manager.reset_init()

        elif goal.action_type == ActionType.REQUEST:
            # For 'request' type actions, we also want to start the action and then
            # monitor for a preemption from the client or the action completion
            # the difference here is that requests typically involve waiting for
            # something external

            rospy.loginfo(f"(Server {self.name}) Received goal. Requesting")

            self.service_manager.response_received = False
            preempted = False

            # self.service_manager.request(goal.optional_data)
            t = threading.Thread(
                target=self.service_manager.request, args=(goal.optional_data,)
            )
            t.start()

            while not self.service_manager.response_received:
                if self.get_preemption_status():
                    preempted = True
                pr.sleep()

            if not hasattr(self.service_manager, "result_msg"):
                self.service_manager.result_msg = ""

            if preempted or self.service_manager.state == State.FAILED:
                self.send_result(
                    do_action=False, message=self.service_manager.result_msg
                )

            elif self.service_manager.state == State.SUCCESS:
                self.send_result(
                    do_action=True, message=self.service_manager.result_msg
                )

            self.service_manager.reset_init()

        return
