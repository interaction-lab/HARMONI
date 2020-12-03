#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
from harmoni_common_lib.constants import State, ActionType
from harmoni_common_lib.action_server import HarmoniActionServer


class HarmoniServiceServer(HarmoniActionServer, object):
    """
    A hardware control provider receives some data which it will formulate
    into an action for the hardware
    """

    def __init__(self, name, service_manager, check_premption_rate=10):
        """Initialize hardware control, logical defaults, and publishers/subscribers
        Can optionally test connctivity and availability of the hardware

        Args:
            name ([type]): [description]
            service_manager ([type]): [description]
        """

        self.name = name
        self.service_manager = service_manager
        self.check_premption_rate = check_premption_rate

        success = self.service_manager.test()
        if success:
            rospy.loginfo(f"Service Server {self.name} has been successfully set up")
        else:
            rospy.logwarn(f"HardwareControlServer {self.name} has not been started")
        super().__init__(name, self._execute_goal_received_callback)

    def update_feedback(self, rate=0.2):
        """Provides feedback at a constant rate on the status of the server

        Args:
            rate (float, optional): Rate feedback on state is sent (hz). Defaults to .2.
        """
        #TODO make this rate a class variable that gets used instead of creating it here.
        r = rospy.Rate(rate)
        rospy.loginfo("HardwareControlServer start continuously updating the feedback")
        while not rospy.is_shutdown():
            # if state has failed the node should be restarted
            if self.service_manager.state != State.FAILED:
                if self.service_manager.state != State.INIT:
                    self.send_feedback(self.service_manager.state)
                r.sleep()
            else:
                break
        return

    def _execute_goal_received_callback(self, goal):
        """Parse action goal cases to calls for service manager

        Args:
            goal (HarmoniAction):
        """

        if goal.action_type == ActionType.ON:
            rospy.loginfo(f"(Server {self.name}) Received goal. Starting")
            self.service_manager.start()

        elif goal.action_type == ActionType.PAUSE:
            rospy.loginfo(f"(Server {self.name}) Received goal. Pausing")
            self.service_manager.stop()

        elif goal.action_type == ActionType.OFF:
            rospy.loginfo(f"(Server {self.name}) Received goal. Stopping")
            self.service_manager.stop()
            self.service_manager.reset_init

        elif goal.action_type == ActionType.DO:
            rospy.loginfo(f"(Server {self.name}) Received goal. Doing")
            self.service_manager.actuation_completed = False
            preempted = False
            self.service_manager.do(goal.optional_data)
            while not self.service_manager.actuation_completed:
                if self.get_preemption_status():
                    preempted = True
                    rospy.Rate(self.check_premption_rate)
            if not hasattr(self.service_manager, "result_msg"):
                self.service_manager.result_msg = ""
            if preempted or self.service_manager.state == State.FAILED:
                self.send_result(
                    do_action=False, message=self.service_manager.result_msg
                )
                self.service_manager.reset_init()
            elif self.service_manager.state == State.SUCCESS:
                self.send_result(
                    do_action=True, message=self.service_manager.result_msg
                )
                self.service_manager.reset_init()

        elif goal.action_type == ActionType.REQUEST:
            rospy.loginfo(f"(Server {self.name}) Received goal. Requesting")
            self.service_manager.response_received = False
            preempted = False
            self.service_manager.request(goal.optional_data)
            while not self.service_manager.response_received:
                if self.get_preemption_status():
                    preempted = True
                    rospy.Rate(self.check_premption_rate)
                    break
            if not hasattr(self.service_manager, "result_msg"):
                self.service_manager.result_msg = ""
            if preempted or self.service_manager.state == State.FAILED:
                self.send_result(
                    do_action=False, message=self.service_manager.result_msg
                )
                self.service_manager.reset_init()
            elif self.service_manager.state == State.SUCCESS:
                self.send_result(
                    do_action=True, message=self.service_manager.result_msg
                )
                self.service_manager.reset_init()

        return
