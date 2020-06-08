#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib.service_manager import HarmoniServiceManager
from harmoni_common_lib.constants import *
from collections import defaultdict

# The main pattern will act as a service
# subscriptions do not need to be dynamic
class BehaviorPatternService(HarmoniServiceManager):
    """Abstract class defining common variables/functions for behavior patterns
    """

    def __init__(self):
        """Setup the Behavior Pattern as a client of all the routers """
        self.state = State.INIT
        super().__init__(self.state)
        self.router_names = [enum.value for enum in list(Router)]
        print(self.router_names)
        self.router_clients = defaultdict(HarmoniActionClient)
        for rout in self.router_names:
            self.router_clients[rout] = HarmoniActionClient()
        for rout, client in self.router_clients.items():
            client.setup_client(rout, self.execute_result_callback, self.execute_feedback_callback)
        rospy.loginfo("Behavior interface action clients have been set up")

    def execute_result_callback(self, result):
        """ Do something when result has been received """
        rospy.loginfo("The result has been received")
        return

    def execute_feedback_callback(self, feedback):
        """ Send the feedback state to the Behavior Pattern tree to decide what to do next """
        rospy.logdebug("The feedback is %s" %feedback)
        return

    def start(self, action_goal, child_server, router, optional_data):
        """Start the Behavior Pattern sending the first goal to the child"""
        self.state = State.START
        super().start()
        try:
            self.router_clients[router].send_goal(action_goal=action_goal, optional_data=optional_data, child_server=child_server)
            self.state = State.SUCCESS
        except:
            self.state = State.FAILED
        return

    def stop(self, router):
        """Stop the Behavior Pattern """
        super().stop()
        try:
            self.router_clients[router].cancel_goal()
            self.state = State.SUCCESS
        except:
            self.state = State.FAILED
        return

    def pause(self):
        """Pause the Behavior Pattern """
        self.pause()
        return

    def update(self, state):
        self.update(state)
        return

