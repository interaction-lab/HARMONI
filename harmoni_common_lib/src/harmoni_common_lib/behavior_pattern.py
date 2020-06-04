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
        self.router_clients[router].send_goal(action_goal=action_goal, optional_data=optional_data, child_server=child_server)
        return

    def stop(self, router):
        """Stop the Behavior Pattern """
        super().stop()
        self.router_clients[router].cancel_goal()
        return

    def pause(self):
        """Pause the Behavior Pattern """
        self.pause()
        return

# The main function will be structured like chilren,
# with a service manager that recieves instructions and controls
# the pattern

class DialogingPattern(BehaviorPatternService):
    """
    Dialoging pattern class
    """
    def __init__(self):
        """Init the behavior pattern """
        self.start(action_goal = ActionType.REQUEST, child_server="harmoni_lex_def", router="dialogue", optional_data="Hey")
        

def main():
    pass


if __name__ == "__main__":
    main()
