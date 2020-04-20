#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
from collections import defaultdict
from harmoni_common_lib.action_client import HarmoniActionClient


class BehaviorController():
    """Instantiates behaviors and receives commands/data for them.

    This class is a singleton ROS node and should only be instantiated once.
    """

    def __init__(self):        
        self.router_names = rospy.get_param("/routers/")
        self.active_patterns = []
        self.router_clients = defaultdict(HarmoniActionClient)

    def _connect_to_routers(self):
        """Setup behavior clients and subscribers """
        for route in self.router_names:
            self.router_clients[route] = HarmoniActionClient()
        for route, client in self.router_clients.items():
            client.setup_client(route, self.router_result_callback, self.router_feedback_callback)
        rospy.loginfo("Behavior controller has connected to HARMONI routers.")
        return
    
    def router_result_callback(self):
        #store data
        pass

    def router_feedback_callback(self):
        #store data
        pass


if __name__ == "__main__":
    try:
        rospy.init_node("behavior_node")
        bc = BehaviorController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass