#!/usr/bin/env python

# Importing the libraries
import rospy
import roslib
import actionlib
from harmoni_common_msgs.msg import harmoniAction, harmoniFeedback, harmoniResult
from action_client import HarmoniActionClient
from action_server import HarmoniActionServer

class Controller(HarmoniActionServer):
    
    def __init__(self):
        print("Init Controller")
        return
    
    def handle_controller(self, request_data):
        print("Handle controller")
        return

    def setup(self):
        print("Setup")
        self.setup_server("controller", self.handle_controller)
        return

if __name__ == "__main__":
    rospy.init_node("test_node")
    contr = Controller()
    contr.setup()
    client = HarmoniActionClient()
    client.setup_client("controller", 5)
    client.send_goal(action_goal="test", optional_data="ciao", child="", condition= "", time_out=20)
    rospy.spin()
    pass