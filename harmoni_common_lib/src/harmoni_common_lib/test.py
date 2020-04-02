#!/usr/bin/env python

# Importing the libraries
import rospy
import roslib
import actionlib
from action_client import HarmoniActionClient
from action_server import HarmoniActionServer

class Controller(HarmoniActionServer):
    """
    Testing the callback function
    """
    def __init__(self):
        print("Init Controller")
        return
    
    def callback_controller(self, request_data):
        print("Handle controller")
        return

    def setup(self):
        print("Setup")
        self.setup_server("controller", self.callback_controller)
        return
    
    def callback_result(self):
        print("Result")

    def callback_feedback(self):
        print("Feedback")

if __name__ == "__main__":
    rospy.init_node("test_node")
    contr = Controller()
    contr.setup()
    client = HarmoniActionClient()
    client.setup_client("controller", timeout= 5, contr.callback_result(), contr.callback_feedback())
    client.send_goal(action_goal="test", optional_data="ciao", child="", condition= "", time_out=20)
    rospy.spin()
    pass