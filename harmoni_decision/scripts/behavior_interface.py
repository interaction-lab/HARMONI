#!/usr/bin/env python

# Importing the libraries
import rospy
import roslib
from harmoni_common_lib.action_client import HarmoniActionClient


class HarmoniBehaviorInterface(HarmoniActionClient, object):
    """
    The behavior interface can act as a Client or as a Subscriber according to the BehaviorPattern it should run
    Test the behavior interface client side.
    """

    def __init__(self):
        """ Init router"""
        super(HarmoniBehaviorInterface, self).__init__()

    def setup_behavior(self, action_type_name):
        self.setup_client(action_type_name, self.execute_result_callback, self.execute_feedback_callback, wait=True)
        rospy.loginfo("Behavior tree actions have been set up")
        return

    def execute_result_callback(self):
        """ Do something when result has been received """
        rospy.loginfo("Execute result callback")
        return

    def execute_feedback_callback(self):
        """ Do something when feedback has been received """
        rospy.loginfo("Execute feedback callback")
        return

    def send_goal(self, action_goal, child):
        self.send_goal(action_goal=action_goal, child=child)
        return

def main():
    try: 
        interface_name = "behavior_interface"
        rospy.init_node(interface_name + "_node")
        hr = HarmoniBehaviorInterface()
        rospy.loginfo("Set up the %s" %interface_name)
        """
        For testing the vertical implementation
        """
        hr.setup_behavior(action_type_name="sensor")
        rospy.loginfo("Send the goal listening to the SensorRouter")
        hr.send_goal(action_goal="listening", child="microphone")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
