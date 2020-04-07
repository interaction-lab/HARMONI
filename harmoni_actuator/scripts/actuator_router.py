#!/usr/bin/env python

# Importing the libraries
import rospy
import roslib
from harmoni_common_lib.router import HarmoniRouter


class HarmoniActuatorRouter(HarmoniRouter):
    """
    The actuator router aims to control the actuators of the platform, interfacing with hardwares
    """

    def __init__(self, router_name, child_names, last_event):
        """ Init router"""
        super(HarmoniActuatorRouter, self).__init__(router_name, child_names, last_event)

    def setup_router(self):
        self.setup_actions(self.execute_result_callback, self.execute_feedback_callback)
        rospy.loginfo("Actuator router actions have been set up")
        return

    def execute_result_callback(self):
        """ Do something when result has been received """
        rospy.loginfo("Execute result callback")
        return

    def execute_feedback_callback(self):
        """ Do something when feedback has been received """
        rospy.loginfo("Execute feedback callback")
        return


if __name__ == "__main__":
    router_name = "actuator"
    rospy.init_node(router_name + "_node")
    last_event = ""  # TODO: How to get information about last_event from behavior controller?
    child_names = rospy.get_param("/"+router_name+"_router")
    # I am not 100% sure but I think you need to pass the same set of args to a parent init
    # Or possible use *args, *kwargs
    hsc = HarmoniActuatorRouter(router_name, child_names, last_event)
    hsc.setup_router()
    rospy.spin()
    pass
