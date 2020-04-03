#!/usr/bin/env python

# Importing the libraries
import rospy
import roslib
from harmoni_common_lib import HarmoniController 

def HarmoniSensorController(HarmoniController):
    """
    The sensor controller aims to handle the sensing of the environment, interfacing with hardwares
    """

    def __init__(self, last_event):
        """ Init controller"""
        controller_name = "sensor" # TODO: Do we want to get the controller name from the config file as well??
        sensor_child_names = rospy.get_param("use_sensor/sensor_controller")
        self.__init__(controller_name, sensor_child_names, last_event)

    def setup_controller(self):
        self.setup_actions(self.execute_result_callback, self.execute_feedback_callback)
        rospy.loginfo("Sensor controller actions have been set up")
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
    rospy.init_node("sensor_controller_node")
    last_event = "" # TODO: How to get information about last_event from behavior controller?
    hsc = HarmoniSensorController(last_event)
    hsc.setup_controller()
    rospy.spin()
    pass

