#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
from harmoni_common_lib.router import HarmoniRouter

class HarmoniSensorRouter(HarmoniRouter):
    """
    The sensor router aims to handle the sensing of the environment, interfacing with hardwares
    """

    def __init__(self, router_name, child_names, last_event):
        """ Init router"""
        super(HarmoniSensorRouter, self).__init__(router_name, child_names, last_event)
        

    def setup_router(self):
        self.setup_actions(self.execute_result_callback, self.execute_feedback_callback)
        rospy.loginfo("Sensor router actions have been set up")
        return

    def execute_result_callback(self, result):
        """ Do something when result has been received """
        rospy.loginfo("The result received is %s" %result)
        return

    def execute_feedback_callback(self, feedback):
        """ Send the feedback to the behavior interface as well """
        rospy.logdebug("The feedback received is %s" %feedback)
        self.send_feedback(feedback["state"])
        return


def main():
    try: 
        router_name = "sensor"
        rospy.init_node(router_name + "_node")
        last_event = ""  # TODO: How to get information about last_event from behavior controller?
        child_names = rospy.get_param("/routers/"+router_name)
        # I am not 100% sure but I think you need to pass the same set of args to a parent init
        # Or possible use *args, *kwargs
        hr = HarmoniSensorRouter(router_name, child_names, last_event)
        hr.setup_router()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
