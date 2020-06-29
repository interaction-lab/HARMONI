#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
from harmoni_common_lib.router import HarmoniRouter
from harmoni_common_lib.constants import RouterDetector, Router


class HarmoniDetectorRouter(HarmoniRouter):
    """
    The detector router aims to handle the processing of the input data
    """

    def __init__(self, router_name, last_event):
        """ Init router"""
        child_constants_names = [enum.name for enum in list(RouterDetector)]
        super(HarmoniDetectorRouter, self).__init__(
            router_name, child_constants_names, last_event
        )

    def setup_router(self):
        self.setup_actions(self.execute_result_callback, self.execute_feedback_callback)
        rospy.loginfo("Detector router actions have been set up")
        return

    def execute_result_callback(self):
        """ Do something when result has been received """
        rospy.loginfo("Execute result callback")
        return

    def execute_feedback_callback(self):
        """ Do something when feedback has been received """
        rospy.loginfo("Execute feedback callback")
        return


def main():
    try:
        router_name = Router.DETECTOR.value
        rospy.init_node(router_name)
        last_event = ""  # TODO: How to get information about last_event from behavior controller?
        hr = HarmoniDetectorRouter(router_name, last_event)
        hr.setup_router()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
