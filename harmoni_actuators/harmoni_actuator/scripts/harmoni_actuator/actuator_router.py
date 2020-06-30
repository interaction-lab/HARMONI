#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
from harmoni_common_lib.router import HarmoniRouter
from harmoni_common_lib.constants import RouterActuator, Router


class HarmoniActuatorRouter(HarmoniRouter):
    """
    The actuator router aims to control the actuator nodes of the platform, interfacing with hardware
    """

    def __init__(self, router_name, last_event):
        """ Init router"""
        child_constants_names = [enum.name for enum in list(RouterActuator)]
        super().__init__(router_name, child_constants_names, last_event)

    def setup_router(self):
        self.setup_actions(self.execute_result_callback, self.execute_feedback_callback)
        rospy.loginfo("Actuator router actions have been set up")
        return

    def execute_result_callback(self, result):
        """ Do something when result has been received """
        rospy.loginfo("ActuatorRouter: A result has been received")
        if not self.result_received:
            rospy.loginfo(f"{self.router_name} Router: NO result was received earlier")
            self.send_result(result["do_action"], result["message"])
        else:
            rospy.loginfo(f"{self.router_name} Router: a result was received earlier")
        self.result_received = False
        return

    def execute_feedback_callback(self, feedback):
        """ Send the feedback backward when feedback has been received """
        rospy.logdebug("ActuatorRouter: The feedback received is %s" % feedback)
        self.send_feedback(feedback["state"])
        return


def main():
    try:
        router_name = Router.ACTUATOR.value
        rospy.init_node(router_name)
        last_event = ""  # TODO: How to get information about last_event from behavior controller?
        hr = HarmoniActuatorRouter(router_name, last_event)
        hr.setup_router()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()