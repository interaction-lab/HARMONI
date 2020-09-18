#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

# Specific Imports
from harmoni_common_lib.constants import [name]NameSpace


class NameService(HarmoniServiceManager):
    """[summary]
    """

    def __init__(self, name):
        """ Setup params, clients, subscribers, publishers """
        super().__init__(name)

        self.state = State.INIT # Once init is done, set state to init
        return

    ######################################################
    # Setup Functions
    ######################################################

    ######################################################
    # Helper Functions
    ######################################################

    ######################################################
    # Overwrite Parent Functions
    ######################################################


def main():
    service_namespace = [name]NameSpace.[name].name
    rospy.init_node(service_namespace)
    # Read ROS Params
    test = rospy.get_param("/test/")
    test_input = rospy.get_param("/test_input/")
    test_id = rospy.get_param("/test_id/")
    try:
        service_id = rospy.get_param("/service_id/")
    except Exception:
        service_id = test_id

    try:
        params = rospy.get_param("~" + service_id + "_param/")
        s = NameService(service, params)
        server = HarmoniServiceServer(name=service, service_manager=s)

        if test:
            rospy.loginfo(f"Testing the {service}")
            data = s.run_test(test_input)
            rospy.loginfo(f"Testing the {service} completed")
        if not test:
            server.update_feedback()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
