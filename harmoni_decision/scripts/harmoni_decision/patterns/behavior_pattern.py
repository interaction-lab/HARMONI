#!/usr/bin/env python3

import rospy
import roslib
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib import constants

from harmoni_common_lib.constants import State, RouterDetector, HelperFunctions, RouterSensor
from harmoni_common_lib.child import InternalServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager

class BehaviorPattern(HarmoniServiceManager):
    """Abstract class defining common variables/functions for behavior patterns
    """
    def __init__(self):
        self.state = State.INIT
        super().__init__(self.state)
        return

    def start(self, rate=None):
        super().start(rate)
        return
    
    def stop(self):
        raise NotImplementedError
    
    def launch(service_name: str, pattern: BehaviorPattern):
        # Launch the pattern as a node.
        try:
            rospy.init_node(service_name)
            rospy.loginfo("Launching", service_name)
            service = pattern()
            service_server_list.append(InternalServiceServer(name=service, service_manager=service))
            rospy.spin()
        except rospy.ROSInterruptException:
            pass