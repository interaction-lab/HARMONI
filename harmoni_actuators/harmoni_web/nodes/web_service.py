#!/usr/bin/env python3

# Common Imports
import rospy
import roslib
import threading
from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf
from harmoni_web.web_service import WebService

# Specific Imports
from harmoni_common_lib.constants import ActuatorNameSpace, DetectorNameSpace
from std_msgs.msg import String
import boto3
import json
import ast



def main():
    """Set names, collect params, and give service to server"""
    service_name = ActuatorNameSpace.web.name
    instance_id = rospy.get_param("/instance_id")
    service_id = f"{service_name}_{instance_id}"
    try:
        rospy.init_node(service_name)
        # params = rospy.get_param(service_name + "/" + instance_id + "_param/")
        s = WebService(service_id)
        service_server = HarmoniServiceServer(service_id, s)

        print(service_name)
        print("**********************************************************************************************")
        print(service_id)

        service_server.start_sending_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
