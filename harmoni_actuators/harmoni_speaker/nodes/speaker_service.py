#!/usr/bin/env python3

# Common Imports
import rospy, rospkg, roslib

from harmoni_common_lib.constants import State, ActuatorNameSpace
from harmoni_common_lib.service_server import HarmoniServiceServer
import harmoni_common_lib.helper_functions as hf
from harmoni_speaker.speaker_service import SpeakerService

# Specific Imports
import numpy as np

# import wget
import contextlib
import ast
import wave
import os

def main():
    """Set names, collect params, and give service to server"""

    service_name = ActuatorNameSpace.speaker.name
    instance_id = rospy.get_param("/instance_id")
    service_id = f"{service_name}_{instance_id}"

    try:
        rospy.init_node(service_name)

        # params = rospy.get_param(service_name + "/" + instance_id + "_param/")

        s = SpeakerService(service_id)

        service_server = HarmoniServiceServer(service_id, s)

        print(service_name)
        print("****************************************************************************")
        print(service_id)

        service_server.start_sending_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
