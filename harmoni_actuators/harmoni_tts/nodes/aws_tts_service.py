#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf
from harmoni_tts.aws_tts_service import AWSTtsService

# Specific Imports
from harmoni_common_lib.constants import ActuatorNameSpace
from botocore.exceptions import BotoCoreError, ClientError
from contextlib import closing
import soundfile as sf
import numpy as np
import boto3
import re
import json
import ast
import sys


def main():
    """[summary]
    Main function for starting HarmoniPolly service
    """
    service_name = ActuatorNameSpace.tts.name
    instance_id = rospy.get_param("instance_id")
    service_id = f"{service_name}_{instance_id}"
    try:
        rospy.init_node(service_name)

        param = rospy.get_param(service_name + "/" + instance_id + "_param/")

        s = AWSTtsService(service_id, param)
        
        print(service_name)
        print("****************************************************************************")
        print(service_id)

        service_server = HarmoniServiceServer(service_id, s)

        service_server.start_sending_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
