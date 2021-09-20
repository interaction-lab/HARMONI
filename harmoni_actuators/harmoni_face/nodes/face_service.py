#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

# Specific Imports
from harmoni_common_lib.constants import ActuatorNameSpace
from harmoni_face.msg import FaceRequest
from harmoni_face.face_service import EyesService, MouthService, NoseService
from harmoni_face.face_client import Face
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from threading import Timer
from collections import deque
import json
import ast
import os


def main():
    """Set names, collect params, and give service to server"""
    service_name = ActuatorNameSpace.face.name
    instance_id = rospy.get_param("/instance_id")
    service_id_mouth = f"{service_name}_mouth_{instance_id}"
    service_id_eyes = f"{service_name}_eyes_{instance_id}"
    service_id_nose = f"{service_name}_nose_{instance_id}"
    try:
        rospy.init_node(service_name)
        print(ActuatorNameSpace.face.value)
        print("*************************************************************************")
        print(instance_id)
        print(service_name + "_eyes_" + instance_id)
        print(service_name + "_mouth_" + instance_id)
        print(service_name + "_nose_" + instance_id)
        param = rospy.get_param(service_name + "/" + instance_id + "_param")
        param_eyes = rospy.get_param(service_name + "/" + instance_id + "_param/eyes/")
        param_mouth = rospy.get_param(service_name + "/" + instance_id + "_param/mouth/")
        param_nose = rospy.get_param(service_name + "/" + instance_id + "_param/nose/")
        face = Face(ActuatorNameSpace.face.value, instance_id, param)
        s_eyes = EyesService(service_name + "_eyes_" + instance_id, param_eyes, face)
        s_mouth = MouthService(service_name + "_mouth_" + instance_id, param_mouth, face)
        s_nose = NoseService(service_name + "_nose_" + instance_id, param_nose, face)
        service_server_eyes = HarmoniServiceServer(service_id_eyes, s_eyes)
        service_server_mouth = HarmoniServiceServer(service_id_mouth, s_mouth)
        service_server_nose = HarmoniServiceServer(service_id_nose, s_nose)
        service_server_eyes.start_sending_feedback()
        service_server_mouth.start_sending_feedback()
        service_server_nose.start_sending_feedback()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
