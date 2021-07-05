#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State, ActuatorNameSpace
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

# Specific Imports
# from qt_gesture_controller.srv import *
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
import numpy as np
import ast
import sys
import os
import math
import xml.etree.ElementTree as ET


class GestureInterface(HarmoniServiceManager):
    """
    Gesture service
    """

    def __init__(self, name, param):
        """ Gesture"""
        super().__init__(name + "_misty")
        self.gestures_name = []
        self.gestures_duration = []
        self.gesture_list = []
        self.read_gesture_done = False
        """ Setup Params """
        self.name = name
        self.path = param["path"]
        self.gesture_topic = param["robot_gesture_topic"]
        self.service_id = hf.get_child_id(self.name)
        """ Setup the gesture """
        self.gesture_service = rospy.Publisher(self.gesture_topic, String, queue_size=1)
        self.gesture_sub = rospy.Subscriber(
            ActuatorNameSpace.gesture.value + self.service_id,
            String,
            self._handle_gesture_callback,
            queue_size=1,
        )
        self.gesture_pub = rospy.Publisher(
            ActuatorNameSpace.gesture.value + self.service_id + "/done",
            Bool,
            queue_size=1,
        )
        self.gesture_list_pub = rospy.Publisher(
            ActuatorNameSpace.gesture.value + self.service_id + "/get_list",
            String,
            queue_size=1,
        )
        #self.joint_sub = rospy.Subscriber(
        #    self.joint_sub_topic, JointState, self._handle_degree
        #)
        #self.joint_pub = rospy.Publisher(self.joint_pub_topic, JointState, queue_size=1)
        """Setup the gesture service as server """
        self.read_gestures(param["path"])
        self.state = State.INIT
        return

    def _handle_degree(self, data):
        degrees = data.position
        radians = [math.radians(d) for d in degrees]
        joint_rad = JointState()
        joint_rad.position = radians
        joint_rad.name = data.name
        joint_rad.velocity = data.velocity
        joint_rad.effort = data.effort
        joint_rad.header = data.header
        self.joint_pub.publish(joint_rad)

    def _handle_gesture_callback(self, data):
        """Gesture callback """
        done = False
        data = ast.literal_eval(data.data)
        print(data["gesture"], data["timing"])
        done = self.gesture_to_act(data["gesture"], data["timing"])
        while not done:
            rospy.logdebug("Wait until gesture is done")
        self.gesture_pub.publish(True)

    def gesture_to_act(self, gesture, timing):
        resp = False
        for i in range(len(self.gestures_name)):
            if self.gestures_name[i] == gesture:
                gesture_time_duration = self.gestures_duration[i]
                for j in range(1, 5):
                    gesture_time_duration_x = float(gesture_time_duration) / (j)
                    if float(timing) < gesture_time_duration_x:
                        speed = j
                    else:
                        speed = 2  # the default speed value
                rospy.loginfo(
                    "The speed of the gesture "
                    + str(self.gestures_name[i])
                    + " is: "
                    + str(speed)
                )
                # I calibrated the speed according to the gesture duration and the timing of the word
                self.gesture_service.publish(self.gestures_name[i])
                resp = True
        return resp

    def get_files(self, dirName):
        # create a list of file and sub directories
        # names in the given directory
        listOfFile = os.listdir(dirName)
        allFiles = list()
        # Iterate over all the entries
        for entry in listOfFile:
            # Create full path
            fullPath = os.path.join(dirName, entry)
            # If entry is a directory then get the list of files in this directory
            if os.path.isdir(fullPath):
                allFiles = allFiles + self.get_files(fullPath)
            else:
                allFiles.append(fullPath)
        return allFiles

    def read_gestures(self, path):
        if not self.read_gesture_done:
            all_files = self.get_files(path)
            for filename in all_files:
                if not filename.endswith(".xml"):
                    continue
                fullname = filename
                tree = ET.parse(fullname)
                root = tree.getroot()
                for child in root:
                    if child.tag == "duration":
                        self.gestures_duration.append(child.text)
                    elif child.tag == "name":
                        self.gestures_name.append(child.text)
            for index, el in enumerate(self.gestures_name):
                self.gesture_list.append(
                    {"name": str(el), "duration": self.gestures_duration[index]}
                )
            self.read_gesture_done = True
            self.gesture_list_pub.publish(str(self.gesture_list))
            self.read_gestures(path)


def main():
    service_name = ActuatorNameSpace.gesture.name
    instance_id = rospy.get_param("/instance_id")
    service_id = f"{service_name}_{instance_id}"

    try:
        rospy.init_node(service_name + "_qt")
        params = rospy.get_param(service_name + "/" + instance_id + "_param/")
        s = GestureInterface(service_name, params)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
