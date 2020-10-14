#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State, ActuatorNameSpace
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf


# Specific Imports
#from qt_gesture_controller.srv import *
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np
import ast
import os
import math
import xml.etree.ElementTree as ET
from threading import Timer
import datetime
import dateutil.relativedelta

class QTSimulatorJoint(HarmoniServiceManager):
    """
    Simulator Joints publisher service
    """

    def __init__(self, name, param):
        """ Simulator"""
        super().__init__(name)
        self.joint_message = JointState()
        self.joint_message.header = Header()
        self.gesture= []
        """ Setup Params """
        self.name = name
        self.rate = rospy.Rate(param["rate"])
        self.path = param["path"]
        self.time_interval = param["time_interval"]
        self.gesture_topic = param["robot_gesture_topic"]
        self.service_id = hf.get_child_id(self.name)
        """ Setup the gesture """
        """self.joint_pub = rospy.Publisher(
            ActuatorNameSpace.gesture.value + self.service_id+"/simulated_joints" ,
            JointState,
            queue_size=1,
        )"""
        self.joint_sub = rospy.Subscriber(
            "/joint_states",
            JointState,
            self._get_joint_state_cb,
            queue_size=1,
        )
        self.command_sub = rospy.Subscriber(self.gesture_topic, String, self.command_cb, queue_size = 1)
        self.joint_pub = rospy.Publisher(
            "simulated_joints",
            JointState,
            queue_size=1,
        )
        """Setup the gesture service as server """
        self.state = State.INIT
        return


    def command_cb(self, data):
        """Get command"""
        command = data.data
        self._parse_gesture(command)
            
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


    def _get_joint_state_cb(self, data):
        "Get initial state"
        if self.joint_message.name == []:
            self.joint_message.name = data.name
            self.joint_message.position = data.position
            self.joint_message.velocity = data.velocity
            self.joint_message.effort = data.effort
            self.talker()
            

    def talker(self):
        if self.joint_message.name != []:
            while not rospy.is_shutdown():
                self.joint_message.header.stamp = rospy.Time.now()
                self.rate.sleep()
                self.joint_pub.publish(self.joint_message)

    def _update_joint(self):
        """create joint message and send command"""
        for joint in self.gesture:
            if joint["name"] in self.joint_message.name:
                index = self.joint_message.name.index(joint["name"])
                self.joint_message.position = np.asarray(self.joint_message.position)
                self.joint_message.position[index] = joint["position"]
                self.joint_pub.publish(self.joint_message)

    def _parse_gesture(self,command):
        start_command = False
        all_files = self.get_files(self.path)
        for filename in all_files:
            if not filename.endswith('.xml'): continue
            fullname = filename
            tree = ET.parse(fullname)
            root = tree.getroot()
            for child in root:
                if child.tag == "name":
                    name = child.text
                    if name == command:
                        start_command = True
            if start_command:
                array = list(root.iter("point"))
                for idx in range(0,len(array)):
                    start_command = False
                    time_start = int(array[idx].get('time'))
                    dt1 = datetime.datetime.fromtimestamp(time_start/1000000000, tz=datetime.timezone.utc)
                    if idx != len(array)-1:
                        time_plus = int(array[idx+1].get('time'))
                        dt2 = datetime.datetime.fromtimestamp(time_plus/1000000000, tz=datetime.timezone.utc)
                        delta_time = dateutil.relativedelta.relativedelta (dt2, dt1)
                        delta_sec = delta_time.microseconds/1000000
                        for el in array[idx]:
                            if "Head" in el.tag:
                                self.gesture.append({"name":el.tag, "position": math.radians(int(float(el.text)))})
                            else:
                                self.gesture.append({"name":el.tag, "position": int(float(el.text))/100})
                        t = Timer(delta_sec, self._update_joint)
                        t.start()
                        start_time = rospy.Time.now()
                        rospy.sleep(delta_sec)
                    else:
                        for el in array[idx]:
                            if "Head" in el.tag:
                                self.gesture.append({"name":el.tag, "position": math.radians(int(float(el.text)))})
                            else:
                                self.gesture.append({"name":el.tag, "position": int(float(el.text))/100})
                        t = Timer(self.time_interval, self._update_joint)
                        t.start()
                        start_time = rospy.Time.now()
                    self.gesture= []
                    


def main():
    service_name = ActuatorNameSpace.gesture.name
    name = rospy.get_param("/name_" + service_name + "/")
    test = rospy.get_param("/test_" + service_name + "/")
    test_input = rospy.get_param("/test_input_" + service_name + "/")
    test_id = rospy.get_param("/test_id_" + service_name + "/")
    try:
        rospy.init_node(service_name+"_"+name+"_simulator")
        param = rospy.get_param(name + "/" + test_id + "_param/")
        service = hf.set_service_server(service_name, test_id)
        s = QTSimulatorJoint(service+"_"+name+"_simulator", param)
        #service_server = HarmoniServiceServer(name=service, service_manager=s)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
