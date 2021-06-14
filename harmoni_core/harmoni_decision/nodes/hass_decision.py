#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

import sys
# print(sys.path)
sys.path.insert(0, "/root/harmoni_catkin_ws/src/HARMONI/harmoni_core/harmoni_pattern/nodes/")
# print(sys.path)

from sequential_pattern import SequentialPattern
# from harmoni_pattern.sequential_pattern import SequentialPattern

# Specific Imports
import rospkg
import json
import inspect

from std_msgs.msg import String
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib.constants import DetectorNameSpace, ActionType, ActuatorNameSpace
from collections import deque
from time import sleep, time
import threading

# CHECK if needed
import logging
import ast
import os


class HomeAssistantDecisionManager(HarmoniServiceManager):
    """Instantiates behaviors and receives commands/data for them.
    This class is a singleton ROS node and should only be instantiated once.
    """

    def __init__(self, name, pattern_list, instance_id ,test_input):
        super().__init__(name)
        self.name = name
        self.service_id = instance_id
        self.scripted_services = pattern_list
        self.index = 0
        
        self.text_pub = rospy.Publisher(
            "/harmoni/detecting/stt/default", String, queue_size=10
        )

        self.class_clients={}
        self._setup_classes()
        self.state = State.INIT


    def _setup_classes(self):
        """
        Set up the pattern classes from the patterns found in pattern parameter in configuration.yaml
        """
        rospack = rospkg.RosPack()
        pck_path = rospack.get_path("harmoni_pattern")

        for pattern in self.scripted_services:
            pattern_script_path = pck_path + f"/pattern_scripting/{pattern}.json"
            with open(pattern_script_path, "r") as read_file:
                script = json.load(read_file)
            rospy.loginfo(pattern)
            self.class_clients[pattern] = SequentialPattern(pattern, script)
            self.client_results[pattern] = deque()

        rospy.loginfo("Classes instantiate")
        rospy.loginfo(
            f"{self.name} Decision manager needs these pattern classes: {self.scripted_services}"
        )
        rospy.loginfo("Decision interface classes clients have been set up!")
        return

    def start(self, service="dialogue"):
        self.index = 0
        self.state = State.START

        self.check_log_request()

        self.do_request(self.index, service, 'Ciao')

        # rospy.loginfo("sleep")
        # sleep(8)

        # rospy.loginfo("published")
        # self.text_pub.publish("LOG: oven still on")

        return


    def check_log_request(self):
        
        # in thread
        # in loop

        def daemon():
            for i in range(1, 3):
                rospy.loginfo("Starting home assistant check thread")

                sleep(15)

                service = "hass"
                self.class_clients[service].reset_init()

                optional_data = "{ \"action\":\"check_log\", \"entity\":\"oven_power\", \"type\":\"switch\", \"answer\":\"yes\"}"

                result_msg = self.class_clients[service].request(optional_data)

                result_msg = ast.literal_eval(result_msg)

                for item in result_msg:
                    if "h" in item.keys():
                        msg = item["h"]["data"]
                        break
                else:
                    msg = ""

                rospy.loginfo("Received result from home assistant "+ msg)

                self.text_pub.publish(msg)

        d = threading.Thread(target=daemon)
        d.setDaemon(True)
        d.start()
        
        return

    def do_request(self, index, service, optional_data=None):
        rospy.loginfo("_____START STEP " + str(index) + " DECISION MANAGER FOR SERVICE " + service + "_______")
        self.state = State.REQUEST

        if optional_data != "":
            optional_data = str(optional_data)

        def daemon():
            rospy.loginfo("Starting")
            self.class_clients[service].reset_init()

            result_msg = self.class_clients[service].request(optional_data)

            result = {"service": service, "message": result_msg}
            rospy.loginfo("Received result from class")

            self._result_callback(result) 

            rospy.loginfo('Exiting')
        d = threading.Thread(target=daemon)
        d.setDaemon(True)
        d.start()
        return

    def _result_callback(self, result):
        """ Receive and store result with timestamp """
        rospy.loginfo("The result of the request has been received")
        rospy.loginfo(
            f"The result callback message from {result['service']} was {len(result['message'])} long"
        )
        self.client_results[result["service"]].append(
            {"time": time(), "data": result["message"]}
        )
        if isinstance(result["message"], str) and result["message"] != "":
            result_data = ast.literal_eval(result["message"])

        rospy.loginfo(result_data)

        if result['service'] == "simple_dialogue":
            
            rospy.loginfo("Index " + str(self.index))

            # Get message from bot
            for item in result_data:
                if "b" in item.keys():
                    msg = item["b"]["data"]
                    break
            else:
                msg = ""
            
            if "{" in msg:
                rospy.loginfo("{ in msg")
                service = "hass"
            else:
                rospy.loginfo("No { in msg")
                service = "simple_dialogue"

            self.do_request(self.index, service, optional_data = msg) 

            self.state = State.SUCCESS

            # Get message from hass
        elif result['service'] == "hass":

            # TODO depending on the code received send a different message to reverse dialogue (ok / not ok)
            # TODO manage multiple consecutive hass requests

            service = "simple_dialogue"

            for item in result_data:
                if "h" in item.keys():
                    msg = item["h"]["data"]
                    break
            else:
                msg = ""
            
            self.do_request(self.index, service, msg)
            self.state = State.SUCCESS

        else:
            rospy.loginfo("Error")
            self.state = State.FAILURE

        rospy.loginfo("_____END STEP " + str(self.index) + " DECISION MANAGER_______")
        return


if __name__ == "__main__":
    name = rospy.get_param("/pattern_name/")
    # test = rospy.get_param("/test_" + name + "/")
    # test_input = rospy.get_param("/test_input_" + name + "/")
    test_input = "Input"
    instance_id = rospy.get_param("/instance_id_" + name + "/")
    
    pattern_list = []

    # pattern_list.append("simple_dialogue")
    pattern_list.append("simple_dialogue")
    pattern_list.append("hass")


    try:
        rospy.init_node(name + "_decision")
        bc = HomeAssistantDecisionManager(name, pattern_list, instance_id, test_input)
        rospy.loginfo(f"START from the first step of {name} decision.")

        bc.start(service="simple_dialogue")

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
