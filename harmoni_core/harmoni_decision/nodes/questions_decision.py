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


class MultipleChoiceDecisionManager(HarmoniServiceManager):
    """Instantiates behaviors and receives commands/data for them.
    This class is a singleton ROS node and should only be instantiated once.
    """

    def __init__(self, name, script, instance_id , path, activity_script):
        super().__init__(name)
        self.name = name
        self.script = script
        self.service_id = instance_id
        self.pattern_script_path = path
        self.config_activity_script = activity_script
        self.index = 0
        
        self.text_pub = rospy.Publisher(
            "/harmoni/detecting/stt/default", String, queue_size=10
        )

        self.index = 0
        self.max_index = 18
        self.choice_index = 16
        self.sequence_scenes = []
        self.state = State.INIT


    def start(self, index_scene= 0):
        self.populate_scene(index_scene)
        self.state = State.START
        dp = SequentialPattern(self.name, self.script)
        dp.start()

    def populate_scene(self, index_scene):

        self.script[1]["steps"][0]["web_default"]["trigger"] = (
            "[{'component_id':'img_1', 'set_content':'"
            + self.config_activity_script[0]["Q&A"][0]["General"][0]["Linguaggio"]["tasks"][index_scene]["img_1"]
            + "'}, {'component_id':'img_2', 'set_content':'"
            + self.config_activity_script[0]["Q&A"][0]["General"][0]["Linguaggio"]["tasks"][index_scene]["img_2"]
            + "'}, {'component_id':'title', 'set_content':'"
            + self.config_activity_script[0]["Q&A"][0]["General"][0]["Linguaggio"]["tasks"][index_scene]["text"]
            + "'}, {'component_id':'questions_container', 'set_content':''}]"
        )
        self.script[1]["steps"][1]["tts_default"]["trigger"] = self.config_activity_script[0]["Q&A"][0]["General"][0]["Linguaggio"]["tasks"][index_scene]["text"]

        with open(self.pattern_script_path, "w") as json_file:
            json.dump(self.script, json_file)



if __name__ == "__main__":
    name = rospy.get_param("/pattern_name/")
    # test = rospy.get_param("/test_" + name + "/")
    # test_input = rospy.get_param("/test_input_" + name + "/")
    test_input = "Input"
    instance_id = rospy.get_param("/instance_id_" + name + "/")
    
    rospack = rospkg.RosPack()
    pck_path = rospack.get_path("harmoni_decision")
    words_file_path = pck_path + f"/dict/words.txt"
    config_activity_path = pck_path + f"/resources/config_activity.json"
    with open(config_activity_path, "r") as read_file:
        activity_script = json.load(read_file)


    # url = rospy.get_param("/url_" + name + "/")
    rospack = rospkg.RosPack()
    pck_path = rospack.get_path("harmoni_pattern")
    pattern_script_path = pck_path + f"/pattern_scripting/questions.json"
    with open(pattern_script_path, "r") as read_file:
        script = json.load(read_file)

    try:
        rospy.init_node(name + "_decision")
        bc = MultipleChoiceDecisionManager(name, script, instance_id, pattern_script_path, activity_script)
        rospy.loginfo(f"START from the first step of {name} decision.")

        bc.start()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass

