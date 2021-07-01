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

    def __init__(self, name, script, instance_id , path):
        super().__init__(name)
        self.name = name
        self.script = script
        self.service_id = instance_id
        self.pattern_script_path = path
        self.index = 0
        
        self.text_pub = rospy.Publisher(
            "/harmoni/detecting/stt/default", String, queue_size=10
        )

        self.index = 0
        self.max_index = 18
        self.choice_index = 16
        self.sequence_scenes = []
        self.setup_scene()
        self.state = State.INIT


    def start(self, index_scene= 0):
        self.populate_scene(index_scene)
        self.state = State.START
        dp = SequentialPattern(self.name, self.script)
        dp.start()

    def setup_scene(self):
        for i in range(1, 16):
            self.sequence_scenes.append(
                {
                    "background_choice": ["container_1", ""],
                    "background_cont": ["container_2", ""],
                    "background": [
                        "img_bkg",
                        "../assets/imgs/test_1.jpg",
                    ],
                    "text": "Domanda",
                    "choice_1": [
                        "img_1",
                        "../assets/imgs/test_1.jpg",
                    ],
                    "choice_2": [
                        "img_2",
                        "../assets/imgs/test_1.jpg",
                    ],
                    "choice_3": [
                        "img_3",
                        "../assets/imgs/test_1.jpg",
                    ],
                }
            )
        self.sequence_scenes.append(
            {
                "background_cont": ["container_2", ""],
                "background": [
                    "img_bkg",
                    "../assets/imgs/test_1.jpg",
                ],
                "text": "Background image",
            }
        )
        self.sequence_scenes.append(
            {
                "background_cont": ["container_2", ""],
                "background": [
                    "img_bkg",
                    "../assets/imgs/test_1.jpg",
                ],
                "text": "Background image",
            }
        )
        return

    def populate_scene(self, index_scene):
        self.script[1]["steps"][0]["web_default"]["trigger"] = (
            "[{'component_id':'"
            + self.sequence_scenes[index_scene]["background"][0]
            + "', 'set_content':'"
            + self.sequence_scenes[index_scene]["background"][1]
            + "'}, {'component_id':'"
            + self.sequence_scenes[index_scene]["background_cont"][0]
            + "', 'set_content':'"
            + self.sequence_scenes[index_scene]["background_cont"][1]
            + "'}]"
        )
        self.script[1]["steps"][1]["tts_default"]["trigger"] = self.sequence_scenes[
            index_scene
        ]["text"]
        # if index_scene < self.choice_index:
        #     self.script[1]["steps"][3]["web_default"]["trigger"] = (
        #         "[{'component_id':'"
        #         + self.sequence_scenes[index_scene]["choice_1"][0]
        #         + "', 'set_content':'"
        #         + self.sequence_scenes[index_scene]["choice_1"][1]
        #         + "'},{'component_id':'"
        #         + self.sequence_scenes[index_scene]["choice_2"][0]
        #         + "', 'set_content':'"
        #         + self.sequence_scenes[index_scene]["choice_2"][1]
        #         + "'},{'component_id':'"
        #         + self.sequence_scenes[index_scene]["choice_3"][0]
        #         + "', 'set_content':'"
        #         + self.sequence_scenes[index_scene]["choice_3"][1]
        #         + "'}, {'component_id':'"
        #         + self.sequence_scenes[index_scene]["background_choice"][0]
        #         + "', 'set_content':'"
        #         + self.sequence_scenes[index_scene]["background_choice"][1]
        #         + "'}]"
        #     )
        with open(self.pattern_script_path, "w") as json_file:
            json.dump(self.script, json_file)

        # optional_data = "[{'component_id':'img_test1', 'set_content':'../assets/imgs/test_1.jpg'},{'component_id':'img_test2', 'set_content':'../assets/imgs/test_2.jpg'},{'component_id':'img_test3', 'set_content':'../assets/imgs/test_3.jpg'}, {'component_id':'questions_container', 'set_content':''}]"
        # optional_data = {"tts_default": self.sequence_scenes["tasks"][self.index]["text"], "web_default":"[{'component_id':'img_test1', 'set_content':'../assets/imgs/test_1.jpg'},{'component_id':'img_test2', 'set_content':'../assets/imgs/test_2.jpg'},{'component_id':'img_test3', 'set_content':'../assets/imgs/test_3.jpg'}, {'component_id':'questions_container', 'set_content':''}]"}


        # self.do_request(self.index, service, optional_data)

    def setup_scene____(self, config_activity_file):
        """Setup the scene """
        # activity_episode = ""
        # base_dir = os.path.dirname(__file__)

        activity_type = "Q&A" #training
        activity_structure = "General" #retelling cp
        activity_name = "Linguaggio" #il compleanno
        # if "episode" in self.activity_selected:
            # activity_episode = self.activity_selected["episode"]
        
        with open(config_activity_file, "r") as json_file:
            data = json.load(json_file)
        
        for typ in data: # e.g. Q&A storytelling or feeling_activity
            rospy.loginfo(typ)
            if activity_type in typ:
                for struct in typ[activity_type]:
                    if activity_structure in struct:
                        for nam in struct[activity_structure]:
                            if activity_name in nam:
                                self.sequence_scenes=nam[activity_name]
        return True



if __name__ == "__main__":
    name = rospy.get_param("/pattern_name/")
    # test = rospy.get_param("/test_" + name + "/")
    # test_input = rospy.get_param("/test_input_" + name + "/")
    test_input = "Input"
    instance_id = rospy.get_param("/instance_id_" + name + "/")
    
    rospack = rospkg.RosPack()
    pck_path = rospack.get_path("harmoni_decision")
    words_file_path = pck_path + f"/dict/words.txt"
    config_activity_file = pck_path + f"/resources/config_activity.json"

    # url = rospy.get_param("/url_" + name + "/")
    rospack = rospkg.RosPack()
    pck_path = rospack.get_path("harmoni_pattern")
    pattern_script_path = pck_path + f"/pattern_scripting/questions.json"
    with open(pattern_script_path, "r") as read_file:
        script = json.load(read_file)

    try:
        rospy.init_node(name + "_decision")
        bc = MultipleChoiceDecisionManager(name, script, instance_id, pattern_script_path)
        rospy.loginfo(f"START from the first step of {name} decision.")

        bc.start()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass

