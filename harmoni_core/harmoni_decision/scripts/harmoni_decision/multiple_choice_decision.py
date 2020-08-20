#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

# Specific Imports
import rospkg
import json
import inspect
from std_msgs.msg import String
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib.constants import DetectorNameSpace, ActionType, ActuatorNameSpace
from harmoni_pattern.sequential_pattern import SequentialPattern
from collections import deque
from time import time
import threading

class MultipleChoiceDecisionManager(HarmoniServiceManager):
    """Instantiates behaviors and receives commands/data for them.

    This class is a singleton ROS node and should only be instantiated once.
    """

    def __init__(self, name, script, test_id, path):
        super().__init__(name)
        self.name = name
        self.script = script
        self.service_id  = test_id
        self.pattern_script_path = path
        self.index = 0
        self.max_index = 18
        self.choice_index = 16
        self.sequence_scenes = []
        self.web_sub = rospy.Subscriber(
            ActuatorNameSpace.web.value + self.service_id + "/listen_click_event",
            String,
            self._event_click_callback,
            queue_size=1,
        )
        self.setup_scene()

    def start(self, index_scene):
        self.populate_scene(index_scene)
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
                        "http://i3lab.elet.polimi.it/letssayresources/img/1_1_"
                        + str(i)
                        + "_Sfondo.png",
                    ],
                    "text": "Background image",
                    "choice_1": [
                        "img_1",
                        "http://i3lab.elet.polimi.it/letssayresources/img/1_1_"
                        + str(i)
                        + "_Comp.png",
                    ],
                    "choice_2": [
                        "img_2",
                        "http://i3lab.elet.polimi.it/letssayresources/img/1_1_"
                        + str(i)
                        + "_Distr.png",
                    ],
                    "choice_3": [
                        "img_3",
                        "http://i3lab.elet.polimi.it/letssayresources/img/1_1_"
                        + str(i)
                        + "_Target.png",
                    ],
                }
            )
        self.sequence_scenes.append(
            {
                "background_cont": ["container_2", ""],
                "background": [
                    "img_bkg",
                    "http://i3lab.elet.polimi.it/letssayresources/img/1_1_17_Sfondo.png",
                ],
                "text": "Background image",
            }
        )
        self.sequence_scenes.append(
            {
                "background_cont": ["container_2", ""],
                "background": [
                    "img_bkg",
                    "http://i3lab.elet.polimi.it/letssayresources/img/1_1_18_Sfondo.png",
                ],
                "text": "Background image",
            }
        )
        return

    def populate_scene(self, index_scene):
        self.script[0]["steps"][0]["harmoni_web_default"]["trigger"] = (
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
        self.script[0]["steps"][1]["harmoni_tts_default"][
            "trigger"
        ] = self.sequence_scenes[index_scene]["text"]
        if index_scene < self.choice_index:
            self.script[0]["steps"][3]["harmoni_web_default"]["trigger"] = (
                "[{'component_id':'"
                + self.sequence_scenes[index_scene]["choice_1"][0]
                + "', 'set_content':'"
                + self.sequence_scenes[index_scene]["choice_1"][1]
                + "'},{'component_id':'"
                + self.sequence_scenes[index_scene]["choice_2"][0]
                + "', 'set_content':'"
                + self.sequence_scenes[index_scene]["choice_2"][1]
                + "'},{'component_id':'"
                + self.sequence_scenes[index_scene]["choice_3"][0]
                + "', 'set_content':'"
                + self.sequence_scenes[index_scene]["choice_3"][1]
                + "'}, {'component_id':'"
                + self.sequence_scenes[index_scene]["background_choice"][0]
                + "', 'set_content':'"
                + self.sequence_scenes[index_scene]["background_choice"][1]
                + "'}]"
            )
        with open(self.pattern_script_path, "w") as json_file:
            json.dump(self.script, json_file)

    def _event_click_callback(self, data):
        """Received data"""
        print(data)
        if "Target" in data.data:  # IF CORRECT INCREMENT THE INDEX
            self.index += 1
        if self.index < self.max_index:
            self.start(self.index)


if __name__ == "__main__":
        pattern_name = rospy.get_param("/pattern_name/")
        test = rospy.get_param("/test_" + pattern_name + "/")
        test_input = rospy.get_param("/test_input_" + pattern_name + "/")
        test_id = rospy.get_param("/test_id_" + pattern_name + "/")
        rospack = rospkg.RosPack()
        pck_path = rospack.get_path("harmoni_pattern")
        pattern_script_path = pck_path + f"/pattern_scripting/{pattern_name}.json"
        with open(pattern_script_path, "r") as read_file:
            script = json.load(read_file)
        try:
            rospy.init_node(pattern_name)
            bc = MultipleChoiceDecisionManager(pattern_name, script, test_id, pattern_script_path)
            rospy.loginfo(f"START from the first step of {pattern_name} pattern.")
            if test:
                bc.start(0)
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
