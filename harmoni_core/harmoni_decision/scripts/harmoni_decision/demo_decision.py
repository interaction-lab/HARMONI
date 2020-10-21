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
from harmoni_common_lib.constants import (
    DetectorNameSpace,
    ActionType,
    ActuatorNameSpace,
)
from harmoni_pattern.sequential_pattern import SequentialPattern
from collections import deque
from time import time
import threading


class TypeDialogueDecisionManager(HarmoniServiceManager):
    """Instantiates behaviors and receives commands/data for them.

    This class is a singleton ROS node and should only be instantiated once.
    """

    def __init__(self, name, script, test_id, path, url):
        super().__init__(name)
        self.name = name
        self.script = script
        self.url = url
        self.service_id = test_id
        self.pattern_script_path = path
        self.index = 0
        self.max_index = 18
        self.sequence_scenes = []
        self.web_sub = rospy.Subscriber(
            ActuatorNameSpace.web.value + self.service_id + "/listen_click_event",
            String,
            self._event_click_callback,
            queue_size=1,
        )
        self.setup_scene()
        self.state = State.INIT

    def start(self, text_input):
        if self.index != 0:
            self.populate_scene(text_input)
        self.state = State.START
        dp = SequentialPattern(self.name, self.script)
        dp.start()

    def setup_scene(self):
        for i in range(1, 16):
            self.sequence_scenes.append({"background": ["container_2", ""]})
        return

    def populate_scene(self, text_input):
        self.script[0]["steps"][0]["bot_default"]["trigger"] = text_input
        with open(self.pattern_script_path, "w") as json_file:
            json.dump(self.script, json_file)

    def _event_click_callback(self, data):
        """Received data"""
        text_input = data.data
        self.index += 1
        if self.index < self.max_index:
            self.start(text_input)


if __name__ == "__main__":
    pattern_name = rospy.get_param("/pattern_name/")
    test = rospy.get_param("/test_" + pattern_name + "/")
    test_input = rospy.get_param("/test_input_" + pattern_name + "/")
    test_id = rospy.get_param("/test_id_" + pattern_name + "/")
    url = rospy.get_param("/url_" + pattern_name + "/")
    rospack = rospkg.RosPack()
    pck_path = rospack.get_path("harmoni_pattern")
    pattern_script_path = pck_path + f"/pattern_scripting/{pattern_name}.json"
    with open(pattern_script_path, "r") as read_file:
        script = json.load(read_file)
    try:
        rospy.init_node(pattern_name)
        bc = TypeDialogueDecisionManager(
            pattern_name, script, test_id, pattern_script_path, url
        )
        service_server = HarmoniServiceServer(name=pattern_name, service_manager=bc)
        rospy.loginfo(f"START from the first step of {pattern_name} pattern.")
        if test:
            bc.start(0)
        else:
            service_server.update_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
