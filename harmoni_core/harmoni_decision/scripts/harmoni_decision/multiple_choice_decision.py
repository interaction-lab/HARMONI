#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
import rospkg
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib.constants import RouterActuator
from harmoni_pattern.sequential_pattern import SequentialPattern
from std_msgs.msg import String
import json
import inspect


class MultipleChoiceDecisionManager():
    """Instantiates behaviors and receives commands/data for them.

    This class is a singleton ROS node and should only be instantiated once.
    """

    def __init__(self):
        # TODO this should be a rosparam
        #pattern_name = "dialogue"
        pattern_name = "multiple-choice"
        self.name = pattern_name
        self.service_id = "default"
        self.index = 0
        #trigger_intent = rospy.get_param("/input_test_" + pattern_name + "/")
        rospack = rospkg.RosPack()
        pck_path = rospack.get_path("harmoni_pattern")
        self.pattern_script_path = pck_path + f"/pattern_scripting/{pattern_name}.json"
        with open(self.pattern_script_path, "r") as read_file:
            self.script = json.load(read_file)
            print(self.script)
        self.web_sub = rospy.Subscriber(
            RouterActuator.web.value + self.service_id + "/listen_click_event",
            String,
            self._event_click_callback,
            queue_size=1,
        )
        self.setup_scene()
        
    def start_scene(self, index_scene):
        self.populate_scene(index_scene)
        dp = SequentialPattern(self.name, self.script)
        dp.start()
        

    def setup_scene(self):
        self.sequence_scenes = [
            {"background":["container_2", ""], "text": "Background image", "choice_1":["", ""], "choice_2":["", ""], "choice_3":["", ""]},
            {"background":["container_1", ""], "text": "Multiple choice", "choice_1":["", ""], "choice_2":["", ""], "choice_3":["", ""]}
        ]
        return

    def populate_scene(self, index_scene):
        self.script[0]["steps"][0]["harmoni_web_default"]["trigger"] = "{'component_id':'"+self.sequence_scenes[index_scene]["background"][0]+"', 'set_content':'"+self.sequence_scenes[index_scene]["background"][1]+"'}"
        self.script[0]["steps"][1]["harmoni_tts_default"]["trigger"] = self.sequence_scenes[index_scene]["text"]
        #self.script[0]["steps"][3]["harmoni_web_default"]["trigger"] = "[{'component_id':"+self.sequence_scenes[index_scene]["choice_1"][0]+", 'set_content':"+self.sequence_scenes[index_scene]["choice_1"][1]+"},{'component_id':"+self.sequence_scenes[index_scene]["choice_2"][0]+", 'set_content':"+self.sequence_scenes[index_scene]["choice_2"][1]+"},{'component_id':"+self.sequence_scenes[index_scene]["choice_3"][0]+", 'set_content':"+self.sequence_scenes[index_scene]["choice_3"][1]+"}]"
        with open(self.pattern_script_path, "w") as json_file:
            json.dump(self.script, json_file)


    def _event_click_callback(self, data):
        "Received data"
        print(data)
        if data.data == "": #IF CORRECT INCREMENT THE INDEX
            self.index+=1
        self.start_scene(self.index)





if __name__ == "__main__":
    try:
        rospy.init_node("decision")
        bc = MultipleChoiceDecisionManager()
        bc.start_scene(0)
        rospy.loginfo("decision started.")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
