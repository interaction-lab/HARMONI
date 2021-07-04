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


class MistyDecisionManager(HarmoniServiceManager):
    """Instantiates behaviors and receives commands/data for them.

    This class is a singleton ROS node and should only be instantiated once.
    """

    def __init__(self, name, script, instance_id, path, url):
        super().__init__(name)
        self.name = name
        self.script = script
        self.url = url
        self.service_id = instance_id
        self.state = State.INIT



    def do_activity(activity=None): #linguaggio

    #for task in activity["tasks"]: #Scegli la risposta corretta. Oggetto che serve a divertire i bambini
        name = name
        script = [{"set": "sequence","steps": [ {"tts_default": {"action_goal": "REQUEST","resource_type": "service","wait_for": "","trigger":"Qual è un animale domestico?"}},[{"speaker_default": {"action_goal": "REQUEST","resource_type": "actuator","wait_for": "new","trigger": "/root/harmoni_catkin_ws/src/HARMONI/harmoni_actuators/harmoni_tts/temp_data/tts.wav"}},{"face_mouth_default": {"action_goal": "DO","resource_type": "actuator","wait_for": "new"}}],{"web_default": {"action_goal": "DO","resource_type": "actuator","wait_for": "new","trigger": "[{'component_id':'QA_container6', 'set_content':''}]"}}]}]
        self.seq_pattern = sequential_pattern(name, script)

    

    def do_request(self, index, service, data=None):
        rospy.loginfo("_____START STEP "+str(index)+" DECISION MANAGER FOR SERVICE "+service+"_______")
        self.state = State.REQUEST
        optional_data=None
        self.command=None
        #if self.type_web=="alt":
        #    service = "display_image"
        if service=="multiple_choice":
            if self.type_web=="full":
                optional_data = {"tts_default": self.sequence_scenes["tasks"][index]["text"], "web_page_default":"[{'component_id':'main_img_full', 'set_content':'"+self.url + self.sequence_scenes["tasks"][index]["main_img"]+".png'},{'component_id':'target_img_"+self.type_web+"', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["third_img"]+".png'},{'component_id':'comp_img_"+self.type_web+"', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["first_img"]+".png'},{'component_id':'distr_img_"+self.type_web+"', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["second_img"]+".png'}, {'component_id':'multiple_choice_"+self.type_web+"_container', 'set_content':''}]"}
            elif self.type_web=="choices":
                optional_data = {"tts_default": self.sequence_scenes["tasks"][index]["text"], "web_page_default":"[{'component_id':'target_img_"+self.type_web+"', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["third_img"]+".png'},{'component_id':'comp_img_"+self.type_web+"', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["first_img"]+".png'},{'component_id':'distr_img_"+self.type_web+"', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["second_img"]+".png'}, {'component_id':'multiple_choice_"+self.type_web+"_container', 'set_content':''}]"}
            elif self.type_web=="composed":
                optional_data = {"tts_default": self.sequence_scenes["tasks"][index]["text"], "web_page_default":"[{'component_id':'first_img', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["first_img"]+".png'},{'component_id':'second_img', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["second_img"]+".png'},{'component_id':'third_img', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["third_img"]+".png'}, {'component_id':'multiple_choice_"+self.type_web+"_container', 'set_content':''}]"}
            elif self.type_web=="alt":
                if self.sequence_scenes["tasks"][index]["text"]=="":
                    service = "display_image"
                else:
                    optional_data = {"tts_default": self.sequence_scenes["tasks"][index]["text"], "web_page_default":"[{'component_id':'target_img_"+self.type_web+"', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["third_img"]+".png'},{'component_id':'comp_img_"+self.type_web+"', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["first_img"]+".png'}, {'component_id':'multiple_choice_"+self.type_web+"_container', 'set_content':''}]"}
            else:
                rospy.loginfo("Not existing activity")
                return
        elif service=="display_image":
            if self.type_web=="alt":
                optional_data = {"tts_default": self.sequence_scenes["tasks"][index]["text"], "web_page_default":"[{'component_id':'main_img_alt', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["main_img"]+".png'},{'component_id':'display_image_container', 'set_content':''}]"}
            elif self.type_web == "retelling":
                optional_data = {"tts_default": self.sequence_scenes["tasks"][index]["text"], "web_page_default":"[{'component_id':'main_img_alt', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["main_img"]+".png'},{'component_id':'display_image_container', 'set_content':''}]"}
                if self.index == 0:
                    for i in range(2,9):
                        self.keyWordsStory += self.sequence_scenes["tasks"][self.index]["keyword"+str(i)] + "\n"
                    self.keyWordsStory += self.sequence_scenes["tasks"][self.index]["keyword9"]
                    rospy.loginfo("Ecco le keyword della storia")
                    rospy.loginfo(self.keyWordsStory)
            else:
                optional_data = {"tts_default": self.sequence_scenes["tasks"][index]["text"], "web_page_default":"[{'component_id':'main_img_alt', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["main_img"]+".png'},{'component_id':'display_image_container', 'set_content':''}]"}
        elif service=="intro":
            optional_data = {"tts_default": self.sequence_scenes["intro"]["text"], "web_page_default":"[{'component_id':'main_img_alt', 'set_content':'"+self.url +self.sequence_scenes["intro"]["img"]+".png'},{'component_id':'display_image_container', 'set_content':''}]"}
            self.index=0
            service = "display_image"
        elif service=="idle":
            if data:
                optional_data = {"tts_default": data}
            self.index=0
        elif service=="code":
            if data:
                optional_data = {"tts_default": data}
        elif service == "sentence_repetition":
            if self.type_web == "repetition":
                optional_data = {"tts_default": self.sequence_scenes["tasks"][index]["text"],"web_page_default":"[{'component_id':'sentence_repetition_container', 'set_content':''}]"}
                self.robot_sentence = self.sequence_scenes["tasks"][index]["text"]
                rospy.loginfo("DOVREBBE ESSERE LA FRASE GIUSTA QUESTA QUI -->")
                rospy.loginfo(self.robot_sentence)
            elif self.type_web == "retelling":
                optional_data = {"tts_default": self.sequence_scenes["tasks"][index]["text"],"web_page_default":"[{'component_id':'main_img_full_1', 'set_content':'"+self.url + self.sequence_scenes["tasks"][index]["img1"]+".png'},{'component_id':'main_img_full_2', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["img2"]+".png'},{'component_id':'main_img_full_3', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["img3"]+".png'},{'component_id':'main_img_full_4', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["img4"]+".png'}, {'component_id':'retelling_container', 'set_content':''}]"}
                rospy.loginfo("Qui siamo in sentence_repetition però con type_web retelling")
            else:
                rospy.loginfo("Qui non dovresti mai arrivaci")
        elif service == "retelling":
            if self.type_web == "retelling":
                optional_data = {"tts_default": self.sequence_scenes["tasks"][index]["text"],"web_page_default":"[{'component_id':'main_img_full_1', 'set_content':'"+self.url + self.sequence_scenes["tasks"][index]["img1"]+".png'},{'component_id':'main_img_full_2', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["img2"]+".png'},{'component_id':'main_img_full_3', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["img3"]+".png'},{'component_id':'main_img_full_4', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["img4"]+".png'}, {'component_id':'retelling_container', 'set_content':''}]"}
                #Questa è l'optional data che usa multiple choice full 
                #optional_data = {"tts_default": self.sequence_scenes["tasks"][index]["text"], "web_page_default":"[{'component_id':'main_img_full', 'set_content':'"+self.url + self.sequence_scenes["tasks"][index]["img1"]+".png'},{'component_id':'target_img_full', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["img2"]+".png'},{'component_id':'comp_img_full', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["img3"]+".png'},{'component_id':'distr_img_full', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["img4"]+".png'}, {'component_id':'multiple_choice_full_container', 'set_content':''}]"}
            else:
                rospy.loginfo("VEDI CHE IL TYPE WEB NON E UGUALE A retelling")
        if optional_data!="":
            optional_data = str(optional_data)
        def daemon():
            rospy.loginfo("Starting")
            self.class_clients[service].reset_init()
            result_msg = self.class_clients[service].request(optional_data)
            result = {"service":service, "message":result_msg}
            rospy.loginfo("Received result from class")
            self._result_callback(result)
            rospy.loginfo('Exiting')
        d = threading.Thread(target=daemon)
        d.setDaemon(True)
        d.start()
        return

if __name__ == "__main__":
    """Set names, collect params, and give service to server"""

    call_start = rospy.get_param("start")
    pattern_to_use = rospy.get_param("pattern_name")
    instance_id = rospy.get_param("instance_id")  # "default"
    service_id = f"{pattern_to_use}_{instance_id}"

    rospack = rospkg.RosPack()
    pck_path = rospack.get_path("harmoni_pattern")
    pattern_script_path = pck_path + f"/pattern_scripting/{pattern_to_use}.json"
    with open(pattern_script_path, "r") as read_file:
        script = json.load(read_file)

    try:
        rospy.init_node(pattern_to_use, log_level=rospy.INFO)
        bc = MistyDecisionManager(
            None, None, None, pattern_script_path, None
        )
        # multiple_choice/default_param/[all your params]
        params = rospy.get_param(pattern_to_use + "/" + instance_id + "_param/")
        script = [{"set": "sequence","steps": [ {"tts_default": {"action_goal": "REQUEST","resource_type": "service","wait_for": "","trigger":"Qual è un animale domestico?"}},[{"speaker_default": {"action_goal": "REQUEST","resource_type": "actuator","wait_for": "new","trigger": "/root/harmoni_catkin_ws/src/HARMONI/harmoni_actuators/harmoni_tts/temp_data/tts.wav"}},{"face_mouth_default": {"action_goal": "DO","resource_type": "actuator","wait_for": "new"}}],{"web_default": {"action_goal": "DO","resource_type": "actuator","wait_for": "new","trigger": "[{'component_id':'QA_container6', 'set_content':''}]"}}]}]
        s = SequentialPattern(pattern_to_use, script)
        #if call_start:
        print(s.start())

        service_server = HarmoniServiceServer(service_id, s)

        service_server.start_sending_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass