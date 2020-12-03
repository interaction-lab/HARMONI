#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
from harmoni_common_lib.websocket_client import HarmoniWebsocketClient
import harmoni_common_lib.helper_functions as hf
from harmoni_pattern.sequential_pattern import SequentialPattern

# Specific Imports
import rospkg
import json
import os
import inspect
import ast
from std_msgs.msg import String
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib.constants import DetectorNameSpace, ActionType, ActuatorNameSpace
from collections import deque
from time import time
import threading
import logging


class LinguisticDecisionManager(HarmoniServiceManager, HarmoniWebsocketClient):
    """Instantiates behaviors and receives commands/data for them.

    This class is a singleton ROS node and should only be instantiated once.
    """

    def __init__(self, name, pattern_list, test_id, url, test_input):
        HarmoniServiceManager.__init__(self,name)
        self.name = name
        self.url = url
        self.service_id  = test_id
        if isinstance(test_input, str):
            self.activity_selected = ast.literal_eval(test_input)
        self.index = 0
        self.sequence_scenes = {}
        self.type_web = ""
        self.patient_id =""
        self.session_id = ""
        self.scripted_services = pattern_list
        self.setup_scene()
        self.class_clients={}
        self._setup_classes()
        self.command = None
        self.state = State.INIT

    

    def _setup_classes(self):
        """
        Set up the pattern classes
        """
        rospack = rospkg.RosPack()
        pck_path = rospack.get_path("harmoni_pattern")
        for pattern in self.scripted_services:
            pattern_script_path = pck_path + f"/pattern_scripting/{pattern}.json"
            with open(pattern_script_path, "r") as read_file:
                script = json.load(read_file)
            rospy.loginfo(pattern)
            self.class_clients[pattern] = SequentialPattern(pattern,script)
            self.client_results[pattern] = deque()
        rospy.loginfo("Classes instantiate")
        rospy.loginfo(
            f"{self.name} Decision manager needs these pattern classess: {self.scripted_services}"
        )
        rospy.loginfo("Decision interface classess clients have been set up!")
        return

    def connect_socket(self, patient_id):
        self.message={"action":"OPEN", "patientId": patient_id}
        self.patient_id = int(patient_id)
        rospy.loginfo("Connection to the socket")
        #ip = "192.168.1.83"
        ip = "192.168.1.104"
        port = 3210
        secure =False
        HarmoniWebsocketClient.__init__(self,ip, port, secure, self.message)

    def open(self, message):
        rospy.loginfo(message)
        if isinstance(message, str):
            message = ast.literal_eval(message)
        if message["patientId"] == self.patient_id:
            rospy.loginfo("Pairing works")
            # set activity to idle
            self.do_request(0,"idle")
        else:
            rospy.loginfo("Wrong code")
            # rewrite the code
            self.do_request(0,"code")
        return

    def next(self,message):
        rospy.loginfo(message)
        #send finished
        self.stop("multiple_choice")
        self.command = "NEXT"
        response = {"action":"FINISHED", "patientId":self.patient_id, "sessionId":self.session_id,"minitask":self.index, "correct":False,"itemSelected": "null"}
        return

    def previous(self,message):
        rospy.loginfo(message)
        #send finished
        self.index-=2
        rospy.loginfo(self.index)
        self.command = "PREVIOUS"
        response = {"action":"FINISHED", "patientId":self.patient_id, "sessionId":self.session_id,"minitask":self.index, "correct":False,"itemSelected": "null"}
        self.stop("multiple_choice")
        return

    def terminate(self,message):
        self.command = "TERMINATE"
        rospy.loginfo("terminate")
        self.stop("multiple_choice")
        return

    def pause(self,message):
        self.command = "PAUSE"
        rospy.loginfo("pause")
        self.stop("multiple_choice")
        return

    def resume(self, message):
        self.command = "RESUME"
        rospy.loginfo("resume game")
        rospy.loginfo(message)
        if isinstance(message, str):
            message = ast.literal_eval(message)
        self.activity_selected = message["config"]
        self.session_id = message["sessionId"]
        rospy.loginfo("The activity selected is " + str(self.activity_selected))
        self.setup_scene()
        #self.send({"action":"STARTED","patientId":self.patient_id, "sessionId":self.session_id})
        rospy.sleep(2) ##handle when it finishes
        self.do_request(self.activity_selected["miniTaskId"],"multiple_choice")
        return

    def repeat(self, message):
        self.command = "REPEAT"
        rospy.loginfo("repeat game")
        rospy.loginfo(message)
        self.index = 0
        self.do_request(0,"intro")
        return

    def play_game(self, message):
        rospy.loginfo("play game")
        rospy.loginfo(message)
        if isinstance(message, str):
            message = ast.literal_eval(message)
        self.activity_selected = message["config"]
        self.session_id = message["sessionId"]
        rospy.loginfo("The activity selected is " + str(self.activity_selected))
        self.setup_scene()
        self.send({"action":"STARTED","patientId":self.patient_id, "sessionId":self.session_id})
        rospy.sleep(2) ##handle when it finishes
        self.do_request(0,"intro")
        return

    def start(self, service="code"):
        self.index = 0
        self.state = State.START
        self.do_request(self.index,service)
        return


    def stop(self, service):
        """Stop the Behavior Pattern """
        try:
            rospy.loginfo("Stop the goal")
            self.class_clients[service].stop("web_page_default")
            self.state = State.SUCCESS
        except Exception as E:
            self.state = State.FAILED
        return

    def do_request(self, index, service):
        rospy.loginfo("_____START STEP "+str(index)+" DECISION MANAGER FOR SERVICE "+service+"_______")
        self.state = State.REQUEST
        optional_data=None
        self.command=None
        if self.type_web=="alt":
            service = "display_image"
        if service=="multiple_choice":
            if self.type_web=="full":
                optional_data = {"tts_default": self.sequence_scenes["tasks"][index]["text"], "web_page_default":"[{'component_id':'main_img_full', 'set_content':'"+self.url + self.sequence_scenes["tasks"][index]["main_img"]+".png'},{'component_id':'target_img_"+self.type_web+"', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["target_img"]+".png'},{'component_id':'comp_img_"+self.type_web+"', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["comp_img"]+".png'},{'component_id':'distr_img_"+self.type_web+"', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["distr_img"]+".png'}, {'component_id':'multiple_choice_"+self.type_web+"_container', 'set_content':''}]"}
            elif self.type_web=="choices":
                optional_data = {"tts_default": self.sequence_scenes["tasks"][index]["text"], "web_page_default":"[{'component_id':'target_img_"+self.type_web+"', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["target_img"]+".png'},{'component_id':'comp_img_"+self.type_web+"', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["comp_img"]+".png'},{'component_id':'distr_img_"+self.type_web+"', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["distr_img"]+".png'}, {'component_id':'multiple_choice_"+self.type_web+"_container', 'set_content':''}]"}
            elif self.type_web=="composed":
                optional_data = {"tts_default": self.sequence_scenes["tasks"][index]["text"], "web_page_default":"[{'component_id':'first_img', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["first_img"]+".png'},{'component_id':'second_img', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["second_img"]+".png'},{'component_id':'third_img', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["third_img"]+".png'}, {'component_id':'multiple_choice_"+self.type_web+"_container', 'set_content':''}]"}
            elif self.type_web=="alt":
                if self.sequence_scenes["tasks"][index]["text"]=="":
                    service = "display_image"
                else:
                    optional_data = {"tts_default": self.sequence_scenes["tasks"][index]["text"], "web_page_default":"[{'component_id':'target_img_"+self.type_web+"', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["target_img"]+".png'},{'component_id':'comp_img_"+self.type_web+"', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["comp_img"]+".png'}, {'component_id':'multiple_choice_"+self.type_web+"_container', 'set_content':''}]"}
            else:
                rospy.loginfo("Not existing activity")
                return
        elif service=="display_image":
            if self.type_web=="alt":
                optional_data = {"tts_default": self.sequence_scenes["tasks"][index]["text"], "web_page_default":"[{'component_id':'main_img_alt', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["main_img"]+".png'},{'component_id':'display_image_container', 'set_content':''}]"}
            else:
                optional_data = {"tts_default": self.sequence_scenes["tasks"][index]["text"], "web_page_default":"[{'component_id':'main_img_alt', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["main_img"]+".png'},{'component_id':'display_image_container', 'set_content':''}]"}
        elif service=="intro":
            optional_data = {"tts_default": self.sequence_scenes["intro"]["text"], "web_page_default":"[{'component_id':'main_img_alt', 'set_content':'"+self.url +self.sequence_scenes["intro"]["img"]+".png'},{'component_id':'display_image_container', 'set_content':''}]"}
            service = "display_image"
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


    def _result_callback(self, result):
        """ Recieve and store result with timestamp """
        rospy.loginfo("The result of the request has been received")
        rospy.loginfo(
            f"The result callback message from {result['service']} was {len(result['message'])} long"
        )
        self.client_results[result["service"]].append(
            {"time": time(), "data": result["message"]}
        )
        if isinstance(result["message"], str) and result["message"]!="":
            result_data = ast.literal_eval(result["message"])
        web_result = []
        for data in result_data:
            if "w" in data:
                web_result.append(data["w"]["data"])
        rospy.loginfo("_____END STEP "+str(self.index)+" DECISION MANAGER_______")
        rospy.loginfo(web_result)
        result_empty = True
        if self.index < len(self.sequence_scenes["tasks"]):
            if self.command =="NEXT" or self.command=="PREVIOUS":
                if result['service']=="multiple_choice":
                    service = "multiple_choice"
                    self.index+=1
                    if self.type_web=="alt":
                        service="display_image"
                    self.do_request(self.index,service)
                    self.state = State.SUCCESS
                    result_empty = False
                elif result['service'] == "display_image":
                    if self.type_web=="alt":
                        rospy.loginfo("Here")
                        service = "multiple_choice"
                        self.index+=1
                        self.do_request(self.index,service)
            elif self.command=="TERMINATE" or self.command=="PAUSE":
                service="idle"
                self.do_request(self.index,service)
            elif self.command=="RESUME":
                rospy.loginfo("RESUME")
            else:
                if result['service']=="multiple_choice":
                        res = web_result[1]
                        service = "multiple_choice"
                        for res in web_result:
                            if res=="":
                                result_empty = True
                            else:
                                if "Target" or "target" in res:
                                    self.index+=1
                                    if self.type_web=="alt":
                                        service="display_image"
                                    elif self.sequence_scenes["tasks"][self.index]["distr_img"]=="": #if choices empty only show main img.
                                        rospy.loginfo("empty choices")
                                        service="display_image"
                                    self.do_request(self.index,service)
                                    self.state = State.SUCCESS
                                    rospy.loginfo("Correct")
                                    result_empty = False
                                elif "Comp" or "Distr" or "comp" or "distr" in res:
                                    self.do_request(self.index,service)
                                    rospy.loginfo("Wrong")
                                    self.state = State.FAILED
                                    result_empty = False
                elif result['service'] == "display_image":
                    if self.type_web=="alt":
                        rospy.loginfo("Here")
                        service = "multiple_choice"
                        self.index+=1
                        self.do_request(self.index,service)
                    elif self.sequence_scenes["tasks"][self.index]["distr_img"]=="":
                        service="display_image"
                        self.do_request(self.index,service)
                    else:
                        rospy.loginfo("Here")
                        service = "multiple_choice"
                        self.do_request(self.index,service)
                elif result['service'] == "code":
                    for res in web_result:
                        if res!="":
                            rospy.loginfo("The result is: "+str(res))
                            if isinstance(res, str):
                                res = ast.literal_eval(res)
                                res = res["patient_id"]
                            patient_id = res
                            self.connect_socket(patient_id)
        else:
            service = "idle"
            self.do_request(self.index,service)
            rospy.loginfo("End of activity")
        return


    def setup_scene(self):
        """Setup the scene """
        activity_episode = ""
        base_dir = os.path.dirname(__file__)
        activity_name = self.activity_selected["test"]
        activity_structure = self.activity_selected["structure"]
        activity_type = self.activity_selected["activityType"]
        if "episode" in self.activity_selected:
            activity_episode = self.activity_selected["episode"]
        with open(
            base_dir + "/resources/multiple_choice_setup.json", "r"
        ) as json_file:
            data = json.load(json_file)
        for typ in data:
            if activity_type in typ:
                for struct in typ[activity_type]:
                    if activity_structure in struct:
                        for nam in struct[activity_structure]:
                            if activity_name in nam:
                                self.type_web = nam[activity_name]["activity_type"]
                                if activity_episode!="":
                                    for ep in nam[activity_name]["content"]:
                                        if activity_episode in ep:
                                            self.sequence_scenes = ep[activity_episode]
                                else:
                                    self.sequence_scenes=nam[activity_name]
        return True

if __name__ == "__main__":
        name = rospy.get_param("/name/")
        test = rospy.get_param("/test_" + name + "/")
        test_input = rospy.get_param("/test_input_" + name + "/")
        test_id = rospy.get_param("/test_id_" + name + "/")
        url = rospy.get_param("/url_" + name + "/")
        pattern_dict = rospy.get_param("/pattern")
        pattern_list = []
        for p in pattern_dict:
            pattern_list.append(p)
        rospy.loginfo(pattern_list)
        try:
            rospy.init_node(name+"_decision")
            bc = LinguisticDecisionManager(name, pattern_list, test_id, url, test_input)
            rospy.loginfo(f"START from the first step of {name} decision.")
            if test:
                bc.start()
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
