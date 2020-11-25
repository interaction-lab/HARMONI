#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
from harmoni_common_lib.client_interface import HarmoniClientInterface
import harmoni_common_lib.helper_functions as hf

# Specific Imports
import rospkg
import json
import os
import inspect
import ast
from std_msgs.msg import String
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib.client_interface import HarmoniClientInterface
from harmoni_common_lib.constants import DetectorNameSpace, ActionType, ActuatorNameSpace
from collections import deque
from time import time
import threading

class MultipleChoiceDecisionManager(HarmoniServiceManager, HarmoniClientInterface):
    """Instantiates behaviors and receives commands/data for them.

    This class is a singleton ROS node and should only be instantiated once.
    """

    def __init__(self, name,  test_id, url, test_input):
        super().__init__(name)
        self.name = name
        self.url = url
        self.service_id  = test_id
        self.activity_selected = ast.literal_eval(test_input)
        self.index = 0
        self.sequence_scenes = {}
        self.type_web = ""
        self.patient_id =""
        self.scripted_services = ["code", "multiple_choice","display_image"] #get the json names
        self.setup_scene()
        self._setup_clients()
        self.state = State.INIT

    def connect_socket(self, data):
        rospy.loginfo(data.data)
        data = data.data
        data = ast.literal_eval(data)
        rospy.loginfo(data)
        patient_id = data["patient_id"]
        self.message={"action":"OPEN", "patientId": patient_id}
        self.patient_id = int(patient_id)
        #start WebSocket connection
        super().open_connect(self.message)

    def open(message):
        rospy.log(message)
        

    def _setup_clients(self):
        """
        Set up the pattern client
        """
        for client in self.scripted_services:
            rospy.loginfo(client)
            self.service_clients[client] = HarmoniActionClient(client)
            self.client_results[client] = deque()
        rospy.loginfo("Clients created")
        rospy.loginfo(
            f"{self.name} Decision manager needs these services: {self.scripted_services}"
        )
        for cl, client in self.service_clients.items():
            client.setup_client(cl, self._result_callback, self._feedback_callback)
        rospy.loginfo("Decision interface action clients have been set up!")
        return


    def start(self, service="code"):
        self.state = State.START
        if self.type_web=="alt":
            service = "display_image"
        self.do_request(0,service)
        return


    def stop(self, service):
        """Stop the Behavior Pattern """
        try:
            self.service_clients[service].cancel_goal()
            self.state = State.SUCCESS
        except Exception as E:
            self.state = State.FAILED
        return

    def do_request(self, index, service):
        rospy.loginfo("_____START STEP "+str(index)+" DECISION MANAGER_______")
        self.state = State.REQUEST
        optional_data=""
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
        elif service=="code":
            optional_data = {"web_page_default":"[{'component_id':'"+service+"_container', 'set_content':''}]"}
        self.service_clients[service].send_goal(
                    action_goal=ActionType.REQUEST,
                    optional_data=str(optional_data),
                    wait=False,
                )
        rospy.loginfo(f"Goal sent to {service}")
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
        result_data = ast.literal_eval(result["message"])
        web_result = []
        for data in result_data:
            if "w" in data:
                web_result.append(data["w"]["data"])
        rospy.loginfo("_____END STEP "+str(self.index)+" DECISION MANAGER_______")
        rospy.loginfo(web_result)
        result_empty = True
        if self.index < len(self.sequence_scenes["tasks"]):
            if result['service']=="multiple_choice":
                for res in web_result:
                        service = "multiple_choice"
                        if res == "":
                            rospy.loginfo("Not doing anything")
                            result_empty = True
                        elif "Target" or "target" in res:
                            self.index+=1
                            if self.type_web=="alt":
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
            elif result['service'] == "code":
                for res in web_result:
                    self.patient_id = res
        else:
            service = "display_image"
            rospy.loginfo("End of activity")
        return

    def _feedback_callback(self, feedback):
        """ Send the feedback state to the Behavior Pattern tree to decide what to do next """
        rospy.logdebug("The feedback recieved is %s and nothing more" % feedback)
        # Check if the state is end, stop the behavior pattern
        # if feedback["state"] == State.END:
        #    self.end_pattern = True
        return

    def setup_scene(self):
        """Setup the scene """
        activity_episode = ""
        base_dir = os.path.dirname(__file__)
        activity_name = self.activity_selected["name"]
        activity_structure = self.activity_selected["structure"]
        activity_type = self.activity_selected["activity_type"]
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
        return

if __name__ == "__main__":
        name = rospy.get_param("/name/")
        test = rospy.get_param("/test_" + name + "/")
        test_input = rospy.get_param("/test_input_" + name + "/")
        test_id = rospy.get_param("/test_id_" + name + "/")
        url = rospy.get_param("/url_" + name + "/")
        try:
            rospy.init_node(name+"_decision")
            bc = MultipleChoiceDecisionManager(name, test_id, url, test_input)
            service_server = HarmoniServiceServer(name=name+"_decision", service_manager=bc)
            client_interface = HarmoniClientInterface(ip="192.168.1.83",port=3210, client_manager=bc)
            rospy.loginfo(f"START from the first step of {name} decision.")
            if test:
                bc.start()
            else:
                service_server.update_feedback()
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
