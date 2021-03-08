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

    def __init__(self, name, pattern_list, test_id, url, test_input, ip, port, secure):
        HarmoniServiceManager.__init__(self,name)
        self.name = name
        self.url_img = url + "img/"
        #self.url_snd = url + "sound/"
        self.url_snd = "/root/harmoni_catkin_ws/src/HARMONI/harmoni_actuators/harmoni_speaker/temp_data/sounds/"
        self.service_id  = test_id
        self.ip = ip
        self.port = port
        self.secure = secure
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
        self.start_time = None
        self.first_img=True
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
        HarmoniWebsocketClient.__init__(self, self.ip, self.port, self.secure, self.message)

    def open(self, message):
        rospy.loginfo(message)
        if isinstance(message, str):
            message = ast.literal_eval(message)
        if message["patientId"] == self.patient_id:
            rospy.loginfo("Pairing works")
            # set activity to idle
            self.do_request(0,"idle",data="*QT/bye* Benvenuto, <break time='800ms'/>  oggi giocheremo insieme!")
        else:
            rospy.loginfo("Wrong code")
            # rewrite the code
            self.do_request(0,"code", data="*QT/show_tablet*Hai sbagliato codice. <break time='800ms'/>  Riproviamo")
        return

    def store_data(self, correct, item, action="FINISHED"):
        now_time = time()
        if not self.start_time:
            int_time = 0
        else:
            int_time = now_time-self.start_time
        rospy.loginfo(f"The time is {int_time}")
        payload = {"action":action, "patientId":self.patient_id, "sessionId":self.session_id,"data":{"miniTask":self.index, "correct":correct,"itemSelected": item,"time":int_time}}
        return payload

    def next(self,message):
        rospy.loginfo(message)
        #send finished
        service = "multiple_choice"
        if self.type_web == "repetition":
            service = "sentence_repetition"
        self.stop(service)
        self.command = "NEXT"
        self.send(self.store_data(0, ""))
        return

    def replay(self,message):
        rospy.loginfo(message)
        #send finished
        service = "multiple_choice"
        if self.type_web == "repetition":
            service = "sentence_repetition"
        self.stop(service)
        self.command = "REPLAY"
        self.send(self.store_data(0, ""))
        return

    def restore(self,message):
        rospy.loginfo(message)
        #send finished
        self.terminate(message)
        os.system("pkill init")
        return

    def previous(self,message):
        rospy.loginfo(message)
        #send finished
        self.index-=2
        rospy.loginfo(self.index)
        self.command = "PREVIOUS"
        self.send(self.store_data(0, ""))
        service = "multiple_choice"
        if self.type_web == "repetition":
            service = "sentence_repetition"
        self.stop(service)
        return

    def terminate(self,message):
        self.command = "TERMINATE"
        rospy.loginfo("terminate")
        service = "multiple_choice"
        if self.type_web == "repetition":
            service = "sentence_repetition"
        self.stop(service)
        return

    def pause(self,message):
        self.command = "PAUSE"
        rospy.loginfo("pause")
        service = "multiple_choice"
        if self.type_web == "repetition":
            service = "sentence_repetition"
        self.stop(service)
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
        service = "multiple_choice"
        if self.type_web == "repetition":
            service = "sentence_repetition"
        self.do_request(self.activity_selected["miniTaskId"],service)
        return

    def repeat(self, message):
        self.command = "REPEAT"
        rospy.loginfo("repeat game")
        rospy.loginfo(message)
        self.index = 0
        self.do_request(0,"intro")
        return

    def challenge(self, message):
        self.command = "CHALLENGE"
        rospy.loginfo("challenge game")
        rospy.loginfo(message["config"])
        config = message["config"]
        self.do_request(config["challenge"],"sentence_repetition")
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
        self.start_time = time()
        rospy.sleep(2) ##handle when it finishes
        self.index = 0
        self.do_request(0,"intro")
        return

    def start(self, service="code"):
        self.index = 0
        self.state = State.START
        self.do_request(self.index,service, "Ciao mi chiamo QT, inserisci sul *QT/point_front* tablet il codice per iniziare a giocare insieme.")
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

    def do_request(self, index, service, data=None):
        rospy.loginfo("_____START STEP "+str(index)+" DECISION MANAGER FOR SERVICE "+service+"_______")
        self.state = State.REQUEST
        optional_data=None
        self.command=None
        tts_data = self.sequence_scenes["tasks"][index]["text"]
        audio_data = self.sequence_scenes["tasks"][index]["audio"]
        if index==-1:
            service = "idle"
            data="FINE"
            self.send(self.store_data(True,"",action="COMPLETED"))
        if service=="multiple_choice":
            if data == "comp":
                tts_data = self.sequence_scenes["tasks"][index]["text_comp"]
                audio_data = self.sequence_scenes["tasks"][index]["audio_comp"]
            elif data =="distr":
                tts_data = self.sequence_scenes["tasks"][index]["text_distr"]
                audio_data = self.sequence_scenes["tasks"][index]["audio_distr"]
                
            if self.type_web=="full":
                optional_data = {"tts_default": "<prosody rate='slow'>"+tts_data+"</prosody>", "speaker_default": self.url_snd+audio_data+"_robot.wav" , "web_page_default":"[{'component_id':'main_img_full', 'set_content':'"+self.url_img + self.sequence_scenes["tasks"][index]["main_img"]+".png'},{'component_id':'target_img_"+self.type_web+"', 'set_content':'"+self.url_img +self.sequence_scenes["tasks"][index]["third_img"]+".png'},{'component_id':'comp_img_"+self.type_web+"', 'set_content':'"+self.url_img +self.sequence_scenes["tasks"][index]["first_img"]+".png'},{'component_id':'distr_img_"+self.type_web+"', 'set_content':'"+self.url_img +self.sequence_scenes["tasks"][index]["second_img"]+".png'}, {'component_id':'multiple_choice_"+self.type_web+"_container', 'set_content':''}]"}
            elif self.type_web=="choices":
                optional_data = {"tts_default": "<prosody rate='slow'>"+tts_data+"</prosody>","speaker_default": self.url_snd+audio_data+"_robot.wav", "web_page_default":"[{'component_id':'target_img_"+self.type_web+"', 'set_content':'"+self.url_img +self.sequence_scenes["tasks"][index]["third_img"]+".png'},{'component_id':'comp_img_"+self.type_web+"', 'set_content':'"+self.url_img +self.sequence_scenes["tasks"][index]["first_img"]+".png'},{'component_id':'distr_img_"+self.type_web+"', 'set_content':'"+self.url_img +self.sequence_scenes["tasks"][index]["second_img"]+".png'}, {'component_id':'multiple_choice_"+self.type_web+"_container', 'set_content':''}]"}
            elif self.type_web=="composed":
                optional_data = {"tts_default": "<prosody rate='slow'>"+tts_data+"</prosody>","speaker_default": self.url_snd+audio_data+"_robot.wav", "web_page_default":"[{'component_id':'first_img', 'set_content':'"+self.url_img +self.sequence_scenes["tasks"][index]["first_img"]+".png'},{'component_id':'second_img', 'set_content':'"+self.url_img +self.sequence_scenes["tasks"][index]["second_img"]+".png'},{'component_id':'third_img', 'set_content':'"+self.url_img +self.sequence_scenes["tasks"][index]["third_img"]+".png'}, {'component_id':'multiple_choice_"+self.type_web+"_container', 'set_content':''}]"}
            elif self.type_web=="alt":
                if tts_data=="":
                    service = "display_image"
                else:
                    optional_data = {"tts_default": tts_data, "web_page_default":"[{'component_id':'target_img_"+self.type_web+"', 'set_content':'"+self.url_img +self.sequence_scenes["tasks"][index]["third_img"]+".png'},{'component_id':'comp_img_"+self.type_web+"', 'set_content':'"+self.url_img +self.sequence_scenes["tasks"][index]["first_img"]+".png'}, {'component_id':'multiple_choice_"+self.type_web+"_container', 'set_content':''}]"}
            else:
                rospy.loginfo("Not existing activity")
                return
        elif service=="display_image":
            if self.type_web=="alt":
                optional_data = {"tts_default": "<prosody rate='slow'>"+tts_data+"</prosody>", "speaker_default": self.url_snd+audio_data+"_robot.wav", "web_page_default":"[{'component_id':'main_img_alt', 'set_content':'"+self.url_img +self.sequence_scenes["tasks"][index]["main_img"]+".png'},{'component_id':'display_image_container', 'set_content':''}]"}
            else:
                optional_data = {"tts_default": "<prosody rate='slow'>"+tts_data+"</prosody>", "speaker_default": self.url_snd+audio_data+"_robot.wav", "web_page_default":"[{'component_id':'main_img_alt', 'set_content':'"+self.url_img +self.sequence_scenes["tasks"][index]["main_img"]+".png'},{'component_id':'display_image_container', 'set_content':''}]"}
        elif service=="intro":
            optional_data = {"tts_default": "<prosody rate='slow'>"+self.sequence_scenes["intro"]["text"]+"</prosody>","speaker_default": self.url_snd + self.sequence_scenes["intro"]["audio"]+"_robot.wav", "web_page_default":"[{'component_id':'main_img_alt', 'set_content':'"+self.url_img +self.sequence_scenes["intro"]["img"]+".png'},{'component_id':'display_image_container', 'set_content':''}]"}
            self.index=0
            service = "display_image"
        elif service=="idle":
            if data:
                if data=="FINE":
                    optional_data = {"tts_default": "<prosody rate='slow'> *QT/emotions/happy* Abbiamo finito ci vediamo al prossimo gioco</prosody>", "speaker_default": self.url_snd+"Abbiamo_finito_robot.wav"}
                else:
                    optional_data = {"tts_default": "<prosody rate='slow'>"+data+"</prosody>", "speaker_default": self.url_snd+"benvenuto.wav"}
            self.index=0
        elif service=="code":
            if data:
                optional_data = {"tts_default": "<prosody rate='slow'>"+data+"</prosody>", "speaker_default": self.url_snd+"Chiuti_intro.wav"}

        elif service=="sentence_repetition":
            if self.type_web=="repetition":
                optional_data = {"tts_default": tts_data, "web_page_default":"[{'component_id':'main_img_ret', 'set_content':'"+self.url_img +"dots.png'},{'component_id':'sentence_repetition_container', 'set_content':''}]"}
            elif self.type_web=="retelling":
                if index<8:
                    service="display_image"
                    optional_data = {"tts_default":  "<prosody rate='slow'>"+tts_data+"</prosody>","speaker_default": self.url_snd+audio_data+"_robot.wav", "web_page_default":"[{'component_id':'main_img_alt', 'set_content':'"+self.url_img +self.sequence_scenes["tasks"][index]["main_img"]+".png'},{'component_id':'display_image_container', 'set_content':''}]"}
                else:
                    rospy.loginfo(self.sequence_scenes["tasks"][index])
                    optional_data = {"tts_default": "<prosody rate='slow'>"+tts_data+"</prosody>", "speaker_default": self.url_snd+audio_data+"_robot.wav", "web_page_default":"[{'component_id':'main_img_ret', 'set_content':'"+self.url_img +self.sequence_scenes["tasks"][index]["main_img"]+".png'},{'component_id':'sentence_repetition_container', 'set_content':''}]"}
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
        
        rospy.loginfo(web_result)
        result_empty = True
        if self.index < (len(self.sequence_scenes["tasks"])-1) and self.index!=-1:
            if self.command =="NEXT" or self.command=="PREVIOUS":
                if result['service']=="multiple_choice":
                    service = "multiple_choice"
                    if self.type_web=="alt":
                        service="display_image"
                    self.index+=1
                    self.do_request(self.index,service)
                    self.state = State.SUCCESS
                    result_empty = False
                elif result['service'] == "display_image":
                    if self.type_web=="alt":
                        rospy.loginfo("Here")
                        service = "multiple_choice"
                        self.index+=1
                        self.do_request(self.index,service)
                elif result['service']=="sentence_repetition":
                    self.index+=1
                    self.do_request(self.index,result['service'])
                    self.state = State.SUCCESS
                    result_empty = False
            elif self.command=="TERMINATE" or self.command=="PAUSE":
                rospy.loginfo("------------Terminate")
                service="idle"
                self.do_request(self.index,service, data="Abbiamo terminato l'attività. *QT/bye* Ciao ciao, alla prossima!")
            elif self.command=="RESUME":
                rospy.loginfo("RESUME")
            elif self.command=="REPLAY":
                rospy.loginfo("REPLAY")
                self.do_request(self.index,result['service'])
            else:
                if result['service']=="multiple_choice":
                        #res = web_result[1]
                        service = "multiple_choice"
                        for res in web_result:
                            if res=="" or res=="_the_queue_is_empty":
                                result_empty = True
                            else:
                                rospy.loginfo(res)
                                if isinstance(res, str):
                                    res = ast.literal_eval(res)
                                itemselected = res["set_view"].replace(self.url_img, "")
                                if "arget" in res["set_view"]:
                                    self.index+=1
                                    if (self.type_web=="alt" and self.sequence_scenes["tasks"][self.index]["main_img"]!=""):
                                        service="display_image"
                                    elif self.sequence_scenes["tasks"][self.index]["first_img"]=="": #if choices empty only show main img.
                                        rospy.loginfo("empty choices")
                                        service="display_image"
                                    self.do_request(self.index,service)
                                    self.state = State.SUCCESS
                                    rospy.loginfo("Correct")
                                    self.send(self.store_data(1, itemselected))
                                    result_empty = False
                                elif "omp" in res["set_view"]:
                                    self.do_request(self.index, service, data="comp")
                                    rospy.loginfo("Wrong")
                                    self.state = State.FAILED
                                    self.send(self.store_data(0, itemselected))
                                    result_empty = False
                                elif "istr" in res["set_view"]:
                                    self.do_request(self.index, service, data="distr")
                                    rospy.loginfo("Wrong")
                                    self.state = State.FAILED
                                    self.send(self.store_data(0, itemselected))
                                    result_empty = False
                elif result['service'] == "display_image":
                    if self.type_web=="alt":
                        rospy.loginfo("Here")
                        service = "multiple_choice"
                        if self.index==0 and self.sequence_scenes["tasks"][self.index]["main_img"]!="":
                            service = "display_image"
                            self.index+=1
                            self.do_request(0,service)
                        else:
                            self.index+=1
                            self.do_request(self.index,service)
                    elif self.type_web=="repetition":
                            service = "sentence_repetition"
                            self.do_request(0,service)
                    elif self.type_web=="retelling":
                            service = "sentence_repetition"
                            if self.index==0 and self.first_img:
                                self.do_request(0,service)
                                self.first_img=False
                            else:
                                self.index+=1
                                self.do_request(self.index,service)
                    elif self.sequence_scenes["tasks"][self.index]["first_img"]=="":
                        print(self.index)
                        service="display_image"
                        if self.index==0 and self.sequence_scenes["tasks"][self.index]["main_img"]!="":
                            self.do_request(0,service)
                            self.index+=1
                        else:
                            self.index+=1
                            self.do_request(self.index,service)
                    else:
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
                elif result['service'] == "sentence_repetition":
                    rospy.loginfo("Sentence Repetition")
                    for res in web_result:
                        if res!="":
                            rospy.loginfo("The result is: "+str(res))
                            if isinstance(res, str):
                                res = ast.literal_eval(res)
                                res = res["transcript"]
                            transcript = res
                            service = result['service']
                            if self.type_web=="repetition":
                                result = self.check_sr(transcript, self.index)
                                self.send(self.store_data(result[0], result[1]))
                            elif self.type_web=="retelling":
                                if self.index > 7:
                                    rospy.loginfo("Send data to the back-end")
                                    self.send(self.store_data(True, transcript))
                                    self.index+=1
        else:
            if self.index==len(self.sequence_scenes["tasks"])-1:
                service = "idle"
                self.send(self.store_data(True,"",action="COMPLETED"))
                self.do_request(self.index,service,data="FINE")
                rospy.loginfo("End of activity")
                self.index = -1
        rospy.loginfo("_____END STEP "+str(self.index)+" DECISION MANAGER_______")
        return

    def check_sr(self, told, index):
        #Check if the sentence told is correct or not
        rospy.loginfo(f"The index is {index}")
        correct = 0
        target = self.sequence_scenes["tasks"][index]["text"]
        if target == told:
            correct = 1
            return [correct,told]
        resultRobot = []
        resultChild = []
        senteceRobot = target.split()
        senteceChild = told.split()

        if  len(senteceRobot) != len(senteceChild):
            resultRobot.append(target)
            resultChild.append(told)
        else:
            for i in range(len(senteceRobot)):
                if senteceRobot[i] != senteceChild[i]:
                    resultRobot.append(senteceRobot[i].upper())
                    resultChild.append(senteceChild[i].upper())
                else:
                    resultRobot.append(senteceRobot[i])
                    resultChild.append(senteceChild[i])
        rospy.loginfo(f"The target sentence was: {resultRobot}")
        rospy.loginfo(f"The told sentence was: {resultChild}")
        result = [correct, told]
        return result

    def check_ret(self, text):
        correct = True
        result = [correct, text]
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
        ip = rospy.get_param("/ip_" + name + "/")
        port = rospy.get_param("/port_" + name + "/")
        secure = rospy.get_param("/secure_" + name + "/")
        pattern_dict = rospy.get_param("/pattern")
        pattern_list = []
        for p in pattern_dict:
            pattern_list.append(p)
        rospy.loginfo(pattern_list)
        try:
            rospy.init_node(name+"_decision")
            bc = LinguisticDecisionManager(name, pattern_list, test_id, url, test_input, ip, port, secure)
            rospy.loginfo(f"START from the first step of {name} decision.")
            if test:
                bc.start(service="intro")
            else:
                bc.start()
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
