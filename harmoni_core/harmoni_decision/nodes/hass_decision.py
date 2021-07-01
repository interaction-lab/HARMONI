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

    def __init__(self, name, pattern_list, instance_id , words_file_path, test_input):
        super().__init__(name)
        self.name = name
        self.service_id = instance_id
        self.scripted_services = pattern_list
        self.index = 0
        
        self.text_pub = rospy.Publisher(
            "/harmoni/detecting/stt/default", String, queue_size=10
        )

        self.activity_is_on = False
        self.class_clients={}
        self._setup_classes()

        self.last_word = "casa"
        self.words = set()
        self.used_words = set()
        self._setup_activities(words_file_path)
        self.state = State.INIT


    def _setup_activities(self, words_file_path):
        # try:
        self.words = set(line.strip() for line in open(words_file_path))
        rospy.loginfo(f"Found {len(self.words)} words")

        # Split all words in syllables
        # for word in self.words:
        #     rospy.loginfo(self._divide(word))


        # except:
        #     rospy.loginfo("Couldn't load txt file")
        return

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

        # CHECK HOME ASSISTANT every x seconds
        # self.check_log_request()

        self.do_request(self.index, service, 'Ciao')

        return


    def check_log_request(self):
        
        def daemon():
            while True:
                # Time between home assistant log checks
                sleep(400)
                
                if not self.activity_is_on:
                    rospy.loginfo("Starting home assistant check log thread")
                    service = "hass"
                    self.class_clients[service].reset_init()

                    optional_data = "{ \"action\":\"check_log\", \"entity_id\":\"switch.oven_power\", \"answer\":\"yes\"}"

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
                    
                    #wait some more time
                    sleep(35)

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
                if msg == "ACTIVITY-1":
                    self.activity_is_on = True
                    service = "catena_di_parole"
                    msg = "Giochiamo alla catena di parole. A turno bisogna dire una parola che comincia con la sillaba finale di quella precedente. La prima parola è: casa."
                else:
                    service = "simple_dialogue"

            self.do_request(self.index, service, optional_data = msg) 

            self.state = State.SUCCESS

        elif result['service'] == "catena_di_parole":

            # prendi da stt
            for item in result_data:
                if "s" in item.keys():
                    word = item["s"]["data"]
                    break
                else:
                    word = ""
            
            # Remove whitespace and keep the first word
            word = word.lstrip()
            sep = ' '
            word = word.split(sep, 1)[0]   
            word = word.lower()         

            rospy.loginfo("Word by user: " + word)

            if word != "stop" and word != "basta" and word != "fine":
                service = "catena_di_parole"
                
                if word == "passo":
                    rospy.loginfo("User passed")
                    new_word = self._retrieve_word_starting_with_last_syllable(word)
                    self.last_word = new_word
                    msg = "Cambiamo parola: " + new_word

                elif word !="":
                    if self._check_word_in_dictionary(word):
                        rospy.loginfo("Word is in dictionary")

                        if self._check_word_not_used_yet(word):
                            rospy.loginfo("Word is new")

                            if self._compare_word_syllable(self.last_word, word):
                                rospy.loginfo("Syllables are the same")

                                self.used_words.add(word)
                                msg = self._retrieve_word_starting_with_last_syllable(word)
                                self.used_words.add(msg)
                                self.last_word = msg

                            else:
                                rospy.loginfo("wrong word syllable")
                                msg = "La parola: " + word + ": non inizia con la sillaba: " + self._get_last_syllable(self.last_word) + ". Riprova"
                        else:
                            rospy.loginfo("word already used")
                            msg = "La parola: " + word + ": è già stata detta. Riprova."
                    else:
                        rospy.loginfo("wrong word not in dict")
                        msg = "La parola: " + word + ": non è presente nel mio dizionario. Riprova."
                else:
                    msg = self.last_word

                self.do_request(self.index, service, optional_data = msg) 

            else:
                self.activity_is_on = False
                self.do_request(self.index, "simple_dialogue", optional_data = "Fine dell'attività.") 

            self.state = State.SUCCESS

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


    # -------------------- SYLLABLES ACTIVITY

    # ORIGINAL CODE IN CBM BASIC 2.0 by Franco Musso, March 1983:
	# http://ready64.it/ccc/pagina.php?ccc=09&pag=036.jpg
	# Adapted from its JAVASCRIPT translation by Francesco Sblendorio, May 2013:
	# http://www.sblendorio.eu/Misc/Sillabe

    def _is_vowel(self, c):
	    return "AEIOUÁÉÍÓÚÀÈÌÒÙ".find(c.upper()) != -1

    def _divide(self, word):
        if word == "":
            return

        rospy.loginfo("Dividing word: " + word)
        a = word.upper()
        result = ""
        s = 0
        while (s < len(a)):
            # rospy.loginfo("s : " + str(s) + " len a: " + str(len(a)))
            if not self._is_vowel(a[s]):
                result += word[s]
                s = s + 1

            elif len(a)-1 != s and not self._is_vowel(a[s+1]):
                if s+2 >= len(a):
                    result += word[s:s+2]+"-"
                    s= s + 2
                elif self._is_vowel(a[s+2]):
                    result += word[s]+"-"
                    s = s + 1
                elif a[s+1] == a[s+2]:
                    result += word[s:s+2]+"-"
                    s= s + 2
                elif "SG".find(a[s+1]) != -1:
                    result += word[s]+"-";
                    s = s + 1
                elif "RLH".find(a[s+2]) != -1:
                    result += word[s]+"-"
                    s = s + 1
                else :
                    result += word[s:s+2]+"-"
                    s= s + 2
                
            elif len(a)-1 != s and ("IÍÌ".find(a[s+1]) != -1) :
                if s>1 and a[s-1:s+1]=="QU" and (len(a) != s+2 and self._is_vowel(a[s+2])):
                    result += word[s:s+2]
                    s= s + 2
                elif len(a) != s+2 and self._is_vowel(a[s+2]) :
                    result += word[s]+"-"
                    s = s + 1
                else :
                    result += word[s]
                    s = s + 1
                
            elif "IÍÌUÚÙ".find(a[s])!=-1 :
                result += word[s]
                s = s + 1
            else :
                result += word[s]+"-"
                s = s + 1
            
        if result[len(result)-1] == "-":
            result = result[0:len(result)-1]
        
        return result

    def _check_word_in_dictionary(self, input_word):
        return input_word in self.words

    def _check_word_not_used_yet(self, input_word):
        return not input_word in self.used_words

    def _compare_word_syllable(self, input_word, word_found):
        return self._get_first_syllable(word_found) == self._get_last_syllable(input_word)  

    def _get_last_syllable(self, input_word):
        input_word_split = self._divide(input_word)
        rospy.loginfo("Input word in syllables: "+ input_word_split)
        input_word_syllables = (input_word_split.split('-'))
        rospy.loginfo("Last syllable : "+ input_word_syllables[-1] )
        return input_word_syllables[-1].strip()        

    def _get_first_syllable(self, word_found):
        word_found_split = self._divide(word_found)
        rospy.loginfo("Word divided in syllables: "+ word_found_split)
        syllables = (word_found_split.split('-'))
        rospy.loginfo("Syllable : "+ syllables[0] )
        return syllables[0].strip()     



    def _retrieve_word_starting_with_last_syllable(self, input_word):
        """ Retrieves a word from the dictionary that starts with the same syllable as the final syllable of the input word """

        found = False
        last_syllable = self._get_last_syllable(input_word)
        list = [x for x in self.words.difference(self.used_words) if x.startswith(last_syllable)]
        list_iterator = iter(list)

        while not found:
            word_found = next(list_iterator,"")
            if word_found == "":
                rospy.loginfo("Word not found")
                break
            rospy.loginfo("Word with selected prefix: "+ word_found)
            found = self._compare_word_syllable(input_word, word_found)
            rospy.loginfo("Syllable equal to prefix?: "+ str(found))

        return word_found

    # --------------------


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
    pattern_list.append("catena_di_parole")

    rospack = rospkg.RosPack()
    pck_path = rospack.get_path("harmoni_decision")
    words_file_path = pck_path + f"/dict/words.txt"

    try:
        rospy.init_node(name + "_decision")
        bc = HomeAssistantDecisionManager(name, pattern_list, instance_id, words_file_path, test_input)
        rospy.loginfo(f"START from the first step of {name} decision.")

        bc.start(service="simple_dialogue")

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
