#!/usr/bin/env python3

# Common Imports
import rospy, rospkg, roslib

from harmoni_common_lib.constants import State, ActuatorNameSpace
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
from harmoni_common_lib.action_client import HarmoniActionClient
from actionlib_msgs.msg import GoalStatus
import harmoni_common_lib.helper_functions as hf
from harmoni_speaker.speaker_service import SpeakerService

# Specific Imports
from audio_common_msgs.msg import AudioData
from harmoni_common_lib.constants import ActuatorNameSpace, ActionType
from botocore.exceptions import BotoCoreError, ClientError
from contextlib import closing
from collections import deque 
import soundfile as sf
import numpy as np
import boto3
import re
import json
import ast
import sys
import time

# import wget
import contextlib
import ast
import wave
import os

#py_tree
import py_trees

class SpeakerServicePyTree(py_trees.behaviour.Behaviour):

    #TODO tutte le print devono diventare console py_tree
    """
    mode è il boolean che controlla la modalità di funzionamento:
    true: opzione 1 (utilizzo come una classe python)
    false: opzione 2 (utilizzo mediate action_goal)
    """
    #TTS è un actuators

    def __init__(self, name = "SpeakerServicePyTree"):
        
        """
        Qui abbiamo pensato di chiamare soltanto 
        il costruttore del behaviour tree 
        """
        self.name = name
        self.mode = False
        self.speaker_service = None
        self.result_data = None
        self.service_client_speaker = None
        self.client_result = None
        self.audio_data = None

        self.blackboards = []
        #serve una blackboard a speaker?
        self.blackboard_tts = self.attach_blackboard_client(name=self.name, namespace="harmoni_tts")
        self.blackboard_tts.register_key("result_data", access=py_trees.common.Access.READ)
        self.blackboard_tts.register_key("result_message", access=py_trees.common.Access.READ)

        super(SpeakerServicePyTree, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self,**additional_parameters):
        """

        """
        for parameter in additional_parameters:
            print(parameter, additional_parameters[parameter])  
            if(parameter =="SpeakerServicePyTree_mode"):
                print("Setto la modalità")
                self.mode = additional_parameters[parameter]  

        service_name = ActuatorNameSpace.speaker.name
        instance_id = rospy.get_param("/instance_id")
        service_id = f"{service_name}_{instance_id}"

        self.speaker_service = SpeakerService(service_id)
        #rospy.init_node("speaker_default", log_level=rospy.INFO)
    
        if(not self.mode):
            self.service_client_speaker = HarmoniActionClient(self.name)
            self.client_result = deque()
            self.service_client_speaker.setup_client("speaker_default", 
                                                self._result_callback,
                                                self._feedback_callback)
            self.logger.debug("Behavior interface action clients have been set up!")
        
        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self):
        """
        
        """   
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))
    
    def update(self):
        """
        
        """
        #TODO vediti meglio INVALID
        if(self.mode):
            if self.blackboard_tts.result_message == "SUCCESS":
                self.audio_data = self.blackboard_tts.result_data
                self.result_data = self.speaker_service.do(self.audio_data)
                #vedi che succede
                #new_status = py_trees.common.Status.SUCCESS
            else:
                #lo stato o è "RUNNING" o è "FAILURE" e quindi in ogni caso sarà:
                new_status = self.blackboard_tts.result_message
        else:
            if self.blackboard_tts.result_message == "SUCCESS":
                #ho già fatto la richiesta? se si non la faccio se no la faccio
                if self.service_client_speaker.get_state() == GoalStatus.LOST:
                    self.audio_data = self.blackboard_tts.result_data
                    self.logger.debug(f"Sending goal to {self.speaker_service}")
                    self.service_client_speaker.send_goal(
                        action_goal = ActionType["DO"].value,
                        optional_data = self.audio_data,
                        wait=False,
                    )
                    self.logger.debug(f"Goal sent to {self.speaker_service}")
                    new_status = py_trees.common.Status.RUNNING
                else:
                    if len(self.client_result) > 0:
                        #se siamo qui vuol dire che il risultato c'è e quindi 
                        #possiamo terminare la foglia
                        self.result_data = self.client_result.popleft()["data"]
                        #se vuoi sapere cosa c'è scritto nel risultato usa self.result_data["response"]
                        new_status = py_trees.common.Status.SUCCESS
                    else:
                        #se siamo qui vuol dire che il risultato ancora non c'è, dunque
                        #si è rotto tutto o dobbiamo solo aspettare?
                        #incerti di questa riga, vedi 408 sequential_pattern.py
                        if(self.speaker_service.state == State.FAILED):
                            self.blackboard_tts.result_message = "FAILURE"
                            new_status = py_trees.common.Status.FAILURE
                        else:
                            new_status = py_trees.common.Status.RUNNING
            else:
                #lo stato o è "RUNNING" o è "FAILURE" e quindi in ogni caso sarà:
                new_status = self.blackboard_tts.result_message
            
        self.logger.debug("%s.update()[%s]--->[%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status

        

    def terminate(self, new_status):
        """
        When is this called?
           Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """
        if(new_status == py_trees.common.Status.INVALID):
            #esegui codice per interrupt 
            #TODO 
            if(self.mode):
                pass
            else:
                pass
        else:
            #esegui codice per terminare (SUCCESS || FAILURE)
            self.client_result = deque()

        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

    def _result_callback(self, result):
        """ Recieve and store result with timestamp """
        self.logger.debug("The result of the request has been received")
        self.logger.debug(
            f"The result callback message from {result['service']} was {len(result['message'])} long"
        )
        self.client_result.append(
            {"data": result["message"]}
        )
        # TODO add handling of errors and continue=False
        return

    def _feedback_callback(self, feedback):
        """ Feedback is currently just logged """
        self.logger.debug("The feedback recieved is %s." % feedback)
        # Check if the state is end, stop the behavior pattern
        # if feedback["state"] == State.END:
        #    self.end_pattern = True
        return

def main():
    #command_line_argument_parser().parse_args()
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    
    speakerPyTree =  SpeakerServicePyTree("SpeakerPyTreeTest")

    additional_parameters = dict([
        ("mode",False)])    

    speakerPyTree.setup(**additional_parameters)
    try:
        for unused_i in range(0, 4):
            speakerPyTree.tick_once()
            time.sleep(0.5)
        print("\n")
    except KeyboardInterrupt:
        print("Exception occurred")
        pass
    

if __name__ == "__main__":
    main()
