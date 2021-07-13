#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from actionlib_msgs.msg import GoalStatus
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
from harmoni_common_lib.action_client import HarmoniActionClient
from google_service import STTGoogleService
import harmoni_common_lib.helper_functions as hf

# Specific Imports
from harmoni_common_lib.constants import DetectorNameSpace, ActionType
from audio_common_msgs.msg import AudioData
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

#py_tree
import py_trees
import time

import py_trees.console

class SpeechToTextServicePytree(py_trees.behaviour.Behaviour):

    #TODO tutte le print devono diventare console py_tree
    """
    mode è il boolean che controlla la modalità di funzionamento:
    true: opzione 1 (utilizzo come una classe python)
    false: opzione 2 (utilizzo mediate action_goal)
    """
    #STT è un detector

    def __init__(self, name = "SpeechToTextServicePytree"):
        
        """
        Qui abbiamo pensato di chiamare soltanto 
        il costruttore del behaviour tree 
        """
        self.name = name
        self.mode = False
        self.google_service = None
        self.result_data = None
        self.service_client_stt = None
        self.client_result = None

        self.blackboards = []
        self.blackboard_input_bot=self.attach_blackboard_client(name=self.name,namespace="harmoni_input_bot")
        self.blackboard_input_bot.register_key("result_data",access=py_trees.common.Access.WRITE)
        self.blackboard_input_bot.register_key("result_message", access=py_trees.common.Access.WRITE)


        super(SpeechToTextServicePytree, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self,**additional_parameters):
        """
        Qui chiamiamo l'inizializzazione del servizio SpeechToTextService, 
        motivo per cui abbiamo aggiunto param al metodo che 
        pensiamo debbano essere passati dal chiamante e non possono essere
        creati all'interno del metodo stesso.  
        """
        for parameter in additional_parameters:
            print(parameter, additional_parameters[parameter])  
            if(parameter =="SpeechToTextServicePytree_mode"):
                self.mode = additional_parameters[parameter]        

        service_name = DetectorNameSpace.stt.name
        instance_id = rospy.get_param("instance_id")

        param = rospy.get_param(service_name + "/" + instance_id + "_param/")

        self.google_service = STTGoogleService(self.name,param)

        #TODO questo dobbiamo farlo nell'if 
        #rospy init node mi fa diventare un nodo ros
        rospy.init_node("stt_default", log_level=rospy.INFO)

        if(not self.mode):
            self.service_client_stt = HarmoniActionClient(self.name)
            self.client_result = deque()
            self.service_client_stt.setup_client("stt_default", 
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
        #Secondo me dovremmo usare transcribe_stream

        if(self.mode):
            #Dobbiamo prima fare la richiesta
            if(self.result_data["response"] == State.SUCCESS):
                self.blackboard_input_bot.result_message = "SUCCESS"
                self.blackboard_input_bot.result_data = self.result_data['message']
                self.result_data = self.result_data['message']
                new_status = py_trees.common.Status.SUCCESS
            else:
                self.blackboard_input_bot.result_message = "FAILURE"
                new_status = py_trees.common.Status.FAILURE
        else:
            if self.service_client_stt.get_state() == GoalStatus.LOST:
                    self.logger.debug(f"Sending goal to {self.google_service}")
                    # Dove posso prendere details["action_goal"]?
                    self.service_client_stt.send_goal(
                        action_goal = ActionType["REQUEST"].value,
                        optional_data = None,
                        wait=False,
                    )
                    self.logger.debug(f"Goal sent to {self.google_service}")
                    new_status = py_trees.common.Status.RUNNING
            else:
                if len(self.client_result) > 0:
                    #se siamo qui vuol dire che il risultato c'è e quindi 
                    #possiamo terminare la foglia
                    self.result_data = self.client_result.popleft()["data"]
                    self.blackboard_input_bot.result_message = "SUCCESS"
                    self.blackboard_input_bot.result_data = self.result_data
                    new_status = py_trees.common.Status.SUCCESS
                else:
                    #se siamo qui vuol dire che il risultato ancora non c'è
                    self.blackboard_input_bot.result_message = "RUNNING"
                    new_status = py_trees.common.Status.RUNNING

                #incerti di questa riga
                if(self.google_service.state == State.FAILED):
                    self.blackboard_input_bot.result_message = "FAILURE"
                    new_status = py_trees.common.Status.FAILURE
            
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
    
    blackboardProva = py_trees.blackboard.Client(name="blackboardProva", namespace="harmoni_input_bot")
    blackboardProva.register_key("result_data", access=py_trees.common.Access.READ)
    blackboardProva.register_key("result_message", access=py_trees.common.Access.READ)
    print(blackboardProva)

    sttPyTree = SpeechToTextServicePytree("SpeechToTextServicePytreeTest")

    additional_parameters = dict([
        ("SpeechToTextServicePytree_mode",False)])

    sttPyTree.setup(**additional_parameters)
    try:
        for unused_i in range(0, 7):
            sttPyTree.tick_once()
            time.sleep(0.5)
            print(blackboardProva)
        print("\n")
    except KeyboardInterrupt:
        print("Exception occurred")
        pass
    

if __name__ == "__main__":
    main()
