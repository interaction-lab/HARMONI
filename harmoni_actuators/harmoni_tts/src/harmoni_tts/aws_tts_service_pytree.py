#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
from harmoni_common_lib.action_client import HarmoniActionClient
import harmoni_common_lib.helper_functions as hf
from harmoni_tts.aws_tts_service import AWSTtsService
# Specific Imports
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

#py_tree
import py_trees
import time

import py_trees.console

class AWSTtsServicePytree(py_trees.behaviour.Behaviour):

    #TODO tutte le print devono diventare console py_tree
    """
    mode è il boolean che controlla la modalità di funzionamento:
    true: opzione 1 (utilizzo come una classe python)
    false: opzione 2 (utilizzo mediate action_goal)
    """
    #TTS è un actuators

    def __init__(self, name = "AWSTtsServicePytree"):
        
        """
        Qui abbiamo pensato di chiamare soltanto 
        il costruttore del behaviour tree 
        """
        self.name = name
        self.mode = False
        self.aws_service = None
        self.result_data = None
        self.service_client_tts = None
        self.client_result = None

        self.blackboards = []
        self.blackboard_tts = self.attach_blackboard_client(name=self.name, namespace="harmoni_tts")
        self.blackboard_tts.register_key("result_data", access=py_trees.common.Access.WRITE)
        self.blackboard_tts.register_key("result_message", access=py_trees.common.Access.WRITE)

        super(AWSTtsServicePytree, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self,**additional_parameters):
        """
        Qui chiamiamo l'inizializzazione del servizio AWSTtsService, 
        motivo per cui abbiamo aggiunto param al metodo che 
        pensiamo debbano essere passati dal chiamante e non possono essere
        creati all'interno del metodo stesso.  
        """
        for parameter in additional_parameters:
            print(parameter, additional_parameters[parameter])  
            if(parameter =="AWSTtsServicePytree_mode"):
                print("Setto la modalità")
                self.mode = additional_parameters[parameter]        

        service_name = ActuatorNameSpace.tts.name
        instance_id = rospy.get_param("instance_id")

        param = rospy.get_param(service_name + "/" + instance_id + "_param/")

        self.aws_service = AWSTtsService(self.name,param)
        rospy.init_node("tts_default", log_level=rospy.INFO)

        self.blackboard_tts.result_message = "INVALID"

        if(not self.mode):
            self.service_client_tts = HarmoniActionClient(self.name)
            self.client_result = deque()
            self.service_client_tts.setup_client("tts_default", 
                                                self._result_callback,
                                                self._feedback_callback)
            self.logger.debug("Behavior interface action clients have been set up!")
        
        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self):
        """
        
        """
        #il result message dice anche in che stato è la foglia
        self.blackboard_tts.result_message = "RUNNING"
        self.blackboard_tts.result_data = ""
        #TODO prendi l'input text da una blackboard
        input_text="my name is Corrado, nice to meet you"
        #TODO Queste cose devono andare nell'update, 
        #dopo aver controllato di avere o meno input_text
        if(self.mode):
            self.result_data = self.aws_service.request(input_text)
        else:
            self.logger.debug(f"Sending goal to {self.aws_service} optional_data len {len(input_text)}")

            # Dove posso prendere details["action_goal"]?
            self.service_client_tts.send_goal(
                action_goal = ActionType["REQUEST"].value,
                optional_data = input_text,
                wait=False,
            )
            self.logger.debug(f"Goal sent to {self.aws_service}")
            
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))
    def update(self):
        """
        
        """
        #TODO rivedi
        if(self.mode):
            if(self.result_data["response"] == State.SUCCESS):
                self.blackboard_tts.result_message = "SUCCESS"
                self.blackboard_tts.result_data = self.result_data['message']
                self.result_data = self.result_data['message']
                new_status = py_trees.common.Status.SUCCESS
            else:
                self.blackboard_tts.result_message = "FAILURE"
                new_status = py_trees.common.Status.FAILURE
        else:
            #non siamo sicuro degli stati
            if len(self.client_result) > 0:
                #se siamo qui vuol dire che il risultato c'è e quindi 
                #possiamo terminare la foglia
                self.result_data = self.client_result.popleft()["data"]
                self.blackboard_tts.result_message = "SUCCESS"
                self.blackboard_tts.result_data = self.result_data
                new_status = py_trees.common.Status.SUCCESS
            else:
                #se siamo qui vuol dire che il risultato ancora non c'è
                self.blackboard_tts.result_message = "RUNNING"
                new_status = py_trees.common.Status.RUNNING

            #incerti di questa riga
            if(self.aws_service.state == State.FAILED):
                self.blackboard_tts.result_message = "FAILURE"
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
            self.blackboard_tts.result_message = "INVALID"
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
    
    blackboardProva = py_trees.blackboard.Client(name="blackboardProva", namespace="harmoni_tts")
    blackboardProva.register_key("result_data", access=py_trees.common.Access.READ)
    blackboardProva.register_key("result_message", access=py_trees.common.Access.READ)
    print(blackboardProva)

    ttsPyTree = AWSTtsServicePytree("AwsPyTreeTest")

    additional_parameters = dict([
        ("mode",False)])

    ttsPyTree.setup(**additional_parameters)
    try:
        for unused_i in range(0, 7):
            ttsPyTree.tick_once()
            time.sleep(0.5)
            print(blackboardProva)
        print("\n")
    except KeyboardInterrupt:
        print("Exception occurred")
        pass
    

if __name__ == "__main__":
    main()
