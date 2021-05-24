#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
from harmoni_common_lib.action_client import HarmoniActionClient
import harmoni_common_lib.helper_functions as hf
from aws_tts_service import AWSTtsService
# Specific Imports
from harmoni_common_lib.constants import ActuatorNameSpace
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

import py_trees.console as console

class AWSTtsServicePytree(py_trees.behaviour.Behaviour):


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

        super(AWSTtsServicePytree, self).__init__(name)
        #Here we would like to put:
        #self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        print(("%s.__init__()" % (self.__class__.__name__)))

    def setup(self,param,mode):
        """
        Qui chiamiamo l'inizializzazione del servizio AWSTtsService, 
        motivo per cui abbiamo aggiunto param al metodo che 
        pensiamo debbano essere passati dal chiamante e non possono essere
        creati all'interno del metodo stesso.  
        """
        self.mode = mode
        self.aws_service = AWSTtsService(self.name,param)
        if(not self.mode):
            self.service_client_tts = HarmoniActionClient(self.name)
            self.client_result = deque()
            self.service_client_tts.setup_client(self.name, self._result_callback, self._feedback_callback)
            rospy.loginfo("Behavior interface action clients have been set up!")
        
        
        #Here we would like to put:
        #self.logger.debug("%s.setup()" % (self.__class__.__name__))
        print("%s.setup()" % (self.__class__.__name__))

    def initialise(self):
        """
        
        """
        #TODO: blackboards per inputText
        input_text="ciao CT, stiamo provando"
        if(self.mode):
            self.aws_service.request(input_text)
            #Qui non abbiamo capito dove andare a prendere il risultato, (dovremmo chiedercelo anche nell' update)
        else:
            rospy.loginfo(f"Sending goal to {self.aws_service.__name__} optional_data len {len(input_text)}")

            # Dove posso prendere details["action_goal"]?
            self.service_client_tts.send_goal(
                action_goal=ActionType[details["action_goal"]].value,
                optional_data=input_text,
                wait=False,
            )
            rospy.loginfo(f"Goal sent to {self.aws_service.__name__}")
            
        #Here we would like to put:
        #self.logger.debug("%s.initialise()" % (self.__class__.__name__))
        print("%s.initialise()" % (self.__class__.__name__))
    def update(self):
        """
        
        """
        if(self.mode):
            #non abbiamo capito dove prendere il risultato
            if(self.aws_service.state == State.REQUEST):
                new_status = py_trees.common.Status.RUNNING
            elif(self.aws_service.state == State.SUCCESS):
                new_status = py_trees.common.Status.SUCCESS
            else:
                new_status = py_trees.common.Status.FAILURE
        else:
            #non siamo sicuro degli stati
            if len(self.client_result) > 0:
                rospy.loginfo("getting result from the service")
                rospy.loginfo(f"Queue is {self.client_result}")
                rospy.loginfo(f"Queue size is {len(self.client_result)}")
                #se siamo qui vuol dire che il risultato c'è e quindi possiamoterninare la foglia
                self.return_data = self.client_result.popleft()["data"]
                new_status = py_trees.common.Status.SUCCESS
            else:
                #se siamo qui vuol dire che il risultato ancora non c'è
                new_status = py_trees.common.Status.RUNNING

            #incerti di questa riga
            if(self.service_client_tts.state == State.FAILED):
                new_status = py_trees.common.Status.FAILURE

        #Here we would like to put:
        #self.logger.debug("%s.update()[%s]--->[%s]" % (self.__class__.__name__, self.status, new_status))
        print("%s.update()[%s]--->[%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status

        

    def terminate(self, new_status):
        """
        When is this called?
           Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """
        if(self.mode):
            self.mode = False
            self.aws_service = None
        else:
            self.service_client_tts = None
            self.client_result = deque()

        #Here we would like to put:
        #self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))
        print("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

    def _result_callback(self, result):
        """ Recieve and store result with timestamp """
        rospy.loginfo("The result of the request has been received")
        rospy.loginfo(
            f"The result callback message from {result['service']} was {len(result['message'])} long"
        )
        self.client_result.append(
            {"time": time(), "data": result["message"]}
        )
        # TODO add handling of errors and continue=False
        return

    def _feedback_callback(self, feedback):
        """ Feedback is currently just logged """
        rospy.logdebug("The feedback recieved is %s." % feedback)
        # Check if the state is end, stop the behavior pattern
        # if feedback["state"] == State.END:
        #    self.end_pattern = True
        return

def main():
    """[summary]
    Main function for starting HarmoniPolly service
    """
    #command_line_argument_parser().parse_args()

    py_trees.logging.level = py_trees.logging.Level.DEBUG
    service_name = ActuatorNameSpace.tts.name
    instance_id = rospy.get_param("instance_id")
    ttsPyTree = AWSTtsServicePytree("AwsPyTreeTest")

    param = rospy.get_param(service_name + "/" + instance_id + "_param/")
    ttsPyTree.setup(param,False)
    try:
        for unused_i in range(0, 7):
            ttsPyTree.tick_once()
            time.sleep(0.5)
        print("\n")
    except KeyboardInterrupt:
        print("Exception occurred")
        pass
    

if __name__ == "__main__":
    main()
