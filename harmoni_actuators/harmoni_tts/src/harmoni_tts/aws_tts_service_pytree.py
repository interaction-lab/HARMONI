#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf
from harmoni_tts import AWSTtsService
# Specific Imports
from harmoni_common_lib.constants import ActuatorNameSpace
from botocore.exceptions import BotoCoreError, ClientError
from contextlib import closing
import soundfile as sf
import numpy as np
import boto3
import re
import json
import ast
import sys

class AWSTtsServicePytree(py_trees.behaviour.Behaviour):


    """
    mode è il boolean che controlla la modalità di funzionamento:
    true: opzione 1 (utilizzo come una classe python)
    false: opzione 2 (utilizzo mediate action_goal)
    """

    def __init__(self, name = "AWSTtsServicePytree"):
        """
        Qui abbiamo pensato di chiamare soltanto 
        il costruttore del behaviour tree 
        """
        self.name = name
        self.mode = false
        self.aws_service = None

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
        self.aws_service=AWSTtsService(self.name,param)
        if(not mode):
            self.service_client_tts = HarmoniActionClient(self.name)
            self.client_result = deque()
            self.service_client_tts.setup_client(name, self._result_callback, self._feedback_callback)
            #Here we would like to put:
            #rospy.loginfo("Behavior interface action clients have been set up!")
            print("Behavior interface action clients have been set up!")
        
        
        #Here we would like to put:
        #self.logger.debug("%s.setup()" % (self.__class__.__name__))
        print("%s.setup()" % (self.__class__.__name__))

    def initialise(self):
        """
        
        """
        #TODO: blackboards per inputText
        input_text="ciao CT, stiamo provando"
        if(mode):
            self.aws_service.request(input_text)
        else:
            pass
            
        #Here we would like to put:
        #self.logger.debug("%s.initialise()" % (self.__class__.__name__))
        print("%s.initialise()" % (self.__class__.__name__))
    def update(self):
        """
        
        """
        if(mode):
            if(self.aws_service.state==State.REQUEST):
                new_status=py_trees.common.Status.RUNNING
            elif(self.aws_service.state==State.SUCCESS):
                new_status=py_trees.common.Status.SUCCESS
            else:
                new_status=py_trees.common.Status.FAILURE
        else:
            pass
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
        if(mode):
            pass

        else:
            pass
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
    

if __name__ == "__main__":
    main()
