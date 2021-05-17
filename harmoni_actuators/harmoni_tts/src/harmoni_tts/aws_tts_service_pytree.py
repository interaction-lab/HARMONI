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
    mode = false
    aws_service
    service_server

    def __init__(self, name):
        """
        Qui abbiamo pensato di chiamare soltanto 
        il costruttore del behaviour tree 
        """
        super(AWSTtsServicePytree, self).__init__(name)
        #Here we would like to put:
        #self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        print(("%s.__init__()" % (self.__class__.__name__)))

    def setup(self,name,param,mode):
        """
        Qui chiamiamo l'inizializzazione del servizio AWSTtsService, 
        motivo per cui abbiamo aggiunto param e name al metodo che 
        pensiamo debbano essere passati dal chiamante e non possono essere
        creati all'interno del metodo stesso.  
        """
        self.mode = mode
        if(mode):
            pass

        else:
            try:
                self.aws_service=AWSTtsService(name,param)
                self.service_server = HarmoniServiceServer(name, self.aws_service)
                self.service_server.start_sending_feedback()
                rospy.spin()
            except rospy.ROSInterruptException:
                pass
        
        
        #Here we would like to put:
        #self.logger.debug("%s.setup()" % (self.__class__.__name__))
        print("%s.setup()" % (self.__class__.__name__))

    def initialise(self):
        """
        
        """
        #Domanda: come facciamo a prendere il testo in input? 
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
            elif(self.aws_service.state==State.INVALID):
                new_status=py_trees.common.Status.RUNNING
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

def main():
    """[summary]
    Main function for starting HarmoniPolly service
    """
    

if __name__ == "__main__":
    main()
