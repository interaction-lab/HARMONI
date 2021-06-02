#!/usr/bin/env python3

# Common Imports
import rospy, rospkg, roslib

from harmoni_common_lib.constants import State, ActuatorNameSpace
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
from harmoni_common_lib.action_client import HarmoniActionClient
import harmoni_common_lib.helper_functions as hf
from speaker_service import SpeakerService

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
        self.aws_service = None
        self.result_data = None
        self.service_client_tts = None
        self.client_result = None
        self.optional_data = None

        self.blackboards = []
        self.blackboard = self.attach_blackboard_client(name=self.name, namespace="harmoni_tts")
        self.blackboard.register_key("result_data", access=py_trees.common.Access.READ)
        self.blackboard.register_key("result_message", access=py_trees.common.Access.READ)
        self.blackboard.result_message = "INVALID"

        super(SpeakerServicePyTree, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self,service_id,mode):
        """

        """
        self.mode = mode
        self.speaker_service = SpeakerService(self.name,service_id)
        rospy.init_node("speaker_default", log_level=rospy.INFO)
    
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
        if(self.mode):
            if self.blackboard.result_message == "SUCCESS":
                pass
            elif self.blackboard.result_message == "RUNNING":
                pass
            elif self.blackboard.result_message == "FAILURE":
                pass
            else:
                #lo stato è "INVALID"
                pass
        else:
            if self.blackboard.result_message == "SUCCESS":
                self.optional_data = self.blackboard.result_data
                self.logger.debug(f"Sending goal to {self.speaker_service}")
                self.service_client_speaker.send_goal(
                    action_goal = ActionType["DO"].value,
                    optional_data = self.optional_data,
                    wait=False,
                )
                self.logger.debug(f"Goal sent to {self.aws_service}")
                new_status = py_trees.common.Status.RUNNING
            elif self.blackboard.result_message == "INVALID":
                pass
            else:
                #lo stato o è "RUNNING" o è "FAILURE" e quindi in ogni caso sarà:
                new_status = self.blackboard.result_message
            
            
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
            pass

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
    pass
    

if __name__ == "__main__":
    main()
