#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import *
from actionlib_msgs.msg import GoalStatus
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_imageai.custom_service import ImageAICustomService
import harmoni_common_lib.helper_functions as hf

# Specific Imports
from harmoni_common_lib.constants import DetectorNameSpace, ActionType
from sensor_msgs.msg import Image
from imageai.Detection.Custom import CustomVideoObjectDetection
from botocore.exceptions import BotoCoreError, ClientError
from contextlib import closing
from collections import deque 
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

    def __init__(self, name = "SpeechToTextServicePytree"):
    
        self.name = name
        self.service_client_stt = None
        self.client_result = None
        self.server_state = None
        self.server_name = None
        
        self.blackboards = []
        self.blackboard_microphone = self.attach_blackboard_client(name=self.name, namespace=SensorNameSpace.microphone.name)
        self.blackboard_microphone.register_key("state", access=py_trees.common.Access.READ)
        self.blackboard_stt = self.attach_blackboard_client(name=self.name, namespace=DetectorNameSpace.stt.name)
        self.blackboard_stt.register_key("result", access=py_trees.common.Access.WRITE)

        super(SpeechToTextServicePytree, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self,**additional_parameters):
        """
         for parameter in additional_parameters:
            print(parameter, additional_parameters[parameter])  
            if(parameter =="SpeechToTextServicePytree_mode"):
                self.mode = additional_parameters[parameter]
        """
        self.service_client_stt = HarmoniActionClient(self.name)
        self.server_name = "stt_default"
        self.service_client_stt.setup_client(self.server_name, 
                                            self._result_callback,
                                            self._feedback_callback)
        self.logger.debug("Behavior %s interface action clients have been set up!" % (self.server_name))

        self.blackboard_stt.result = "null"

        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self):
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self):
        new_state = self.service_client_stt.get_state()
        print(new_state)
        if new_state == GoalStatus.LOST:
            self.logger.debug(f"Sending goal to {self.server_name}")
            self.service_client_stt.send_goal(
                action_goal = ActionType["REQUEST"].value,
                optional_data="",
                wait=False,
            )
            self.logger.debug(f"Goal sent to {self.server_name}")
            new_status = py_trees.common.Status.RUNNING
        elif new_state == GoalStatus.PENDING or new_state == GoalStatus.ACTIVE:
            #there is no result yet
            new_status = py_trees.common.Status.RUNNING
        elif new_state == GoalStatus.SUCCEEDED:
            if self.client_result is not None:
                self.blackboard_stt.result = self.client_result
                self.client_result = None
                new_status = py_trees.common.Status.SUCCESS
            else:
                #we haven't received the result correctly.
                new_status = py_trees.common.Status.FAILURE
        else: 
            new_status = py_trees.common.Status.FAILURE
        
        self.logger.debug("%s.update()[%s]--->[%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status

        
    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID:
            new_state = self.service_client_stt.get_state()
            if new_state != GoalStatus.LOST:
                self.logger.debug(f"Cancelling goal to {self.server_name}")
                self.service_client_stt.cancel_goal()
                self.client_result = None
                #self.blackboard_stt.result = None
                self.logger.debug(f"Goal cancelled to {self.server_name}")
                self.service_client_stt.stop_tracking_goal()
                self.logger.debug(f"Goal tracking stopped to {self.server_name}")
        else:
            #execute actions for the following states (SUCCESS || FAILURE)
            pass
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

    def _result_callback(self, result):
        """ Recieve and store result with timestamp """
        self.logger.debug("The result of the request has been received")
        self.logger.debug(
            f"The result callback message from {result['service']} was {len(result['message'])} long"
        )
        self.client_result = result["message"]
        return

    def _feedback_callback(self, feedback):
        """ Feedback is currently just logged """
        self.logger.debug("The feedback recieved is %s." % feedback)
        self.server_state = feedback["state"]
        return

def main():
    #command_line_argument_parser().parse_args()

    py_trees.logging.level = py_trees.logging.Level.DEBUG
    
    blackboardProva = py_trees.blackboard.Client(name="blackboardProva", namespace=DetectorNameSpace.stt.name)
    blackboardProva.register_key("result", access=py_trees.common.Access.READ)
    print(blackboardProva)

    #rospy init node mi fa diventare un nodo ros
    rospy.init_node("stt_default", log_level=rospy.INFO)
    
    sttPyTree = SpeechToTextServicePytree("GoogleSSTPytreeTest")

    sttPyTree.setup()
    try:
        for unused_i in range(0, 20):
            sttPyTree.tick_once()
            time.sleep(2)
            print(blackboardProva)
        print("\n")
    except KeyboardInterrupt:
        print("Exception occurred")
        pass

if __name__ == "__main__":
    main()
