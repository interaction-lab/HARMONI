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

class ImageAICustomServicePytree(py_trees.behaviour.Behaviour):

    #TODO tutte le print devono diventare console py_tree
    """
    mode è il boolean che controlla la modalità di funzionamento:
    true: opzione 1 (utilizzo come una classe python)
    false: opzione 2 (utilizzo mediate action_goal)
    """
    #ImageAICustom è un detector

    def __init__(self, name = "ImageAICustomServicePytree"):
        
        """
        Here there is just the constructor of the
        behaviour tree 
        """
        self.name = name
        self.mode = False
        self.custom_service = None
        self.result_data = None
        self.service_client_custom = None
        self.client_result = None

        # here there is the inizialization of the blackboards
        self.blackboards = []
        #blackboard we suppose are useful to know when to start imageai detection
        self.blackboard_camera=self.attach_blackboard_client(name=self.name,namespace="harmoni_camera")
        self.blackboard_camera.register_key("result_message", access=py_trees.common.Access.READ)
        #blackboard used to comunicate with aws_lex (bot)
        self.blackboard_custom=self.attach_blackboard_client(name=self.name,namespace="harmoni_imageai_custom")
        self.blackboard_custom.register_key("result_data",access=py_trees.common.Access.WRITE)
        self.blackboard_custom.register_key("result_message", access=py_trees.common.Access.WRITE)

        #TODO DOVREBBERO ESSERE QUESTE LE GIUSTE
        self.blackboard_camera = self.attach_blackboard_client(name=self.name, namespace=SensorNameSpace.camera.name+"/external")
        self.blackboard_camera.register_key("state", access=py_trees.common.Access.READ)
        self.blackboard_card_detection = self.attach_blackboard_client(name=self.name, namespace=DetectorNameSpace.card_detect.name)
        self.blackboard_card_detection.register_key("state", access=py_trees.common.Access.WRITE)
        self.blackboard_card_detection.register_key("result", access=py_trees.common.Access.WRITE)


        super(ImageAICustomServicePytree, self).__init__(name)
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
            if(parameter =="ImageAICustomServicePytree_mode"):
                self.mode = additional_parameters[parameter]        

        #service_name = DetectorNameSpace.imageai.name
        #instance_id = rospy.get_param("instance_id")

        #param = rospy.get_param(service_name + "/" + instance_id + "_param/")

        #self.custom_service = ImageAICustomService(self.name,param)

        #TODO questo dobbiamo farlo nell'if 
        #rospy init node mi fa diventare un nodo ros
        rospy.init_node("imageai_default", log_level=rospy.INFO)

        if(not self.mode):
            self.service_client_custom = HarmoniActionClient(self.name)
            self.client_result = deque()
            self.server_name = "imageai_default"
            self.service_client_custom.setup_client(self.server_name, 
                                                self._result_callback,
                                                self._feedback_callback)
            self.logger.debug("Behavior %s interface action clients have been set up!" % (self.server_name))
        
        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self):
        """
        
        """ 
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self):
        """
        
        """
        if self.service_client_custom.get_state() == GoalStatus.LOST:
                self.logger.debug(f"Sending goal to {self.google_service}")
                # Dove posso prendere details["action_goal"]?
                self.service_client_custom.send_goal(
                    action_goal = ActionType["REQUEST"].value,
                    optional_data="",
                    wait=False,
                )
                self.logger.debug(f"Goal sent to {self.google_service}")
                new_status = py_trees.common.Status.RUNNING
        else:
            if len(self.client_result) > 0:
                #if we are here, it means that there is the result so we can
                #terminate the leaf
                self.result_data = self.client_result.popleft()["data"]
                self.blackboard_custom.result_message = "SUCCESS"
                self.blackboard_custom.result_data = self.result_data
                new_status = py_trees.common.Status.SUCCESS
            else:
                #there is no result yet
                self.blackboard_custom.result_message = "RUNNING"
                new_status = py_trees.common.Status.RUNNING

            #not sure about these lines
            if(self.service_client_custom.get_state() == State.FAILED):
                self.blackboard_custom.result_message = "FAILURE"
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
    
    blackboardProva = py_trees.blackboard.Client(name="blackboardProva", namespace="harmoni_imageai_custom")
    blackboardProva.register_key("result_data", access=py_trees.common.Access.READ)
    blackboardProva.register_key("result_message", access=py_trees.common.Access.READ)
    print(blackboardProva)

    customPyTree = ImageAICustomServicePytree("ImageAICustomServicePytreeTest")

    additional_parameters = dict([
        ("ImageAICustomServicePytree_mode",False)])

    customPyTree.setup(**additional_parameters)
    try:

        for unused_i in range(0, 7):
            customPyTree.tick_once()
            time.sleep(0.5)
            print(blackboardProva)
        print("\n")
    except KeyboardInterrupt:
        print("Exception occurred")
        pass
    

if __name__ == "__main__":
    main()
