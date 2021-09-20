#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import *
from actionlib_msgs.msg import GoalStatus
from harmoni_common_lib.action_client import HarmoniActionClient
import harmoni_common_lib.helper_functions as hf
# Other Imports
from harmoni_common_lib.constants import SensorNameSpace

# Specific Imports
from harmoni_common_lib.constants import ActuatorNameSpace, ActionType, DialogueNameSpace
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

class CameraServicePytree(py_trees.behaviour.Behaviour):

    def __init__(self, name = "CameraServicePytree"):

        self.name = name
        self.result_data = None
        self.service_client_camera = None
        self.client_result = None 
        """
        # here there is the inizialization of the blackboards
        self.blackboards = []
        self.blackboard_camera = self.attach_blackboard_client(name=self.name, namespace=SensorNameSpace.camera.name)
        self.blackboard_camera.register_key("result_message", access=py_trees.common.Access.WRITE)
        """

        super(CameraServicePytree, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self,**additional_parameters):
        """
        for parameter in additional_parameters:
            print(parameter, additional_parameters[parameter])  
            if(parameter =="CameraServicePytree_mode"):
                self.mode = additional_parameters[parameter]     
        """

        #rospy init node mi fa diventare un nodo ros
        #rospy.init_node(self.service_name, log_level=rospy.INFO)
        
        self.service_client_camera = HarmoniActionClient(self.name)
        self.client_result = deque()
        self.server_name = "camera_default"
        self.service_client_camera.setup_client(self.server_name, 
                                            self._result_callback,
                                            self._feedback_callback)
        self.logger.debug("Behavior %s interface action clients have been set up!" % (self.server_name))
        
        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self):
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self):
        if self.service_client_camera.get_state() == GoalStatus.LOST:
            self.logger.debug(f"Sending goal to {self.server_name}")
            # Send request for each sensor service to set themselves up
            self.service_client_camera.send_goal(
                action_goal=ActionType["ON"].value,
                optional_data="Setup",
                wait="",
            )
            self.logger.debug(f"Goal sent to {self.server_name}")
            new_status = py_trees.common.Status.RUNNING
        else if self.service_client_camera.get_state() == GoalStatus.PENDING or self.service_client_camera.get_state() == GoalStatus.ACTIVE:
            new_status = py_trees.common.Status.RUNNING
        else if self.service_client_camera.get_state() == GoalStatus.SUCCEEDED:
            new_status = py_trees.common.Status.SUCCESS
        else:
            new_status = py_trees.common.Status.FAILURE

        self.logger.debug("%s.update()[%s]--->[%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status

        

    def terminate(self, new_status):
        if(new_status == py_trees.common.Status.INVALID):
            self.logger.debug(f"Sending goal to {self.server_name} to stop the service")
            # Send request for each sensor service to set themselves up
            self.service_client_camera.send_goal(
                action_goal=ActionType["STOP"].value,
                optional_data="",
                wait="",
            )
            self.logger.debug(f"Goal sent to {self.server_name}")
            self.client_result = deque()
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
    
    blackboardProva = py_trees.blackboard.Client(name="blackboardProva", namespace="harmoni_camera")
    blackboardProva.register_key("result_message", access=py_trees.common.Access.READ)


    print(blackboardProva)

    cameraPyTree = CameraServicePytree("CameraServicePytreeTest")

    additional_parameters = dict([
        ("CameraServicePytree_mode",False)])

    cameraPyTree.setup(**additional_parameters)
    try:
        for unused_i in range(0, 3):
            cameraPyTree.tick_once()
            time.sleep(0.5)
            print(blackboardProva)
        print("\n")
    except KeyboardInterrupt:
        print("Exception occurred")
        pass
    

