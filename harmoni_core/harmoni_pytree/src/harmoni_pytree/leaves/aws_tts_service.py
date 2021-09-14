#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from actionlib_msgs.msg import GoalStatus
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
from harmoni_common_lib.action_client import HarmoniActionClient
import harmoni_common_lib.helper_functions as hf
from harmoni_tts.aws_tts_service import AWSTtsService
# Specific Imports
from harmoni_common_lib.constants import ActuatorNameSpace, ActionType, State, DialogueNameSpace
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

    #TODO log with pytree
    """
    the boolean "mode" changes the functioning of the Behaviour:
    true: we use the leaf as both client and server (inner module)
    false: we use the leaf as client that makes request to the server
    """
    #TTS è un actuators

    def __init__(self, name):
        
        """
        """
        self.name = name
        self.mode = False
        self.aws_service = None
        self.result_data = None
        self.service_client_tts = None
        self.client_result = None

        # here there is the inizialization of the blackboards
        self.blackboards = []
        self.blackboard_tts_OLD = self.attach_blackboard_client(name=self.name, namespace=ActuatorNameSpace.tts.name)
        self.blackboard_tts_OLD.register_key("result_data", access=py_trees.common.Access.WRITE)
        self.blackboard_tts_OLD.register_key("result_message", access=py_trees.common.Access.WRITE)
        self.blackboard_output_bot=self.attach_blackboard_client(name=self.name,namespace=DialogueNameSpace.bot.name+"output")
        self.blackboard_output_bot.register_key("result_data", access=py_trees.common.Access.READ)
        self.blackboard_output_bot.register_key("result_message", access=py_trees.common.Access.READ)

        #TODO: usa queste bb che sono le nuove
        self.blackboard_tts = self.attach_blackboard_client(name=self.name, namespace=ActuatorNameSpace.tts.name)
        self.blackboard_tts.register_key("result", access=py_trees.common.Access.WRITE)
        self.blackboard_bot = self.attach_blackboard_client(name=self.name, namespace=DialogueNameSpace.bot.name)
        self.blackboard_bot.register_key("result", access=py_trees.common.Access.READ)

        super(AWSTtsServicePytree, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self,**additional_parameters):
        """
        In order to select the mode after that the tree is created 
        an additional_parameters parameter is used:
        this parameter is a dictionary that contains couples like   
        name_of_the_leaf --> boolean mode
        """
        for parameter in additional_parameters:
            print(parameter, additional_parameters[parameter])  
            if(parameter ==ActuatorNameSpace.tts.name):
                self.mode = additional_parameters[parameter]        

        service_name = ActuatorNameSpace.tts.name
        instance_id = rospy.get_param("instance_id")

        param = rospy.get_param(service_name + "/" + instance_id + "_param/")

        self.aws_service = AWSTtsService(self.name,param)
        #TODO we have to do this in the if
        #rospy init node mi fa diventare un nodo ros
        #rospy.init_node("tts_default", log_level=rospy.INFO)

        self.blackboard_tts_OLD.result_message = "INVALID"

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
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self):
        """
        
        """
        if(self.mode):
            if self.blackboard_output_bot.result_message == State.SUCCESS:
                # results data contains all the response from the lex service (not only the sentence)
                self.result_data = self.aws_service.request(self.blackboard_output_bot.result_data["message"])
                self.blackboard_tts_OLD.result_message = State.SUCCESS
                self.blackboard_tts_OLD.result_data = self.result_data['message']
                new_status = py_trees.common.Status.SUCCESS
            else:
                new_status = self.blackboard_output_bot.result_message
        else:
            if self.blackboard_output_bot.result_message ==  State.SUCCESS:
                if self.service_client_tts.get_state() == GoalStatus.LOST:
                    self.logger.debug(f"Sending goal to {self.aws_service}")
                    #Where can we take details["action_goal"]?
                    rospy.loginfo(self.blackboard_output_bot.result_data)
                    self.service_client_tts.send_goal(
                        action_goal = ActionType["REQUEST"].value,
                        optional_data = self.blackboard_output_bot.result_data['message'],
                        wait=False,
                    )
                    self.logger.debug(f"Goal sent to {self.aws_service}")
                    new_status = py_trees.common.Status.RUNNING
                else:
                    if len(self.client_result) > 0:
                        #if we reach this point we have the result(s) 
                        #so we can make the leaf terminate
                        self.result_data = self.client_result.popleft()["data"]
                        self.blackboard_tts_OLD.result_message = State.SUCCESS
                        self.blackboard_tts_OLD.result_data = self.result_data
                        new_status = py_trees.common.Status.SUCCESS
                    else:
                        #not sure about the followings lines
                        if(self.aws_service.state == State.FAILED):
                            self.blackboard_tts_OLD.result_message = State.FAILED
                            new_status = py_trees.common.Status.FAILURE
                        else:
                            new_status = py_trees.common.Status.RUNNING
            else:
                new_status = self.blackboard_output_bot.result_message
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
            #do the code for handling interuptions
            #self.blackboard_tts_OLD.result_message = "INVALID"
            #TODO 
            if(self.mode):
                pass
            else:
                pass
        else:
            #do the code for the termination of the leaf (SUCCESS || FAILURE)
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