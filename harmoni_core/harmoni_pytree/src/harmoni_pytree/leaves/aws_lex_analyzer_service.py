#!/usr/bin/env python3

# Common Imports
import rospy
from harmoni_common_lib.constants import *
from actionlib_msgs.msg import GoalStatus
from harmoni_common_lib.action_client import HarmoniActionClient
import harmoni_common_lib.helper_functions as hf
from harmoni_bot.aws_lex_service import AWSLexService

# Specific Imports
from harmoni_common_lib.constants import ActuatorNameSpace, ActionType, DialogueNameSpace
from botocore.exceptions import BotoCoreError, ClientError
from contextlib import closing
from collections import deque 
import soundfile as sf
import numpy as np
import re
import json
import ast
import sys

#py_tree
import py_trees
import time

import py_trees.console

class AWSLexAnalyzerServicePytree(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        self.name = name
        self.server_state = None
        self.service_client_lex = None
        self.client_result = None

        self.blackboards = []
        """
        self.blackboard_output_bot = self.attach_blackboard_client(name=self.name, namespace=DialogueNameSpace.bot.name+"output")
        self.blackboard_output_bot.register_key("result_data", access=py_trees.common.Access.WRITE)
        self.blackboard_output_bot.register_key("result_message", access=py_trees.common.Access.WRITE)
        self.blackboard_input_bot = self.attach_blackboard_client(name=self.name, namespace=DialogueNameSpace.bot.name)
        self.blackboard_input_bot.register_key("result_data", access=py_trees.common.Access.READ)
        self.blackboard_input_bot.register_key("result_message", access=py_trees.common.Access.READ)
        """

        #TODO: usa queste bb che sono le nuove
        #TODO creare due foglie per il bot uno che si occupa del trigger e viene messo in EOR con scene dove useremo solo
        #json e il secondo bot che invece usiamo come analyzer. 
        
        self.blackboard_stt = self.attach_blackboard_client(name=self.name, namespace=DetectorNameSpace.stt.name)
        self.blackboard_stt.register_key("result", access=py_trees.common.Access.READ)
        self.blackboard_card_detect = self.attach_blackboard_client(name=self.name, namespace=DetectorNameSpace.card_detect.name)
        self.blackboard_card_detect.register_key("result", access=py_trees.common.Access.READ)
        self.blackboard_bot = self.attach_blackboard_client(name=self.name, namespace=DialogueNameSpace.bot.name)
        self.blackboard_bot.register_key("result", access=py_trees.common.Access.WRITE)
        
        super(AWSLexAnalyzerServicePytree, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self,**additional_parameters):
        """
        for parameter in additional_parameters:
            print(parameter, additional_parameters[parameter])  
            if(parameter ==DialogueNameSpace.bot.name):
                self.mode = additional_parameters[parameter]        
        """
        self.service_client_lex = HarmoniActionClient(self.name)
        self.server_name = "bot_default"
        self.service_client_lex.setup_client(self.server_name, 
                                            self._result_callback,
                                            self._feedback_callback)
        self.logger.debug("Behavior %s interface action clients have been set up!" % (self.server_name))
        
        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self):           
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self):
        if self.server_state == State.INIT:
            if self.blackboard_card_detect.result != "null":
                #metti in input quello che è nella bb
                self.logger.debug(f"Sending goal to {self.server_name}")
                self.service_client_lex.send_goal(
                    action_goal = ActionType["REQUEST"].value,
                    optional_data=self.blackboard_card_detect.result,
                    wait=False,
                )
                self.logger.debug(f"Goal sent to {self.server_name}")
                new_status = py_trees.common.Status.RUNNING
            elif self.blackboard_stt.result != "null":
                #metti in input quello che è nella bb
                self.logger.debug(f"Sending goal to {self.server_name}")
                self.service_client_lex.send_goal(
                    action_goal = ActionType["REQUEST"].value,
                    optional_data=self.blackboard_stt.result,
                    wait=False,
                )
                self.logger.debug(f"Goal sent to {self.server_name}")
                new_status = py_trees.common.Status.RUNNING
            else:
                self.blackboard_bot.result = "null"
                new_status = py_trees.common.Status.SUCCESS
        elif self.server_state == State.REQUEST:
            #there is no result yet
            new_status = py_trees.common.Status.RUNNING
        elif self.server_state == State.SUCCESS:
            if self.client_result is not None:
                self.blackboard_bot.result = self.client_result
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
        if(new_status == py_trees.common.Status.INVALID):
            self.logger.debug(f"Sending goal to {self.server_name} to stop the service")
            # Send request for each sensor service to set themselves up
            self.service_client_lex.send_goal(
                action_goal=ActionType["OFF"].value,
                optional_data="",
                wait=False,
            )
            self.client_result = None
            self.blackboard_bot.result = None
            self.logger.debug(f"Goal sent to {self.server_name}")
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