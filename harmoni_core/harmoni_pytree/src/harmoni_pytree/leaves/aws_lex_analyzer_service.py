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
        self.send_request = True

        self.blackboards = []
        
        self.blackboard_stt = self.attach_blackboard_client(name=self.name, namespace=DetectorNameSpace.stt.name)
        self.blackboard_stt.register_key("result", access=py_trees.common.Access.READ)
        self.blackboard_card_detect = self.attach_blackboard_client(name=self.name, namespace=DetectorNameSpace.card_detect.name)
        self.blackboard_card_detect.register_key("result", access=py_trees.common.Access.READ)
        self.blackboard_buttons = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.buttons.name)
        self.blackboard_buttons.register_key("result", access=py_trees.common.Access.READ)
        self.blackboard_bot = self.attach_blackboard_client(name=self.name, namespace=DialogueNameSpace.bot.name+"/"+ PyTreeNameSpace.analyzer.name)
        self.blackboard_bot.register_key("result", access=py_trees.common.Access.WRITE)
        self.blackboard_mainactivity = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.mainactivity.name)
        self.blackboard_mainactivity.register_key("counter_no_answer", access=py_trees.common.Access.WRITE)
        
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
        
        self.blackboard_bot.result = "null"
        self.blackboard_mainactivity.counter_no_answer = 0

        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self):           
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self):
        if self.send_request:
            self.send_request = False
            if self.blackboard_card_detect.result != "null":
                self.logger.debug(f"Sending  again goal to {self.server_name}")
                self.service_client_lex.send_goal(
                    action_goal = ActionType["REQUEST"].value,
                    optional_data=self.blackboard_card_detect.result,
                    wait=False,
                )
                self.logger.debug(f"Goal sent to {self.server_name}")
                new_status = py_trees.common.Status.RUNNING
            elif self.blackboard_stt.result != "null":
                self.logger.debug(f"Sending goal to {self.server_name}")
                self.service_client_lex.send_goal(
                    action_goal = ActionType["REQUEST"].value,
                    optional_data=self.blackboard_stt.result,
                    wait=False,
                )
                self.logger.debug(f"Goal sent to {self.server_name}")
                new_status = py_trees.common.Status.RUNNING
            elif self.blackboard_buttons.result != "null":
                self.logger.debug(f"Sending goal to {self.server_name}")
                self.service_client_lex.send_goal(
                    action_goal = ActionType["REQUEST"].value,
                    optional_data=self.blackboard_buttons.result,
                    wait=False,
                )
                self.logger.debug(f"Goal sent to {self.server_name}")
                new_status = py_trees.common.Status.RUNNING
            else:
                self.blackboard_mainactivity.counter_no_answer += 1 
                self.blackboard_bot.result = "void_answer"
                new_status = py_trees.common.Status.SUCCESS
        else:
            new_state = self.service_client_lex.get_state()
            print("update : ",new_state)
            if new_state == GoalStatus.ACTIVE:
                new_status = py_trees.common.Status.RUNNING
            elif new_state == GoalStatus.SUCCEEDED:
                if self.client_result is not None:
                    self.blackboard_bot.result = eval(self.client_result)
                    self.client_result = None
                    new_status = py_trees.common.Status.SUCCESS
                else:
                    self.logger.debug(f"Waiting fot the result ({self.server_name})")
                    new_status = py_trees.common.Status.RUNNING
            elif new_state == GoalStatus.PENDING:
                self.send_request = True
                self.logger.debug(f"Cancelling goal to {self.server_name}")
                self.service_client_lex.cancel_all_goals()
                self.client_result = None
                self.logger.debug(f"Goal cancelled to {self.server_name}")
                #self.service_client_lex.stop_tracking_goal()
                #self.logger.debug(f"Goal tracking stopped to {self.server_name}")
                new_status = py_trees.common.Status.RUNNING
            else:
                new_status = py_trees.common.Status.FAILURE
                raise

        self.logger.debug("%s.update()[%s]--->[%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status

    def terminate(self, new_status):
        new_state = self.service_client_lex.get_state()
        print("terminate : ",new_state)
        if new_state == GoalStatus.SUCCEEDED or new_state == GoalStatus.ABORTED or new_state == GoalStatus.LOST:
            self.send_request = True
        if new_state == GoalStatus.PENDING:
            self.send_request = True
            self.logger.debug(f"Cancelling goal to {self.server_name}")
            self.service_client_lex.cancel_all_goals()
            self.client_result = None
            self.logger.debug(f"Goal cancelled to {self.server_name}")
            #self.service_client_lex.stop_tracking_goal()
            #self.logger.debug(f"Goal tracking stopped to {self.server_name}")
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
