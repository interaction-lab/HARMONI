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

class AWSLexServicePytree(py_trees.behaviour.Behaviour):

    """
    the boolean "mode" changes the functioning of the Behaviour:
    true: we use the leaf as both client and server (inner module)
    false: we use the leaf as client that makes request to the server
    """

    def __init__(self, name):
        self.name = name
        self.mode = False
        self.aws_service = None
        self.input_message_lex = None
        self.result_data = None
        self.service_client_lex = None
        self.client_result = None

        self.blackboards = []
        self.blackboard_output_bot = self.attach_blackboard_client(name=self.name, namespace=DialogueNameSpace.bot.name+"output")
        self.blackboard_output_bot.register_key("result_data", access=py_trees.common.Access.WRITE)
        self.blackboard_output_bot.register_key("result_message", access=py_trees.common.Access.WRITE)
        self.blackboard_input_bot = self.attach_blackboard_client(name=self.name, namespace=DialogueNameSpace.bot.name)
        self.blackboard_input_bot.register_key("result_data", access=py_trees.common.Access.READ)
        self.blackboard_input_bot.register_key("result_message", access=py_trees.common.Access.READ)

        #TODO: usa queste bb che sono le nuove
        self.blackboard_scene = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name)
        self.blackboard_scene.register_key("utterance", access=py_trees.common.Access.READ)
        self.blackboard_bot = self.attach_blackboard_client(name=self.name, namespace=DialogueNameSpace.bot.name)
        self.blackboard_bot.register_key("result", access=py_trees.common.Access.WRITE)
        
        self.blackboard_stt = self.attach_blackboard_client(name=self.name, namespace=DetectorNameSpace.stt.name)
        self.blackboard_stt.register_key("result", access=py_trees.common.Access.READ)
        self.blackboard_card_detect = self.attach_blackboard_client(name=self.name, namespace=DetectorNameSpace.card_detect.name)
        self.blackboard_card_detect.register_key("result", access=py_trees.common.Access.READ)
        

        super(AWSLexServicePytree, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self,**additional_parameters):
        """
        In order to select the mode after that the tree is created 
        an additional_parameters parameter is used:
        this parameter is a dictionary that contains couples like
        name_of_the_leaf --> boolean mode
        """
        #for parameter in additional_parameters:
        #    print(parameter, additional_parameters[parameter])  
        #    if(parameter ==DialogueNameSpace.bot.name):
        #        self.mode = additional_parameters[parameter]        

        #service_name = DialogueNameSpace.bot.name
        #instance_id = rospy.get_param("instance_id")  # "default"
        #service_id = f"{service_name}_{instance_id}"

        #params = rospy.get_param(service_name + "/" + instance_id + "_param/")

        #self.aws_service = AWSLexService(service_id, params)
        #self.aws_service.setup_aws_lex()
<<<<<<< HEAD
        #if(not self.mode):
=======
        if(not self.mode):
>>>>>>> 875abbc975997aed210e7d918276353e4410dd40
            
        self.service_client_lex = HarmoniActionClient(self.name)
        rospy.loginfo("Client initialized")
        self.client_result = deque()
        self.server_name = "bot_default"
        self.service_client_lex.setup_client(self.server_name, 
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
        if(self.mode):
            if self.blackboard_input_bot.result_message == State.SUCCESS:
                self.input_message_lex = self.blackboard_input_bot.result_data
                #la risposta di aws_service.request è composta da due campi
                self.result_data = self.aws_service.request(self.input_message_lex)
                self.blackboard_output_bot.result_data = self.result_data["message"]
                self.blackboard_output_bot.result_message = State.SUCCESS
                #anche se vorresti scrivere self.blackboard_output_bot.result_message = self.result_data["response"]
                new_status = py_trees.common.Status.SUCCESS
            else:
                #lo stato o è "RUNNING" o è "FAILURE" e quindi in ogni caso sarà:
                self.blackboard_output_bot.result_message = self.blackboard_input_bot.result_message
                new_status = self.blackboard_input_bot.result_message
        else:
            
            if self.blackboard_input_bot.result_message == State.SUCCESS:
                #TODO non funziona il self.mode = false
                #ho già fatto la richiesta? se si non la faccio se no la faccio
                if self.service_client_lex.get_state() == GoalStatus.LOST:
                    self.input_message_lex = self.blackboard_input_bot.result_data
                    self.logger.debug(f"Sending goal to {self.aws_service}")
                    self.service_client_lex.send_goal(
                        action_goal = ActionType["REQUEST"].value,
                        optional_data = self.input_message_lex,
                        wait=False,
                    )
                    self.logger.debug(f"Goal sent to {self.aws_service}")
                    self.blackboard_output_bot.result_message = "RUNNING"
                    new_status = py_trees.common.Status.RUNNING
                else:
                    
                    if len(self.client_result) > 0:
                        #se siamo qui vuol dire che il risultato c'è e quindi 
                        #possiamo terminare la foglia
                        self.result_data = self.client_result.popleft()["data"]
                        self.blackboard_output_bot.result_message = State.SUCCESS
                        self.blackboard_output_bot.result_data = self.result_data
                        #se vuoi sapere cosa c'è scritto nel risultato usa self.result_data["response"]
                        new_status = py_trees.common.Status.SUCCESS
                    else:
                        #se siamo qui vuol dire che il risultato ancora non c'è, dunque
                        #si è rotto tutto o dobbiamo solo aspettare?
                        #incerti di questa riga, vedi 408 sequential_pattern.py
                        if(self.aws_service.state == State.FAILED):
                            self.blackboard_output_bot.result_message = State.FAILED
                            new_status = py_trees.common.Status.FAILURE
                        else:
                            new_status = py_trees.common.Status.RUNNING
            else:
                #lo stato o è "RUNNING" o è "FAILURE" e quindi in ogni caso sarà:
                new_status = self.blackboard_input_bot.result_message

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
            #self.blackboard_tts.result_message = "INVALID"
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


