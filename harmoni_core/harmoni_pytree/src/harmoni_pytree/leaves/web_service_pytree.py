#!/usr/bin/env python3

# Common Imports
import rospy, rospkg, roslib

from harmoni_common_lib.constants import State, ActuatorNameSpace
from harmoni_common_lib.action_client import HarmoniActionClient
from actionlib_msgs.msg import GoalStatus
import harmoni_common_lib.helper_functions as hf

# Specific Imports
from harmoni_web.web_service import WebService
from harmoni_common_lib.constants import ActuatorNameSpace, ActionType, State
from contextlib import closing
from collections import deque 
import soundfile as sf
import numpy as np
import re
import json
import ast
import sys
import time

# import wget
import ast

#py_tree
import py_trees

class WebServicePytree(py_trees.behaviour.Behaviour):
    """
    the boolean "mode" changes the functioning of the Behaviour:
    true: we use the leaf as both client and server (inner module)
    false: we use the leaf as client that makes request to the server
    """

    def __init__(self, name = "WebServicePytree"):
        
        self.name = name
        self.mode = False
        self.web_service = None
        self.result_data = None
        self.service_client_web = None
        self.client_result = None

        self.blackboards = []
        #serve una blackboard a speaker?
        self.blackboard_web = self.attach_blackboard_client(name=self.name, namespace=ActuatorNameSpace.web.name)
        self.blackboard_web.register_key("result_data", access=py_trees.common.Access.WRITE)
        self.blackboard_web.register_key("result_message", access=py_trees.common.Access.WRITE)
        self.blackboard_input_web=self.attach_blackboard_client(name=self.name,namespace="input_web")
        self.blackboard_input_web.register_key("result_data", access=py_trees.common.Access.READ)
        self.blackboard_input_web.register_key("result_message", access=py_trees.common.Access.READ)


        super(WebServicePytree, self).__init__(name)
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
            if(parameter =="WebServicePytree_mode"):
                self.mode = additional_parameters[parameter]        

        service_name = ActuatorNameSpace.web.name
        instance_id = rospy.get_param("/instance_id")
        service_id = f"{service_name}_{instance_id}"

        self.web_service = WebService(service_id)

        #comment the following line if you are not running main

        if(not self.mode):
            self.service_client_web = HarmoniActionClient(self.name)
            self.client_result = deque()
            self.service_client_web.setup_client("web_default",
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
        #TODO check
        
        if(self.mode):
            if self.blackboard_input_web.result_message == State.SUCCESS:
                self.result_data = self.web_service.do(self.blackboard_input_web.result_data)
                self.blackboard_web.result_message = State.SUCCESS
                self.blackboard_web.result_data = self.result_data['message']
                new_status = py_trees.common.Status.SUCCESS
            else:
                #TODO add running state
                new_status = py_trees.common.Status.FAILURE
        else:
            if self.blackboard_input_web.result_message == State.SUCCESS:
                if self.service_client_web.get_state() == GoalStatus.LOST:
                    self.logger.debug(f"Sending goal to {self.web_service}")
                    self.service_client_web.send_goal(
                        action_goal = ActionType["DO"].value,
                        optional_data = self.blackboard_input_web.result_data,
                        wait=False,
                    )
                    self.logger.debug(f"Goal sent to {self.web_service}")
                    new_status = py_trees.common.Status.RUNNING
                else:
                    if len(self.client_result) > 0:
                        #if we reach this point we have the result(s) 
                            #so we can make the leaf terminate
                        self.result_data = self.client_result.popleft()["data"]
                        #if you want to see what is written in the result use --> self.result_data["response"]
                        self.blackboard_web.result_message = State.SUCCESS
                        self.blackboard_web.result_data = self.result_data
                        new_status = py_trees.common.Status.SUCCESS
                    else:
                        #if we are here it means that we dont have the result yet, so
                        #do we have to wait or something went wrong?
                        #not sure about the followings lines, see row 408 of sequential_pattern.py
                        if(self.web_service.state == State.FAILED):
                            self.blackboard_web.result_message = State.FAILED
                            new_status = py_trees.common.Status.FAILURE
                        else:
                            new_status = py_trees.common.Status.RUNNING
            else:
                new_status = self.blackboard_input_web.result_message
            
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

