#!/usr/bin/env python3

# Common Imports
import rospy, rospkg, roslib

from harmoni_common_lib.constants import *
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
from harmoni_common_lib.action_client import HarmoniActionClient
from actionlib_msgs.msg import GoalStatus
import harmoni_common_lib.helper_functions as hf
from harmoni_face.face_service import EyesService, MouthService, NoseService
from harmoni_face.face_client import Face

# Specific Imports
from audio_common_msgs.msg import AudioData
from harmoni_common_lib.constants import ActuatorNameSpace, ActionType, State
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
import time

# import wget
import contextlib
import ast
import wave
import os

#py_tree
import py_trees

class FaceServicePytree(py_trees.behaviour.Behaviour):
    def __init__(self, name):

        self.name = name
        self.service_client_mouth = None
        self.service_client_eyes = None
        self.service_client_nose = None
        self.client_result = None
        
        # here there is the inizialization of the blackboards
        self.blackboards = []

        #TODO: usa queste bb che sono le nuove
        self.blackboard_tts = self.attach_blackboard_client(name=self.name, namespace=ActuatorNameSpace.tts.name)
        self.blackboard_tts.register_key("result", access=py_trees.common.Access.READ)
        
        #lips_sync
        self.blackboard_lips = self.attach_blackboard_client(name=self.name, namespace=Resources.face.value[1])
        self.blackboard_lips.register_key("state", access=py_trees.common.Access.WRITE)
        
        #face_exp
        self.blackboard_scene = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name)
        self.blackboard_scene.register_key("face_exp", access=py_trees.common.Access.READ)

        super(FaceServicePytree, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self,**additional_parameters):
        """
        for parameter in additional_parameters:
            print(parameter, additional_parameters[parameter])  
            if(parameter == ActuatorNameSpace.face.name):
                self.mode = additional_parameters[parameter]        
        """
        self.service_client_face = HarmoniActionClient(self.name)
        self.server_name = "face"
        
        self.service_client_face.setup_client(self.server_name, self._result_callback, self._feedback_callback)
        
        self.instance_id = "default"
        self.name_mouth = ActuatorNameSpace.face.name + "_mouth_" + self.instance_id
        self.service_client_mouth = HarmoniActionClient(self.name_mouth)
        self.name_nose = ActuatorNameSpace.face.name + "_nose_" + self.instance_id
        self.service_client_nose = HarmoniActionClient(self.name_nose)
        self.name_eyes = ActuatorNameSpace.face.name + "_eyes_" + self.instance_id
        self.service_client_eyes = HarmoniActionClient(self.name_eyes)
        self.service_client_mouth.setup_client(self.name_mouth, self._result_callback, self._feedback_callback)
        self.service_client_eyes.setup_client(self.name_eyes, self._result_callback, self._feedback_callback)
        self.service_client_nose.setup_client(self.name_nose, self._result_callback, self._feedback_callback)
        
        self.logger.debug("Behavior %s interface action clients have been set up!" % (self.server_name))
        
        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self):
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self):

            if self.blackboard_tts.result_message == State.SUCCESS:
                #have I already done the request? check for this
                if self.service_client_face.get_state() == GoalStatus.LOST:
                    self.audio_data = self.blackboard_tts.result_data
                    self.logger.debug(f"Sending goal to {self.mouth_service} and {self.eyes_service}")
                    self.service_client_face.send_goal(
                        action_goal = ActionType["DO"].value,
                        optional_data = self.audio_data,
                        wait=False,
                    )
                    self.logger.debug(f"Goal sent to {self.mouth_service} and {self.eyes_service}")
                    new_status = py_trees.common.Status.RUNNING
                else:
                    if len(self.client_result) > 0:
                        #if we reach this point we have the result(s) 
                        #so we can make the leaf terminate
                        self.result_data = self.client_result.popleft()["data"]
                        new_status = py_trees.common.Status.SUCCESS
                    else:
                        #if we are here it means that we dont have the result yet, so
                        #do we have to wait or something went wrong?
                        #not sure about the followings lines, see row 408 of sequential_pattern.py
                        if(self.mouth_service.state == State.FAILED):
                            self.blackboard_tts.result_message = "FAILURE"
                            new_status = py_trees.common.Status.FAILURE
                        else:
                            new_status = py_trees.common.Status.RUNNING

            self.logger.debug("%s.update()[%s]--->[%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status 

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID:
            self.logger.debug(f"Sending goal to {self.server_name} to stop the service")
            # Send request for each sensor service to set themselves up
            self.service_client_camera.send_goal(
                action_goal=ActionType["STOP"].value,
                optional_data="",
                wait="",
            )
            self.logger.debug(f"Goal sent to {self.server_name}")
            self.client_result = None
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
