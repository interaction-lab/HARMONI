#!/usr/bin/env python3

# Common Imports
import rospy, rospkg, roslib

from harmoni_common_lib.action_client import HarmoniActionClient
from actionlib_msgs.msg import GoalStatus
import harmoni_common_lib.helper_functions as hf

# Specific Imports
from harmoni_web.web_service import WebService
from harmoni_common_lib.constants import *
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
    def __init__(self, name = "WebServicePytree"):
        
        self.name = name
        self.service_client_web = None
        self.server_state = None
        self.server_name = None
        self.client_result = None

        self.blackboards = []
        self.blackboard_scene = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name)
        self.blackboard_scene.register_key("image", access=py_trees.common.Access.READ)

        super(WebServicePytree, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self,**additional_parameters):
        """
        for parameter in additional_parameters:
            print(parameter, additional_parameters[parameter])  
            if(parameter =="WebServicePytree_mode"):
                self.mode = additional_parameters[parameter]        
        """
        """
        self.service_client_web = HarmoniActionClient(self.name)
        self.server_name = "web_default"
        self.service_client_web.setup_client(self.server_name,
                                            self._result_callback,
                                            self._feedback_callback)
        self.logger.debug("Behavior %s interface action clients have been set up!" % (self.server_name))
        """
        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self): 
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self):
        """
        new_state = self.service_client_web.get_state()
        print(new_state)
        if new_state == GoalStatus.LOST:
            self.logger.debug(f"Sending goal to {self.server_name}")
            self.service_client_web.send_goal(
                action_goal = ActionType["DO"].value,
                optional_data = self.blackboard_scene.image,
                wait=False,
            )
            self.logger.debug(f"Goal sent to {self.server_name}")
            new_status = py_trees.common.Status.RUNNING
        elif new_state == GoalStatus.PENDING or new_state == GoalStatus.ACTIVE:
            new_status = py_trees.common.Status.RUNNING
        elif new_state == GoalStatus.SUCCEEDED:
            new_status = py_trees.common.Status.SUCCESS
        else:
            new_status = py_trees.common.Status.FAILURE
        """
        new_status = py_trees.common.Status.SUCCESS
        self.logger.debug("%s.update()[%s]--->[%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status

    def terminate(self, new_status):
        """
         and new_state != GoalStatus.SUCCEEDED:
        self.logger.debug(f"Cancelling goal to {self.server_name}")
        self.service_client_web.cancel_goal()
        self.client_result = None
        self.logger.debug(f"Goal cancelled to {self.server_name}")
        self.service_client_web.stop_tracking_goal()
        self.logger.debug(f"Goal tracking stopped to {self.server_name}")
        """
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
    rospy.init_node("web_default" , log_level=rospy.INFO)
