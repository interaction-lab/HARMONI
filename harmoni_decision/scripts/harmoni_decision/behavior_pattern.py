#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib import constants

from behavior_controller import BehaviorController

class BehaviorPattern():
    """Abstract class defining common variables/functions for behavior patterns

    Args:
        controller(BehaviorController): Class instance where all data and commands pass through.
    """
    #TOPICS = [(str route, msg_type)] # A list of tuples indicating topics this pattern requires.

    def ___init__(self, controller: BehaviorController):
        self.controller = controller
    
    def start(self):
        raise NotImplementedError
    
    def stop(self):
        raise NotImplementedError
    
    def pause(self):
        raise NotImplementedError
    
