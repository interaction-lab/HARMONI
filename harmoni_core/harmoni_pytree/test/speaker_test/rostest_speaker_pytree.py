#!/usr/bin/env python3


PKG = "test_harmoni_speaker"
# Common Imports
import unittest, rospy, roslib, sys

# Specific Imports
from actionlib_msgs.msg import GoalStatus
from harmoni_common_msgs.msg import harmoniAction, harmoniFeedback, harmoniResult
from std_msgs.msg import String
from harmoni_common_lib.action_client import HarmoniActionClient
from std_msgs.msg import String
from harmoni_common_lib.constants import ActuatorNameSpace, ActionType, State
from collections import deque
import os, io
import ast
import time
#py_tree
import py_trees
from harmoni_speaker.speaker_service_pytree import SpeakerServicePytree


class TestSpeakerPyTree(unittest.TestCase):

    def setUp(self):
        """
        Set up the client for requesting to harmoni_speaker
        """
        rospy.init_node("test_speaker_pytree", log_level=rospy.INFO)
        self.data = rospy.get_param(
            "test_speaker_input"
        ) 
        self.instance_id = rospy.get_param("instance_id")
        # NOTE currently no feedback, status, or result is received.
        py_trees.logging.level = py_trees.logging.Level.DEBUG
        
        # ex namespace: harmoni_tts
        self.blackboardProva = py_trees.blackboard.Client(name="blackboardProva", namespace=ActuatorNameSpace.tts.name)
        self.blackboardProva.register_key("result_data", access=py_trees.common.Access.WRITE)
        self.blackboardProva.register_key("result_message", access=py_trees.common.Access.WRITE)

        self.blackboardProva.result_message = State.SUCCESS
        self.blackboardProva.result_data = self.data

        additional_parameters = dict([
            (ActuatorNameSpace.speaker.name,False)])   
        rospy.loginfo("--------------------"+str(additional_parameters)) 
        self.speakerPyTree =  SpeakerServicePytree("speakerPyTreeTest")
        self.speakerPyTree.setup(**additional_parameters)

        rospy.loginfo("Testspeaker: Started up. waiting for speaker startup")
        rospy.loginfo("Testspeaker: Started")

   
    
    def test_leaf_pytree_speaker(self):
        rospy.loginfo(f"The input data is {self.data}")
        for unused_i in range(0, 4):
            self.speakerPyTree.tick_once()
            time.sleep(0.5)
            print(self.blackboardProva)
        print("\n")
        return
    

def main():
    import rostest
    rospy.loginfo("test_speaker started")
    rospy.loginfo("Testspeaker: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, "test_speaker_pytree", TestSpeakerPyTree, sys.argv)


if __name__ == "__main__":
    main()
