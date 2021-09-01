#!/usr/bin/env python3


PKG = "test_harmoni_gesture"
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
from harmoni_pytree.gesture_service_pytree import GestureServicePytree


class TestGesturePyTree(unittest.TestCase):

    def setUp(self):
        """
        Set up the client for requesting to harmoni_gesture
        """
        rospy.init_node("test_gesture_pytree", log_level=rospy.INFO)
        self.data = rospy.get_param(
            "test_gesture_input"
        ) 
        self.instance_id = rospy.get_param("instance_id")
        # NOTE currently no feedback, status, or result is received.
        py_trees.logging.level = py_trees.logging.Level.DEBUG
        
        self.blackboardProva = py_trees.blackboard.Client(name="blackboardProva", namespace=ActuatorNameSpace.gesture.name)
        self.blackboardProva.register_key("result_data", access=py_trees.common.Access.WRITE)
        self.blackboardProva.register_key("result_message", access=py_trees.common.Access.WRITE)

        self.blackboardProva.result_message = State.SUCCESS
        self.blackboardProva.result_data = self.data

        additional_parameters = dict([
            (ActuatorNameSpace.gesture.name,True)])   
        rospy.loginfo("--------------------"+str(additional_parameters)) 
        self.gesturePyTree =  GestureServicePytree("gesturePyTreeTest")
        self.gesturePyTree.setup(**additional_parameters)

        rospy.loginfo("TestGesture: Started up. waiting for gesture startup")
        rospy.loginfo("TestGesture: Started")

   
    
    def test_leaf_pytree_gesture(self):
        rospy.loginfo(f"The input data is {self.data}")
        for unused_i in range(0, 4):
            self.gesturePyTree.tick_once()
            time.sleep(0.5)
            print(self.blackboardProva)
        print("\n")
        return
    

def main():
    import rostest
    rospy.loginfo("test_gesture started")
    rospy.loginfo("TestGesture: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, "test_gesture_pytree", TestGesture, sys.argv)


if __name__ == "__main__":
    main()
