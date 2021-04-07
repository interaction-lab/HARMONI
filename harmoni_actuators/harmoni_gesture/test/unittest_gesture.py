#!/usr/bin/env python3


PKG = 'test_harmoni_gesture'
# Common Imports
import unittest, rospy, rospkg, roslib, sys
#from unittest.mock import Mock, patch
# Specific Imports
from actionlib_msgs.msg import GoalStatus
from harmoni_common_msgs.msg import harmoniAction, harmoniFeedback, harmoniResult
from harmoni_common_lib.constants import State
from std_msgs.msg import String
import os, io
import ast
from harmoni_gesture.gesture_service import GestureService
import json

class TestGesture(unittest.TestCase):

    def __init__(self, *args):
        super(TestGesture, self).__init__(*args)
        rospy.loginfo("For running unittest you should also run roscore into another terminal")
        rospy.init_node("test_gesture")

    def setUp(self):
        self.test_gesture_input = "{'gesture':'QT/bye', 'timing': 0.5}"
        self.result = False
        rospy.loginfo("TestGesture: Started up. waiting for gesture startup")
        rospack = rospkg.RosPack()
        self.gesture_service = GestureService("test_gesture", 
            param={"path": rospack.get_path("harmoni_gesture") + "/data",  
                "rate": 10,
                "robot_joint_topic": "qt_robot/joints/state", 
                "robot_joint_radians_topic": "qt_robot/joints/state_rad", 
                "robot_gesture_topic":"qt_robot/gesture/play",
                "time_interval":0.01})
        rospy.loginfo("TestGesture: Started")
    
    
    def test_move(self):
        # Send a request to the real API server and store the response.
        response = self.gesture_service.do(self.test_gesture_input)
        # Confirm that the request-response cycle completed successfully.
        rospy.loginfo(response)
        if response["response"]==State.SUCCESS:
            rospy.loginfo("The response succeed")
            self.result = True #set the response to true if the request succeeded
        assert(self.result == True)

def main():
    #TODO convert to a test suite so that setup doesn't have to run over and over.
    import rosunit
    rospy.loginfo("test_gesture started")
    rospy.loginfo("TestGesture: sys.argv: %s" % str(sys.argv))
    rosunit.unitrun(PKG, 'test_gesture', TestGesture, sys.argv)

if __name__ == "__main__":
    main()
