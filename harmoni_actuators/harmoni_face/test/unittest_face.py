#!/usr/bin/env python3


PKG = 'test_harmoni_face'
# Common Imports
import unittest, rospy, roslib, sys
#from unittest.mock import Mock, patch
# Specific Imports
from actionlib_msgs.msg import GoalStatus
from harmoni_common_msgs.msg import harmoniAction, harmoniFeedback, harmoniResult
from harmoni_common_lib.constants import State
from std_msgs.msg import String
import os, io
import ast
from harmoni_face.face_service import MouthService, EyesService
import json

class TestFace(unittest.TestCase):

    def __init__(self, *args):
        super(TestFace, self).__init__(*args)
        rospy.loginfo("For running unittest you need to open the browser and connect to rosbridge")

    def setUp(self):
        self.data = "[{'start': 0.075, 'time': 2,'type': 'action', 'id': 'QT/point_front'}, {'start': 0.075,'time': 2, 'type': 'viseme', 'id': 'POSTALVEOLAR'},{'start': 0.006, 'time': 2,  'type': 'action', 'id': 'happy_face'}]"
        self.result = False
        rospy.loginfo("TestFace: Started up. waiting for face startup")
        self.face_mouth_service = MouthService("test_face_mouth", param={"min_duration_viseme":0.05,  "speed_viseme": 10,"timer_interval": 0.01})
        self.face_eyes_service = EyesService("test_face_eyes", param={"gaze_speed":0.05})
        rospy.loginfo("TestFace: Started")
    
    
    def test_play(self):
        # Send a request to the real API server and store the response.
        response_mouth = self.face_mouth_service.do(self.data)
        # Need to test also eyes. TODO: not working yet
        #response_eyes = self.face_eyes_service.do(self.data)
        # Confirm that the request-response cycle completed successfully.
        rospy.loginfo(response)
        if response["response"]==State.SUCCESS:
            rospy.loginfo("The response succeed")
            self.result = True #set the response to true if the request succeeded
        assert(self.result == True)

def main():
    #TODO convert to a test suite so that setup doesn't have to run over and over.
    import rosunit
    rospy.loginfo("test_face started")
    rospy.loginfo("TestFace: sys.argv: %s" % str(sys.argv))
    rosunit.unitrun(PKG, 'test_face', TestFace, sys.argv)

if __name__ == "__main__":
    main()
