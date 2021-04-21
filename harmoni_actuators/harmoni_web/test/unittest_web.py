#!/usr/bin/env python3


PKG = 'test_harmoni_web'
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
from harmoni_web.web_service import WebService
import json

class TestWeb(unittest.TestCase):

    def __init__(self, *args):
        super(TestWeb, self).__init__(*args)
        rospy.loginfo("For running unittest you need to open the browser and connect to rosbridge")

    def setUp(self):
        self.data = "{'component_id':'test_container', 'set_content': ''}"
        self.result = False
        rospy.loginfo("TestWeb: Started up. waiting for web startup")
        self.web_service = WebService("test_web")
        rospy.loginfo("TestWeb: Started")
    
    
    def test_do(self):
        # Send a request to the real API server and store the response.
        response = self.web_service.do(self.data)
        # Need to test also eyes. TODO: not working yet
        #response_eyes = self.web_eyes_service.do(self.data)
        # Confirm that the request-response cycle completed successfully.
        rospy.loginfo(response)
        if response["response"]==State.SUCCESS:
            rospy.loginfo("The response succeed")
            self.result = True #set the response to true if the request succeeded
        assert(self.result == True)

    def test_request(self):
        # Send a request to the real API server and store the response.
        response = self.web_service.request(self.data)
        # Need to test also eyes. TODO: not working yet
        #response_eyes = self.web_eyes_service.do(self.data)
        # Confirm that the request-response cycle completed successfully.
        rospy.loginfo(response)
        if response["response"]==State.SUCCESS:
            rospy.loginfo("The response succeed")
            self.result = True #set the response to true if the request succeeded
        assert(self.result == True)

def main():
    #TODO convert to a test suite so that setup doesn't have to run over and over.
    import rosunit
    rospy.loginfo("test_web started")
    rospy.loginfo("TestWeb: sys.argv: %s" % str(sys.argv))
    rosunit.unitrun(PKG, 'test_web', TestWeb, sys.argv)

if __name__ == "__main__":
    main()
