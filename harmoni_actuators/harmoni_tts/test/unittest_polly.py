#!/usr/bin/env python3


PKG = 'test_harmoni_tts'
# Common Imports
import unittest, rospy, roslib, sys
#from unittest.mock import Mock, patch
# Specific Imports
from actionlib_msgs.msg import GoalStatus
from harmoni_common_lib.constants import State
from harmoni_common_msgs.msg import harmoniAction, harmoniFeedback, harmoniResult
from std_msgs.msg import String
import os, io
import ast
from harmoni_tts.aws_tts_service import AWSTtsService
import json

class TestPolly(unittest.TestCase):

    def __init__(self, *args):
        super(TestPolly, self).__init__(*args)

    def setUp(self):
        self.text = "Hello"
        self.result = False
        rospy.loginfo("TestPolly: Started up. waiting for polly startup")
        self.aws_service = AWSTtsService("test_aws_polly", param={"region_name":"us-west-2",  "language": "en-US","outdir": "/root/harmoni_catkin_ws/src/HARMONI/harmoni_actuators/harmoni_tts/temp_data", "wav_header_length": 24, "voice":"Ivy"})
        rospy.loginfo("TestPolly: Started")
    
    
    def test_request_response(self):
        # Send a request to the real API server and store the response.
        response = self.aws_service.request(self.text)
        # Confirm that the request-response cycle completed successfully.
        rospy.loginfo(response)
        if response["response"]==State.SUCCESS:
            rospy.loginfo("The response succeed")
            self.result = True #set the response to true if the request succeeded
        assert(self.result == True)

def main():
    #TODO convert to a test suite so that setup doesn't have to run over and over.
    import rosunit
    rospy.loginfo("test_polly started")
    rospy.loginfo("TestPolly: sys.argv: %s" % str(sys.argv))
    rosunit.unitrun(PKG, 'test_polly', TestPolly, sys.argv)

if __name__ == "__main__":
    main()
