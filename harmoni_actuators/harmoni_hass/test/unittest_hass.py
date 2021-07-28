#!/usr/bin/env python3


PKG = 'test_harmoni_hass'
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
from harmoni_hass.hass_service import HassService
import json


class TestHass(unittest.TestCase):

    def __init__(self, *args):
        super(TestHass, self).__init__(*args)

    def setUp(self):
        self.test_hass_input = { "action":"turn_on", "entity":"googlehome8554", "type":"media_player"}
        self.result = False
        rospy.loginfo("TestHass: Started up. waiting for hass startup")
        self.hass_service = HassService("test_hass")
        rospy.loginfo("TestHass: Started")


    def test_request_response(self):
        # Send a request to the real API server and store the response.
        response = self.hass_service.request(self.test_hass_input)
        # Confirm that the request-response cycle completed successfully.
        rospy.loginfo(response)
        if response["response"]==State.SUCCESS:
            rospy.loginfo("The response succeed")
            self.result = True #set the response to true if the request succeeded
        assert(self.result == True)

def main():
    #TODO convert to a test suite so that setup doesn't have to run over and over.
    import rosunit
    rospy.loginfo("test_hass started")
    rospy.loginfo("TestHass: sys.argv: %s" % str(sys.argv))
    rosunit.unitrun(PKG, 'test_hass', TestHass, sys.argv)

if __name__ == "__main__":
    main()
