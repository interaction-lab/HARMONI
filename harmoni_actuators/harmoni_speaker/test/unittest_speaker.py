#!/usr/bin/env python3


PKG = 'test_harmoni_speaker'
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
from harmoni_speaker.speaker_service import SpeakerService
import json

class TestSpeaker(unittest.TestCase):

    def __init__(self, *args):
        super(TestSpeaker, self).__init__(*args)
        rospy.loginfo("For running unittest you should also run roscore into another terminal")
        rospy.init_node("test_speaker")

    def setUp(self):
        rospack = rospkg.RosPack()
        self.path = rospack.get_path("harmoni_tts") + "/temp_data/tts.wav"
        self.result = False
        rospy.loginfo("TestSpeaker: Started up. waiting for speaker startup")
        self.speaker_service = SpeakerService("test_speaker")
        rospy.loginfo("TestSpeaker: Started")
    
    
    def test_play(self):
        # Send a request to the real API server and store the response.
        response = self.speaker_service.do(self.path)
        # Confirm that the request-response cycle completed successfully.
        rospy.loginfo(response)
        if response["response"]==State.SUCCESS:
            rospy.loginfo("The response succeed")
            self.result = True #set the response to true if the request succeeded
        assert(self.result == True)

def main():
    #TODO convert to a test suite so that setup doesn't have to run over and over.
    import rosunit
    rospy.loginfo("test_speaker started")
    rospy.loginfo("TestSpeaker: sys.argv: %s" % str(sys.argv))
    rosunit.unitrun(PKG, 'test_speaker', TestSpeaker, sys.argv)

if __name__ == "__main__":
    main()
