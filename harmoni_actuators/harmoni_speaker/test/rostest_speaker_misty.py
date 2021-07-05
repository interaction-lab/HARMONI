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
from harmoni_common_lib.constants import ActuatorNameSpace, ActionType
from collections import deque
import os, io
import ast


class TestSpeaker(unittest.TestCase):
    def __init__(self, *args):
        super(TestSpeaker, self).__init__(*args)

    def setUp(self):
        """
        Set up the client for requesting to harmoni_speaker
        """
        rospy.init_node("test_speaker", log_level=rospy.INFO)
        self.data = rospy.get_param(
            "test_speaker_input"
        )  # "$(find harmoni_tts)/temp_data/tts.wav"
        self.instance_id = rospy.get_param("instance_id")
        self.result = False
        self.name = ActuatorNameSpace.speaker.name + "_" + self.instance_id
        self.service_client = HarmoniActionClient(self.name)
        self.client_result = deque()
        self.service_client.setup_client(self.name, self.result_cb, self.feedback_cb)
        # NOTE currently no feedback, status, or result is received.
        rospy.Subscriber(
            "/harmoni_speaker_default/feedback", harmoniFeedback, self.feedback_cb
        )
        rospy.Subscriber("/harmoni_speaker_default/status", GoalStatus, self.status_cb)
        rospy.Subscriber(
            "/harmoni_speaker_default/result", harmoniResult, self.result_cb
        )
        rospy.loginfo("TestSpeaker: Started up. waiting for speaker startup")
        rospy.sleep(
            1
        )  # TODO implement non-magic wait for audio_play node to initialize.
        rospy.loginfo("TestSpeaker: Started")

    def feedback_cb(self, data):
        rospy.loginfo(f"Feedback: {data}")
        self.result = False

    def status_cb(self, data):
        rospy.loginfo(f"Status: {data}")
        self.result = False

    def result_cb(self, data):
        rospy.loginfo(f"Result: {data}")
        self.result = True

    def test_request_response(self):
        rospy.loginfo(f"The input data is {self.data}")
        self.service_client.send_goal(
            action_goal=ActionType.REQUEST.value,
            optional_data=self.data,
            wait=True,
        )
        assert self.result == True


def main():
    import rostest

    rospy.loginfo("test_speaker started")
    rospy.loginfo("TestSpeaker: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, "test_speaker", TestSpeaker, sys.argv)


if __name__ == "__main__":
    main()
