#!/usr/bin/env python3


PKG = "test_harmoni_microphone"
# Common Imports
import unittest, rospy, roslib, sys

# Specific Imports
from actionlib_msgs.msg import GoalStatus
from harmoni_common_msgs.msg import harmoniAction, harmoniFeedback, harmoniResult
from audio_common_msgs.msg import AudioData
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib.constants import SensorNameSpace, ActionType
from collections import deque
import os, io
import ast


class TestMicrophone(unittest.TestCase):

    def setUp(self):
        """
        Set up the client for requesting to harmoni_microphone
        """
        rospy.init_node("test_microphone", log_level=rospy.INFO)
        # self.data = rospy.get_param(
        #     "test_microphone_input"
        # )  # "$(find harmoni_tts)/temp_data/tts.wav"
        self.instance_id = "default"  # rospy.get_param("instance_id")
        self.result = False
        self.name = SensorNameSpace.microphone.name + "_" + self.instance_id
        self.service_client = HarmoniActionClient(self.name)
        self.client_result = deque()
        self.service_client.setup_client(self.name, self.result_cb, self.feedback_cb)

        # NOTE currently no feedback, status, or result is received.
        rospy.Subscriber(
            "/harmoni_microphone_default/feedback", harmoniFeedback, self.feedback_cb
        )
        rospy.Subscriber(
            "/harmoni_microphone_default/status", GoalStatus, self.status_cb
        )
        rospy.Subscriber(
            "/harmoni_microphone_default/result", harmoniResult, self.result_cb
        )
        rospy.Subscriber(
            SensorNameSpace.microphone.value + self.instance_id, AudioData, self.mic_cb
        )
        rospy.loginfo("TestMicrophone: Started up. waiting for microphone startup")
        rospy.sleep(
            1
        )  # TODO implement non-magic wait for audio_play node to initialize.
        rospy.loginfo("TestMicrophone: Started")

    def mic_cb(self, data):
        if not self.result:
            rospy.loginfo("Mic is publishing")
            self.result = True

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
        rospy.loginfo("Start test by sending the 'on' goal")
        self.service_client.send_goal(
            action_goal=ActionType.ON.value,
            # optional_data=self.data,
            wait=False,
        )
        rospy.sleep(1)
        assert self.result == True, "Mic should be publishing by now"
        # TODO: Fix preempt requests so stop can interrupt start
        rospy.loginfo("Next test by sending the 'off' goal")
        self.service_client.send_goal(
            action_goal=ActionType.OFF.value,
            # optional_data=self.data,
            wait=True,
        )
        rospy.loginfo("Goal sent. Stop publishing please")
        rospy.sleep(1)
        rospy.loginfo("Should be off now, resetting result")
        self.result = False
        rospy.sleep(1)
        assert self.result == False, "Mic should be off by now"

    def test_recording(self):
        # TODO use microphone service recording functionality
        # either through an import ant test or may add optional data
        # which specifies recording
        raise NotImplementedError()



def main():
    import rostest

    rospy.loginfo("test_microphone started")
    rospy.loginfo("TestMicrophone: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, "test_microphone", TestMicrophone, sys.argv)


if __name__ == "__main__":
    main()
