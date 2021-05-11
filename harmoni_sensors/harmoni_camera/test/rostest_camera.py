#!/usr/bin/env python3


PKG = "test_harmoni_camera"
# Common Imports
import unittest, rospy, roslib, sys

# Specific Imports
from actionlib_msgs.msg import GoalStatus
from harmoni_common_msgs.msg import harmoniAction, harmoniFeedback, harmoniResult
from sensor_msgs.msg import Image
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib.constants import SensorNameSpace, ActionType
from collections import deque
import os, io
import ast


class TestCamera(unittest.TestCase):

    def setUp(self):
        """
        Set up the client for requesting to harmoni_camera
        """
        rospy.init_node("test_camera", log_level=rospy.INFO)
        # self.data = rospy.get_param(
        #     "test_camera"
        # )  # "$(find harmoni_tts)/temp_data/tts.wav"
        self.instance_id = "default"  # rospy.get_param("instance_id")
        self.result = False
        self.name = SensorNameSpace.camera.name + "_" + self.instance_id
        self.service_client = HarmoniActionClient(self.name)
        self.client_result = deque()
        self.service_client.setup_client(self.name, self.result_cb, self.feedback_cb)

        # NOTE currently no feedback, status, or result is received.
        rospy.Subscriber("/harmoni_camera/feedback", harmoniFeedback, self.feedback_cb)
        rospy.Subscriber("/harmoni_camera/status", GoalStatus, self.status_cb)
        rospy.Subscriber("/harmoni_camera/result", harmoniResult, self.result_cb)
        rospy.Subscriber(
            SensorNameSpace.camera.value + self.instance_id, Image, self.camera_cb
        )
        rospy.loginfo("TestCamera: Started up. waiting for camera startup")
        rospy.sleep(
            1
        )  # TODO implement non-magic wait for audio_play node to initialize.
        rospy.loginfo("TestCamera: Started")

    def camera_cb(self, data):
        if not self.result:
            rospy.loginfo("Camera is publishing")
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
        rospy.sleep(5)
        assert self.result == True, "Camera should be publishing by now"
        # TODO: Fix preempt requests so stop can interrupt start
        rospy.loginfo("Next test by sending the 'off' goal")
        self.service_client.send_goal(
            action_goal=ActionType.OFF.value,
            # optional_data=self.data,
            wait=True,
        )
        rospy.loginfo("Goal sent. Stop publishing please")
        rospy.sleep(5)
        rospy.loginfo("Should be off now, resetting result")
        self.result = False
        rospy.sleep(5)
        assert self.result == False, "Camera should be off by now"

    def test_recording(self):
        # TODO use camera service recording functionality
        # either through an import ant test or may add optional data
        # which specifies recording
        pass


def main():
    import rostest

    rospy.loginfo("test_camera started")
    rospy.loginfo("TestCamera: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, "test_camera", TestCamera, sys.argv)


if __name__ == "__main__":
    main()
