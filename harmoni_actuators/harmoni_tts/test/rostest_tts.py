#!/usr/bin/env python3

# Common Imports
import rospy
import sys
import unittest
from collections import deque

# Specific Imports
from actionlib_msgs.msg import GoalStatus
from harmoni_common_msgs.msg import harmoniFeedback, harmoniResult

from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib.constants import ActuatorNameSpace, ActionType

PKG = "test_harmoni_tts"


class TestTts(unittest.TestCase):

    def setUp(self):
        """
        Set up the client for requesting to harmoni_tts
        """
        rospy.init_node("test_tts", log_level=rospy.INFO)
        self.text = rospy.get_param("test_tts_input")
        self.instance_id = rospy.get_param("instance_id")
        self.result = False
        self.name = ActuatorNameSpace.tts.name + "_" + self.instance_id
        self.service_client = HarmoniActionClient(self.name)
        self.client_result = deque()
        self.service_client.setup_client(self.name, self.result_cb, self.feedback_cb)
        # NOTE currently no feedback, status, or result is received.
        rospy.Subscriber(
            "/harmoni_tts_default/feedback", harmoniFeedback, self.feedback_cb
        )
        rospy.Subscriber("/harmoni_tts_default/status", GoalStatus, self.status_cb)
        rospy.Subscriber("/harmoni_tts_default/result", harmoniResult, self.result_cb)
        rospy.loginfo("TestTTS: Started")

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
        rospy.loginfo(f"The input text is {self.text}")
        self.service_client.send_goal(
            action_goal=ActionType.REQUEST.value,
            optional_data=self.text,
            wait=True,
        )
        assert self.result


def main():
    # TODO convert to a test suite so that setup doesn't have to run over and over.
    import rostest

    rospy.loginfo("test_tts started")
    rospy.loginfo("TestTTS: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, "test_tts", TestTts, sys.argv)


if __name__ == "__main__":
    main()
