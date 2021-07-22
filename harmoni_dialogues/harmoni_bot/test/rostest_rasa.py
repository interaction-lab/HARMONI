#!/usr/bin/env python3

# Common Imports
import rospy
import sys
import unittest
from collections import deque

# Specific Imports
import requests
from actionlib_msgs.msg import GoalStatus
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib.constants import DialogueNameSpace, ActionType
from harmoni_common_msgs.msg import harmoniFeedback, harmoniResult

PKG = "test_harmoni_bot"


class TestRasa(unittest.TestCase):

    def setUp(self):
        """
        Set up the client for requesting to harmoni_bot
        """
        rospy.init_node("test_rasa", log_level=rospy.INFO)
        self.text = rospy.get_param("test_rasa_input")
        self.instance_id = rospy.get_param("instance_id")
        self.host = rospy.get_param("rasa/default_param/host")
        self.port = rospy.get_param("rasa/default_param/port")
        self.result = False
        self.name = DialogueNameSpace.bot.name + "_" + self.instance_id
        self.service_client = HarmoniActionClient(self.name)
        self.client_result = deque()
        self.service_client.setup_client(self.name, self.result_cb, self.feedback_cb)
        # NOTE currently no feedback, status, or result is received.
        rospy.Subscriber(
            "/harmoni_bot_default/feedback", harmoniFeedback, self.feedback_cb
        )
        rospy.Subscriber("/harmoni_bot_default/status", GoalStatus, self.status_cb)
        rospy.Subscriber("/harmoni_bot_default/result", harmoniResult, self.result_cb)
        rospy.loginfo("TestRasa: Started up; waiting for Rasa startup")

        # Wait until Rasa server is running before running any test methods
        connected = False
        while not connected:
            try:
                headers = {
                    'Content-Type': 'application/json',
                }
                data = '{ "sender": "test_user", "message": "", "metadata": {} }'
                response = requests.post(
                    f"http://{self.host}:{self.port}/webhooks/myio/webhook", headers=headers, data=data
                )
                connected = response.ok
            except Exception:
                rospy.loginfo("Retrying server connection...")
                rospy.sleep(1)

        rospy.loginfo("TestRasa: Started")

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
        # expected response from the rasa_example bot when the input text is "Hello"
        text = "Hey! How are you?"
        self.service_client.send_goal(
            action_goal=ActionType.REQUEST.value,
            optional_data=self.text,
            wait=True,
        )
        rasa_response = self.service_client.get_result()
        rospy.loginfo("HarmoniResult:" + rasa_response.message)
        assert rasa_response.message == text


def main():
    # TODO convert to a test suite so that setup doesn't have to run over and over.
    import rostest

    rospy.loginfo("test_rasa started")
    rospy.loginfo("TestRasa: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, "test_rasa", TestRasa, sys.argv)


if __name__ == "__main__":
    main()
