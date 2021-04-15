#!/usr/bin/env python3


PKG = "test_harmoni_pattern"
# Common Imports
import unittest, rospy, roslib, sys

# Specific Imports
from actionlib_msgs.msg import GoalStatus
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib.constants import (
    DetectorNameSpace,
    SensorNameSpace,
    ActionType,
    State,
)
from harmoni_common_msgs.msg import harmoniAction, harmoniFeedback, harmoniResult
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
import time
import os, io


class TestSequential(unittest.TestCase):
    """This test of the sequential pattern player uses a simple mic test script.

    If the mic is launched and starts successfully (as verified by the subscriber)
    then the test is passed.
    """

    def __init__(self, *args):
        super(TestSequential, self).__init__(*args)
        self.feedback = State.INIT
        self.result = False
        rospy.init_node("test_sequence", log_level=rospy.INFO)
        self.rate = rospy.Rate(20)

    def setUp(self):
        self.microphone_topic = SensorNameSpace.microphone.value + "default"
        self.output_sub = rospy.Subscriber(
            self.microphone_topic, AudioData, self._mic_listener
        )

        # startup stt node
        self.server = "mic_test_default"
        self.client = HarmoniActionClient(self.server)
        self.client.setup_client(
            self.server, self._result_callback, self._feedback_callback, wait=True
        )
        rospy.loginfo("TestSequential: Turning ON sequential server")
        self.client.send_goal(
            action_goal=ActionType.ON, optional_data="Setup", wait=False
        )
        rospy.loginfo("TestSequential: Started up. waiting for sequence startup")

        # wait for start state
        # while not rospy.is_shutdown() and self.feedback != State.START:
        #     self.rate.sleep()
        time.sleep(3)

    def _feedback_callback(self, data):
        rospy.loginfo(f"TestSequential: Feedback: {data}")
        self.feedback = data["state"]

    def _mic_listener(self, data):
        rospy.loginfo(f"TestSequential: Mic heard: {data}")
        self.result = True

    def _result_callback(self, data):
        rospy.loginfo(f"TestSequential: Result: \n{data}")
        self.result = True


class TestSequential_Valid(TestSequential):
    def test_IO(self):
        rospy.loginfo(
            "TestSequential[TEST]: basic test to ensure pattern can start a service "
        )
        while not rospy.is_shutdown() and not self.result:
            self.rate.sleep()
        assert self.result == True


def main():
    # TODO combine validity tests into test suite so that setup doesn't have to run over and over.
    import rostest

    rospy.loginfo("test_sequence started")
    rospy.loginfo("TestSequential: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, "test_sequence", TestSequential_Valid, sys.argv)


if __name__ == "__main__":
    main()
