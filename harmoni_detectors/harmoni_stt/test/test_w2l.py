#!/usr/bin/env python3


PKG = "test_harmoni_stt"
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


class TestW2L_Common(unittest.TestCase):
    def __init__(self, *args):
        super(TestW2L_Common, self).__init__(*args)
        

    def wav_to_data(self, path):
        with io.open(path, "rb") as f:
            content = f.read()
        return content

    def setUp(self):
        self.feedback = State.INIT
        self.result = False
        self.test_file = rospy.get_param("test_w2l_input")
        self.audio = self.wav_to_data(self.test_file)
        rospy.init_node("test_w2l", log_level=rospy.INFO)
        self.rate = rospy.Rate(20)
        self.output_sub = rospy.Subscriber(
            "/harmoni/detecting/stt/default", String, self._detecting_callback
        )
        # provide mock microphone
        self.audio_pub = rospy.Publisher(
            SensorNameSpace.microphone.value
            + rospy.get_param("stt/default_param/subscriber_id"),
            AudioData,
            queue_size=10,
        )
        rospy.Subscriber(
            DetectorNameSpace.stt.value + "stt_default",
            String,
            self.text_received_callback,
        )

        # startup stt node
        self.server = "stt_default"
        self.client = HarmoniActionClient(self.server)
        self.client.setup_client(
            self.server, self._result_callback, self._feedback_callback, wait=True
        )
        rospy.loginfo("TestW2L: Turning ON stt server")
        self.client.send_goal(
            action_goal=ActionType.ON, optional_data="Setup", wait=False
        )
        rospy.loginfo("TestW2L: Started up. waiting for w2l startup")

        # wait for start state
        # while not rospy.is_shutdown() and self.feedback != State.START:
        #     self.rate.sleep()
        time.sleep(5)

        rospy.loginfo("TestW2L: publishing audio")

        self.audio_pub.publish(self.audio)
        self.audio_pub.publish(self.audio[:14000])

        rospy.loginfo(
            f"TestW2L: audio subscribed to by #{self.output_sub.get_num_connections()} connections."
        )

    def _feedback_callback(self, data):
        rospy.loginfo(f"TestW2L: Feedback: {data}")
        self.feedback = data["state"]

    def _status_callback(self, data):
        rospy.loginfo(f"TestW2L: Status: {data}")
        self.result = True

    def _result_callback(self, data):
        rospy.loginfo(f"TestW2L: Result: {data}")
        self.result = True

    def text_received_callback(self, data):
        rospy.loginfo(f"TestW2L: Text back: {data}")
        self.result = True

    def _detecting_callback(self, data):
        rospy.loginfo(f"TestW2L: Detecting: {data}")
        self.result = True


class TestW2L_Valid(TestW2L_Common):
    def test_IO(self):
        rospy.loginfo(
            "TestW2L[TEST]: basic IO test to ensure data "
            + "('hello' audio) is received and responded to. Waiting for transcription..."
        )
        while not rospy.is_shutdown() and not self.result:
            self.rate.sleep()
        assert self.result == True


def main():
    # TODO combine validity tests into test suite so that setup doesn't have to run over and over.
    import rostest

    rospy.loginfo("test_w2l started")
    rospy.loginfo("TestW2L: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, "test_w2l", TestW2L_Valid, sys.argv)


if __name__ == "__main__":
    main()
