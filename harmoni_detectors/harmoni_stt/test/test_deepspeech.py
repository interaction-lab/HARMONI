#!/usr/bin/env python3

# Common Imports
import io
import rospy
import sys
import unittest

# Specific Imports
import time
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib.constants import ActionType, DetectorNameSpace, SensorNameSpace, State

from audio_common_msgs.msg import AudioData
from std_msgs.msg import String

PKG = "test_harmoni_stt"


class TestDeepSpeech_Common(unittest.TestCase):

    def wav_to_data(self, path):
        with io.open(path, "rb") as f:
            content = f.read()
        return content

    def setUp(self):
        self.feedback = State.INIT
        self.result = False
        self.test_file = rospy.get_param("test_deepspeech_input")
        self.audio = self.wav_to_data(self.test_file)
        rospy.init_node("test_deepspeech", log_level=rospy.INFO)
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
        rospy.loginfo("TestDeepSpeech: Turning ON stt server")
        self.client.send_goal(
            action_goal=ActionType.ON, optional_data="Setup", wait=False
        )
        rospy.loginfo("TestDeepSpeech: Started up. waiting for DeepSpeech startup")

        time.sleep(5)

        rospy.loginfo("TestDeepSpeech: publishing audio")

        self.audio_pub.publish(self.audio)
        self.audio_pub.publish(self.audio[:14000])

        rospy.loginfo(
            f"TestDeepSpeech: audio subscribed to by #{self.output_sub.get_num_connections()} connections."
        )

    def _feedback_callback(self, data):
        rospy.loginfo(f"TestDeepSpeech: Feedback: {data}")
        self.feedback = data["state"]

    def _status_callback(self, data):
        rospy.loginfo(f"TestDeepSpeech: Status: {data}")
        self.result = True

    def _result_callback(self, data):
        rospy.loginfo(f"TestDeepSpeech: Result: {data}")
        self.result = True

    def text_received_callback(self, data):
        rospy.loginfo(f"TestDeepSpeech: Text back: {data}")
        self.result = True

    def _detecting_callback(self, data):
        rospy.loginfo(f"TestDeepSpeech: Detecting: {data}")
        self.result = True


class TestDeepSpeech_Valid(TestDeepSpeech_Common):
    def test_IO(self):
        rospy.loginfo(
            "TestDeepSpeech[TEST]: basic IO test to ensure data "
            + "('hello' audio) is received and responded to. Waiting for transcription..."
        )
        while not rospy.is_shutdown() and not self.result:
            self.rate.sleep()
        assert self.result


def main():
    # TODO combine validity tests into test suite so that setup doesn't have to run over and over.
    import rostest

    rospy.loginfo("test_deepspeech started")
    rospy.loginfo("TestDeepSpeech: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, "test_deepspeech", TestDeepSpeech_Valid, sys.argv)


if __name__ == "__main__":
    main()
