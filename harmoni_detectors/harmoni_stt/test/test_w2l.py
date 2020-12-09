#!/usr/bin/env python3


PKG = 'test_harmoni_stt'
# Common Imports
import unittest, rospy, roslib, sys

# Specific Imports
from actionlib_msgs.msg import GoalStatus
from harmoni_common_msgs.msg import harmoniAction, harmoniFeedback, harmoniResult
from audio_common_msgs.msg import AudioData 
from std_msgs.msg import String
import os, io

class TestW2L(unittest.TestCase):
    def wav_to_data(self, path):
        with io.open(path, "rb") as f:
            content = f.read()
        return content

    def __init__(self, *args):
        super(TestW2L, self).__init__(*args)

    def setUp(self):
        rospy.init_node("test_w2l", log_level=rospy.INFO)
        self.test_file = rospy.get_param("test_w2l_input")
        self.audio = self.wav_to_data(self.test_file)
        self.result = False

        # NOTE currently no feedback, status, or result is received.
        # rospy.Subscriber("/stt_default/feedback", harmoniFeedback, self.feedback_cb)
        # rospy.Subscriber("/stt_default/status", GoalStatus, self.status_cb)
        # rospy.Subscriber("/stt_default/result", harmoniResult, self.result_cb)

        self.output_sub = rospy.Subscriber("/harmoni/detecting/stt/default", String, self.detecting_cb)
        self.audio_pub = rospy.Publisher("/audio/audio", AudioData, queue_size=1)
        rospy.loginfo("TestW2L: Started up. waiting for w2l startup")
        rospy.sleep(5) # NOTE If sleep does not occur here, the published message gets ignored
        self.audio_pub.publish(self.audio)
        rospy.loginfo("TestW2L: publishing audio")
        rospy.loginfo(f"TestW2L: audio subscribed to by {self.output_sub.get_num_connections()}.")
    

    def feedback_cb(self, data):
        print(f"Feedback: {data}")
        self.result = True

    def status_cb(self, data):
        print(f"Status: {data}")
        self.result = True
    
    def result_cb(self, data):
        print(f"Result: {data}")
        self.result = True
    
    def detecting_cb(self, data):
        print(f"Detecting: {data}")
        self.result = True

    def test_IO(self):
        rospy.sleep(1)
        rospy.loginfo("TestW2L: basic IO test to ensure data ('hello' audio) is received and responded to.")
        rospy.sleep(5)
        assert(self.result == True)

def main():
    #TODO convert to a test suite so that setup doesn't have to run over and over.
    import rostest
    rospy.loginfo("test_w2l started")
    rospy.loginfo("TestW2L: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, 'test_w2l', TestW2L, sys.argv)


if __name__ == "__main__":
    main()
