#!/usr/bin/env python3


PKG = "test_harmoni_face_detect"
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
from cv_bridge import CvBridge, CvBridgeError
from harmoni_common_lib.constants import SensorNameSpace
from sensor_msgs.msg import Image

sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
import cv2
from std_msgs.msg import String
import time
import os, io


class TestFacenet_Common(unittest.TestCase):
    def __init__(self, *args):
        super(TestFacenet_Common, self).__init__(*args)
        self.feedback = State.INIT
        self.result = False
        self.test_file = rospy.get_param("test_facenet_input")
        self.image = self.jpg_to_data(self.test_file)
        rospy.init_node("test_facenet", log_level=rospy.INFO)
        self.rate = rospy.Rate(20)
        self.cv_bridge = CvBridge()

    def jpg_to_data(self, path):
        with io.open(path, "rb") as f:
            content = f.read()
        return content

    def setUp(self):
        # self.output_sub = rospy.Subscriber(
        #     "/harmoni/detecting/face_detect/default", String, self._detecting_callback
        # )
        # provide mock camera
        self.camera_topic = SensorNameSpace.camera.value + "default/watching"
        self.image_pub = rospy.Publisher(
            self.camera_topic,
            Image,
            queue_size=10,
        )
        # /harmoni/detecting/face_detect/default

        rospy.Subscriber(
            DetectorNameSpace.face_detect.value + "default",
            String,
            self.text_received_callback,
        )
        print(
            "Testside-Image source: ", SensorNameSpace.camera.value + "default/watching"
        )
        print(
            "Testside-expected detection: ",
            DetectorNameSpace.face_detect.value + "default",
        )

        # startup face_detect node
        self.server = "face_detect_default"
        self.client = HarmoniActionClient(self.server)
        print("***********SETTING UP CLIENT")
        self.client.setup_client(
            self.server, self._result_callback, self._feedback_callback, wait=True
        )
        print("DONE SETTING UP****************")
        rospy.loginfo("TestFacenet: Turning ON face_detect server")
        self.client.send_goal(
            action_goal=ActionType.ON, optional_data="Setup", wait=False
        )
        rospy.loginfo("TestFacenet: Started up. waiting for face detect startup")

        # wait for start state
        # while not rospy.is_shutdown() and self.feedback != State.START:
        #     self.rate.sleep()
        time.sleep(5)

        rospy.loginfo("TestFacenet: publishing image")

        self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(self.image))
        # self.image_pub.publish(self.image[:14000])

        rospy.loginfo(
            f"TestFacenet: image subscribed to by #{self.output_sub.get_num_connections()} connections."
        )

    def _feedback_callback(self, data):
        rospy.loginfo(f"TestFacenet: Feedback: {data}")
        self.feedback = data["state"]

    def _status_callback(self, data):
        rospy.loginfo(f"TestFacenet: Status: {data}")
        self.result = True

    def _result_callback(self, data):
        rospy.loginfo(f"TestFacenet: Result: {data}")
        self.result = True

    def text_received_callback(self, data):
        rospy.loginfo(f"TestFacenet: Text back: {data}")
        self.result = True

    def _detecting_callback(self, data):
        rospy.loginfo(f"TestFacenet: Detecting: {data}")
        self.result = True


class TestFacenet_Valid(TestFacenet_Common):
    def test_IO(self):
        print("TEST_IO")
        rospy.loginfo(
            "TestFacenet[TEST]: basic IO test to ensure data "
            + "(example image) is received and responded to. Waiting for transcription..."
        )
        while not rospy.is_shutdown() and not self.result:
            print("waiting for reseult")
            self.rate.sleep()
        assert self.result == True


def main():
    print("MAIN")
    # TODO combine validity tests into test suite so that setup doesn't have to run over and over.
    import rostest

    rospy.loginfo("test_facenet started")
    rospy.loginfo("TestFacenet: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, "test_facenet", TestFacenet_Valid, sys.argv)
    print("DONE")


if __name__ == "__main__":
    main()
