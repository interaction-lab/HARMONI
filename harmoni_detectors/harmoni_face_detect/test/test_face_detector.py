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
from harmoni_common_msgs.msg import Object2D, Object2DArray
from sensor_msgs.msg import Image

path = sys.path
using_kinetic = any([True for p in path if ("kinetic" in p)])
if using_kinetic:
    sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
    sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
import cv2

# from std_msgs.msg import String
import time
import os, io


class TestFaceDetector_Common(unittest.TestCase):
    def __init__(self, *args):
        super(TestFaceDetector_Common, self).__init__(*args)
        self.feedback = State.INIT
        self.result = False
        self.detections = []
        self.img_encoding = "rgb8"  # NOTE: There's a weird bug with facenet and ROS Kinetic which will crash if this is set to "bgr8"
        self.image = cv2.imread(rospy.get_param("test_face_detector_input"))
        self.expected_detection = Object2D(
            center_x=485,
            center_y=270,
        )
        rospy.init_node("test_face_detector", log_level=rospy.INFO)
        self.rate = rospy.Rate(1)
        self.cv_bridge = CvBridge()

    def setUp(self):
        # provide mock camera
        self.camera_topic = SensorNameSpace.camera.value + "default"
        self.image_pub = rospy.Publisher(
            self.camera_topic,
            Image,
            queue_size=10,
        )

        self.output_sub = rospy.Subscriber(
            DetectorNameSpace.face_detect.value + "default",
            Object2DArray,
            self._detecting_callback,
        )
        rospy.loginfo(f"Testside-Image source: {SensorNameSpace.camera.value}default")
        rospy.loginfo(
            f"Testside-expected detection: {DetectorNameSpace.face_detect.value}default"
        )

        # startup face_detect node
        self.server = "/harmoni/detecting/face_detect/default"
        self.client = HarmoniActionClient(self.server)
        rospy.loginfo("***********SETTING UP CLIENT")
        self.client.setup_client(
            self.server, self._result_callback, self._feedback_callback, wait=True
        )
        rospy.loginfo("DONE SETTING UP****************")
        rospy.loginfo("TestFaceDetector: Turning ON face_detect server")
        self.client.send_goal(
            action_goal=ActionType.ON, optional_data="Setup", wait=False
        )
        rospy.loginfo("TestFaceDetector: Started up. waiting for face detect startup")

        # wait for start state
        # while not rospy.is_shutdown() and self.feedback != State.START:
        #     self.rate.sleep()

        rospy.loginfo("TestFaceDetector: publishing image")

        self.image_pub.publish(
            self.cv_bridge.cv2_to_imgmsg(self.image, encoding=self.img_encoding)
        )
        # self.image_pub.publish(self.image[:14000])

        rospy.loginfo(
            f"TestFaceDetector: image subscribed to by #{self.output_sub.get_num_connections()} connections."
        )

    def _feedback_callback(self, data):
        rospy.loginfo(f"TestFaceDetector: Feedback: {data}")
        self.feedback = data["state"]

    def _status_callback(self, data):
        rospy.loginfo(f"TestFaceDetector: Status: {data}")
        # self.result = True

    def _result_callback(self, data):
        rospy.loginfo(f"TestFaceDetector: Result: {data}")
        # self.result = True

    def text_received_callback(self, data):
        rospy.loginfo(f"TestFaceDetector: Text back: {data}")
        # self.result = True

    def _detecting_callback(self, data):
        rospy.logdebug(f"TestFaceDetector: Detecting: {data}")
        self.detections = data.data
        self.result = True

    def close_detection(self, detection, expected, precision=15.0):
        """Determine if a detection is "close enough" to pass

        Args:
            detection (Object2D): The detection
            expected (Object2D): The [roughly] expected detection
            precision (float, optional): How close expected needs to be to detection. Defaults to 5.

        Returns:
            bool: True if detection is close to correct
        """
        return (
            abs(detection.center_x - expected.center_x) <= precision
            and abs(detection.center_y - expected.center_y) <= precision
        )


class TestFaceDetector_Valid(TestFaceDetector_Common):
    def test_IO(self):
        rospy.loginfo(
            "TestFaceDetector[TEST]: basic IO test to ensure data "
            + "(example image) is received and responded to. Waiting for detection..."
        )
        while not rospy.is_shutdown() and not self.result:
            # print("waiting for result")
            self.image_pub.publish(
                self.cv_bridge.cv2_to_imgmsg(self.image, encoding=self.img_encoding)
            )
            self.rate.sleep()
        assert self.result == True

    def test_detection(self):
        rospy.loginfo(
            "TestFaceDetector[TEST]: test to verify a face is detected roughly "
            + "Where expected. Waiting for detection..."
        )
        while not rospy.is_shutdown() and len(self.detections) < 1:
            # print("waiting for result")
            self.image_pub.publish(
                self.cv_bridge.cv2_to_imgmsg(self.image, encoding=self.img_encoding)
            )
            self.rate.sleep()
        rospy.loginfo(
            f"Expected face near {self.expected_detection.center_x},{self.expected_detection.center_y}"
        )
        rospy.loginfo(
            f"Detected face at {self.detections[0].center_x},{self.detections[0].center_y}"
        )
        assert self.close_detection(self.detections[0], self.expected_detection)


def main():
    import rostest

    rospy.loginfo("TestFaceDetector: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, "test_face_detector", TestFaceDetector_Valid, sys.argv)


if __name__ == "__main__":
    main()
