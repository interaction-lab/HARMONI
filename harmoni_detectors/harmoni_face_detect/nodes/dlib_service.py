#! /usr/bin/env python3


# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

# Specific Imports
from harmoni_common_lib.constants import DetectorNameSpace, SensorNameSpace
from harmoni_common_msgs.msg import Object2D, Object2DArray
from sensor_msgs.msg import Image

import sys

path = sys.path
using_kinetic = any([True for p in path if ("kinetic" in p)])
if using_kinetic:
    sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
    sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
import cv2
from cv_bridge import CvBridge, CvBridgeError
import dlib


class DlibFaceDetector(HarmoniServiceManager):
    """Face detector based off of Dlib

    Args:
        detector_threshold(float): Confidence threshold for faces. Positive values
            will return fewer detections, and negative values more detections.
            This value can be changed at any time with no major side-effects.
    """

    # The input image can be upsampled for the detector to see more faces.
    # This is usually not necessary.
    # UPSAMPLING = 0
    # DEFAULT_RATE = 10 # Hz

    def __init__(self, name, params, detector_threshold=0):
        super().__init__(name)
        for key in params:
            setattr(self, key, params[key])
        rospy.loginfo("Setting ros params as class attributes")
        self.state = State.INIT
        self.detector_threshold = detector_threshold
        self.service_id = name
        camera_topic = SensorNameSpace.camera.value + self.subscriber_id
        self._image_source = camera_topic
        self._image_sub = None  # assign this when start() called.
        if not hf.topic_active(camera_topic, Image):
            rospy.logwarn(
                f"Unable to find topic {camera_topic} with correct type. Is it publishing yet?"
            )
        self._face_pub = rospy.Publisher(
            self.service_id,
            Object2DArray,
            queue_size=1,
        )

        self._hogFaceDetector = dlib.get_frontal_face_detector()
        self._cv_bridge = CvBridge()
        rospy.get_published_topics()

    def start(self, rate=None):
        """
        Args:
            rate(int): How often the detector should run per second (Hz).
                Note that this rate should be limited by subscribed camera framerate.
                TODO: actually use this rate. Rate currently matches camera publish rate regardless of this setting
        """
        self.rate_frame = rate
        self._image_sub = rospy.Subscriber(
            self._image_source, Image, self.detect_callback
        )
        rospy.logdebug(f"Image source: {self._image_source}")
        if self._image_sub != None:
            self.state = State.START
            rospy.loginfo("Face detector started.")
        else:
            self.state = State.FAILED
            rospy.logerr("Face detector failed to start.")
        return

    def stop(self):
        rospy.loginfo("Face detector stopped.")
        self.state = State.SUCCESS
        try:
            self._image_sub.unregister()
        except rospy.ROSInternalException:
            pass

    def pause(self):
        self.stop()

    def detect_callback(self, image):
        """Uses image to detect and publish face info.

        Args:
            image(Image): the image we want to run face detection on.
        """
        frame = self._cv_bridge.imgmsg_to_cv2(image, desired_encoding="rgb8")
        if frame is not None:
            h, w, _ = frame.shape
            faces = []
            dets, probs, idx = self._hogFaceDetector.run(
                frame, self.up_sampling, self.detector_threshold
            )
            for i, d in enumerate(dets):
                rospy.logdebug(f"Detections: {d}")
                center = d.center()
                x1 = d.left()
                y1 = d.top()
                x2 = d.right()
                y2 = d.bottom()

                faces.append(
                    Object2D(
                        width=w,
                        height=h,
                        id=idx[i],
                        center_x=center.x,
                        center_y=center.y,
                        topleft_x=x1,
                        topleft_y=y1,
                        botright_x=x2,
                        botright_y=y2,
                        confidence=probs[i],
                    )
                )
            self._face_pub.publish(Object2DArray(faces))


def main():
    # default doesn't really matter as the launch file overrides it anyway, so string literals are OK
    service_name = DetectorNameSpace.face_detect.name  # "face_detect"
    instance_id = rospy.get_param("instance_id")
    service_id = DetectorNameSpace.face_detect.value + instance_id

    try:
        rospy.init_node(service_name, log_level=rospy.INFO)

        rospy.logdebug(f"Node namespace: {rospy.get_namespace()}")
        params = rospy.get_param(instance_id + "_param/")

        s = DlibFaceDetector(service_id, params)

        service_server = HarmoniServiceServer(service_id, s)

        service_server.start_sending_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
