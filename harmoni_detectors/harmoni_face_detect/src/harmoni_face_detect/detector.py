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

import cv2
from cv_bridge import CvBridge, CvBridgeError
import dlib
import numpy as np


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

    def __init__(self, name, param, detector_threshold=0):
        super().__init__(name)
        self._upsampling = param["up_sampling"]
        self._rate = param["rate_frame"]
        self.subscriber_id = param["subscriber_id"]
        self.update(State.INIT)
        self.detector_threshold = detector_threshold
        self.service_id = hf.get_child_id(self.name)
        self._image_source = (
            SensorNameSpace.camera.value + self.subscriber_id + "watching"
        )  # /harmoni/sensing/watching/harmoni_camera"
        self._image_sub = (
            None  # assign this when start() called. #TODO test subscription during init
        )
        self._face_pub = rospy.Publisher(
            DetectorNameSpace.face_detect.value + self.service_id,
            Object2DArray,
            queue_size=1,
        )

        self._hogFaceDetector = dlib.get_frontal_face_detector()
        self._cv_bridge = CvBridge()
        self.state = State.INIT

    def start(self, rate=None):
        """
        Args:
            rate(int): How often the detector should run per second (Hz).
                Note that this rate should be limited by subscribed camera framerate.
                TODO: actually use this rate. Rate currently matches camera publish rate regardless of this setting
        """
        self._rate = rate
        self._image_sub = rospy.Subscriber(
            self._image_source, Image, self.detect_callback
        )
        print(self._image_source)
        if self._image_sub != None:
            self.update(State.START)
            rospy.loginfo("Face detector started.")
        else:
            self.update(State.FAILED)
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
        frame = self._cv_bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
        if frame is not None:
            h, w, _ = frame.shape

            # preprocess img acquired
            # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # convert bgr to rgb
            faces = []
            dets, probs, idx = self._hogFaceDetector.run(
                frame, self._upsampling, self.detector_threshold
            )
            for i, d in enumerate(dets):
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
    service_name = DetectorNameSpace.face_detect.name
    name = rospy.get_param("/name_" + service_name + "/")
    test = rospy.get_param("/test_" + service_name + "/")
    test_input = rospy.get_param("/test_input_" + service_name + "/")
    test_id = rospy.get_param("/test_id_" + service_name + "/")
    try:
        rospy.init_node(service_name)
        param = rospy.get_param(name + "/" + test_id + "_param/")
        if not hf.check_if_id_exist(service_name, test_id):
            rospy.logerr(
                "ERROR: Remember to add your configuration ID also in the harmoni_core config file"
            )
            return
        service = hf.set_service_server(service_name, test_id)
        s = DlibFaceDetector(service, param)
        service_server = HarmoniServiceServer(name=service, service_manager=s)
        if test:
            rospy.loginfo("Testing the %s" % (service))
            s.start(test_input)
        else:
            service_server.update_feedback()
            rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
