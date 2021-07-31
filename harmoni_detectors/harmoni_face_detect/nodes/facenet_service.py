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
from cv_bridge import CvBridge

path = sys.path
using_kinetic = any([True for p in path if ("kinetic" in p)])
if using_kinetic:
    sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
    sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")

from facenet_pytorch import MTCNN
import cv2


class FacenetFaceDetector(HarmoniServiceManager):
    """Face detector based off of Facenet
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
        for key in param:
            setattr(self, key, param[key])
        self.detector_threshold = detector_threshold
        self.service_id = name
        self._image_source = SensorNameSpace.camera.value + self.subscriber_id
        print("Expected image source: ", self._image_source)
        self._image_sub = (
            None  # assign this when start() called. #TODO test subscription during init
        )
        print(
            "Expected detected destination: ",
            DetectorNameSpace.face_detect.value + self.service_id,
        )
        self._face_pub = rospy.Publisher(
            self.service_id,
            Object2DArray,
            queue_size=1,
        )

        # self._hogFaceDetector = dlib.get_frontal_face_detector()
        self._cv_bridge = CvBridge()
        self.state = State.INIT

    def start(self, rate=""):
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
        self.state = State.START

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
        # FIXME: code crashes without these log statements???
        rospy.loginfo("DETECTING A CALLBACK HERE")
        frame = self._cv_bridge.imgmsg_to_cv2(image, desired_encoding="rgb8")
        rospy.loginfo("Made it past the bridge")
        if frame is not None:
            h, w, _ = frame.shape
            boxes, probs, landmarks = MTCNN().detect(frame, landmarks=True)
            # help(MTCNN)
            faces = []
            for i, (box, probs) in enumerate(zip(boxes, probs)):
                x1 = box[0]
                y1 = box[1]
                x2 = box[2]
                y2 = box[3]
                cx = (box[0] + box[2]) / 2
                cy = (box[1] + box[3]) / 2

                faces.append(
                    Object2D(
                        width=int(w),
                        height=int(h),
                        id=i,
                        center_x=int(cx),
                        center_y=int(cy),
                        topleft_x=int(x1),
                        topleft_y=int(y1),
                        botright_x=int(x2),
                        botright_y=int(y2),
                        confidence=probs,
                    )
                )

            self._face_pub.publish(Object2DArray(faces))


def main():

    service_name = DetectorNameSpace.face_detect.name  # "w2l"
    instance_id = rospy.get_param("instance_id")  # "default"
    service_id = DetectorNameSpace.face_detect.value + instance_id

    try:
        rospy.init_node(service_name, log_level=rospy.INFO)

        params = rospy.get_param(instance_id + "_param/")

        s = FacenetFaceDetector(service_id, params)
        service_server = HarmoniServiceServer(service_id, s)

        service_server.start_sending_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
