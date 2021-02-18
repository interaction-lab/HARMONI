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

        # self._hogFaceDetector = dlib.get_frontal_face_detector()
        # self._cv_bridge = CvBridge()
        self.state = State.INIT

    def start(self, rate):
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

    # def paint(self, frame, boxes, probability, landmarks):
    #     try:
    #         for box, probability, ld in zip(boxes, probability, landmarks):
    #             cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), (0, 0, 255), 10)
    #     except:
    #         pass
    #     return frame
    #
    # def run(self):
    #     cap = cv2.VideoCapture(0)
    #     while True:
    #         ret, img = cap.read()
    #         try:
    #             boxes, probability, landmarks = self.mtcnn.detect(img, landmarks=True)
    #             self.paint(img, boxes, probability, landmarks)
    #         except:
    #             pass
    #         cv2.imshow('img', img)
    #         if cv2.waitKey(1) & 0xFF == ord('q'):
    #             break
    #     cap.release()
    #     cv2.destroyAllWindows()

    def detect_callback(self, image):
        """Uses image to detect and publish face info.
        Args:
            image(Image): the image we want to run face detection on.
        """
        frame = self._cv_bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
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
                        width=w,
                        height=h,
                        id=i,
                        center_x=cx,
                        center_y=cy,
                        topleft_x=x1,
                        topleft_y=y1,
                        botright_x=x2,
                        botright_y=y2,
                        confidence=probs,
                    )
                )

                # cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), (0, 0, 255), 10)
                # cv2.circle(frame, tuple((int(cx), int(cy))), 5, (0, 0, 255), -1)
                # cv2.putText(frame, str(
                #     probs), (box[2], box[3]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

            self._face_pub.publish(Object2DArray(faces))

            # cv2.imshow('img', frame)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()


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

            return
        service = hf.set_service_server(service_name, test_id)
        s = FacenetFaceDetector(service, param)
        service_server = HarmoniServiceServer(name=service, service_manager=s)
        if test:
            rospy.loginfo("Testing the %s" % (service))
            s.start(test_input)
        else:
            service_server.start_sending_feedback()
            rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
