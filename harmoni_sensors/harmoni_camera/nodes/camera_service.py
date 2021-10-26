#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

# Other Imports
import sys

path = sys.path
using_kinetic = any([True for p in path if ("kinetic" in p)])
if using_kinetic:
    sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
    sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")

from cv_bridge import CvBridge, CvBridgeError
from harmoni_common_lib.constants import SensorNameSpace
from sensor_msgs.msg import Image
import cv2

class CameraService(HarmoniServiceManager):
    """Reads from a camera and publishes image data.

    As a sensor service, the camera is responsible for reading the image data
    from a physical camera and publishing it so that it can be recorded or
    used by a detector.

    The camera has many parameters which are set in the configuration.yaml

    The public functions exposed by the camera include start(), stop(), and pause()
    """

    def __init__(self, name, param):

        """ Initialization of variables and camera parameters """
        super().__init__(name)
        self.input_device_index = param["input_device_index"]
        self.show = param["show"]
        self.video_format = param["video_format"]
        self.file_path = param["test_outdir"]

        self.service_id = hf.get_child_id(self.name)

        """ Setup the camera """
        self.cv_bridge = CvBridge()
        self.setup_camera()

        """ Init the camera publisher"""
        self.camera_topic = SensorNameSpace.camera.value + self.service_id
        self._video_pub = rospy.Publisher(
            self.camera_topic,
            Image,
            queue_size=1,
        )

        self.state = State.INIT
        return

    def start(self):
        """Start the camera stream and publish images"""
        rospy.loginfo("Start the %s service" % self.name)
        if self.state == State.INIT:
            self.state = State.START
            self._read_stream_and_publish()  # Start the camera service at the INIT
            self.state = State.FAILED
        else:
            rospy.loginfo("Trying to start stream when already started")
            self.state = State.START
        return

    def stop(self):
        """Stop the service and close the stream"""
        rospy.loginfo("Stop the %s service" % self.name)
        try:
            self._close_stream()
            self.state = State.SUCCESS
        except Exception:
            self.state = State.FAILED
        return

    def pause(self):
        """Set the service to success to stop publishing"""
        rospy.loginfo("Pause the %s service" % self.name)
        self.state = State.SUCCESS
        return

    def setup_camera(self):
        """ Setup the camera """
        rospy.loginfo("Setting up the %s" % self.name)
        self.video_cap = cv2.VideoCapture(self.input_device_index)
        self._open_stream()
        return

    def _open_stream(self):
        """Opening the stream """
        rospy.loginfo("Opening the video input stream")
        self.width = int(self.video_cap.get(cv2.CAP_PROP_FRAME_WIDTH) + 0.5)
        self.height = int(self.video_cap.get(cv2.CAP_PROP_FRAME_HEIGHT) + 0.5)
        self.size = (self.width, self.height)
        self.fps = self.video_cap.get(cv2.CAP_PROP_FPS)
        rospy.set_param("/" + self.name + "_param/fps/", self.fps)
        return

    def _close_stream(self):
        """Closing the stream """
        self.video_cap.release()
        if self.show:
            cv2.destroyAllWindows()
        return

    def _read_stream_and_publish(self):
        """Continously publish image data from the camera

        While state is START publish images
        """
        while not rospy.is_shutdown():
            _, frame = self.video_cap.read()
            if (frame is None):
                raise RuntimeError("No camera frame available. Is the configured device accessible?")
            image = self.cv_bridge.cv2_to_imgmsg(frame, self.video_format)
            self._video_pub.publish(image)
            if self.show:
                cv2.imwrite(self.file_path, frame)
                # TODO: Allow showing images in docker
                # cv2.imshow("PcCameraVideo", frame)
                # if cv2.waitKey(1) and (0xFF == ord("x")) and self.show:
                #     break
        return


def main():
    """Set names, collect params, and give service to server"""

    service_name = SensorNameSpace.camera.name  # "camera"
    instance_id = rospy.get_param("instance_id")  # "default"
    service_id = f"{service_name}_{instance_id}"

    try:
        rospy.init_node(service_name, log_level=rospy.INFO)

        # camera/default_param/[all your params]
        params = rospy.get_param(service_name + "/" + instance_id + "_param/")

        s = CameraService(service_id, params)

        service_server = HarmoniServiceServer(service_id, s)

        print(service_name)
        print("**********************************************************************************************")
        print(service_id)

        #TODO: comment it out and create a test for ImageAI
        s.start()

        service_server.start_sending_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
