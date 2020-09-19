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

# sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
sys.path.append("/opt/ros/noetic/lib/python2.7/dist-packages")
from cv_bridge import CvBridge, CvBridgeError
from harmoni_common_lib.constants import SensorNameSpace
from sensor_msgs.msg import Image
import cv2


class CameraService(HarmoniServiceManager):
    """
    Camera service
    """

    def __init__(self, name, param):
        super().__init__(name)
        """ Initialization of variables and camera parameters """
        self.input_device_index = param["input_device_index"]
        self.show = param["show"]
        self.video_format = param["video_format"]
        self.service_id = hf.get_child_id(self.name)
        """ Setup the camera """
        self.cv_bridge = CvBridge()
        """ Init the camera publisher"""
        self._video_pub = rospy.Publisher(
            SensorNameSpace.camera.value + self.service_id + "/watching",
            Image,
            queue_size=1,
        )
        """Setup the camera service as server """
        self.setup_camera()
        self.state = State.INIT
        return

    def start(self, rate=""):
        rospy.loginfo("Start the %s service" % self.name)
        if self.state == State.INIT:
            self.state = State.START
            try:
                self.watch()  # Start the camera service at the INIT
            except Exception:
                self.state = State.FAILED
        else:
            self.state = State.START
        return

    def stop(self):
        rospy.loginfo("Stop the %s service" % self.name)
        try:
            self.close_stream()
            self.state = State.SUCCESS
        except Exception:
            self.state = State.FAILED
        return

    def pause(self):
        rospy.loginfo("Pause the %s service" % self.name)
        self.state = State.SUCCESS
        return

    def setup_camera(self):
        """ Setup the camera """
        rospy.loginfo("Setting up the %s" % self.name)
        self.video_cap = cv2.VideoCapture(self.input_device_index)
        self.open_stream()
        return

    def open_stream(self):
        """Opening the stream """
        rospy.loginfo("Opening the video input stream")
        self.width = int(self.video_cap.get(cv2.CAP_PROP_FRAME_WIDTH) + 0.5)
        self.height = int(self.video_cap.get(cv2.CAP_PROP_FRAME_HEIGHT) + 0.5)
        self.size = (self.width, self.height)
        self.fps = self.video_cap.get(cv2.CAP_PROP_FPS)
        rospy.set_param("/" + self.name + "_param/fps/", self.fps)
        return

    def close_stream(self):
        """Closing the stream """
        self.video_cap.release()
        if self.show:
            cv2.destroyAllWindows()
        return

    def watch(self):
        while not rospy.is_shutdown():
            _, frame = self.video_cap.read()
            image = self.cv_bridge.cv2_to_imgmsg(frame, self.video_format)
            self._video_pub.publish(image)
            if self.show:
                cv2.imshow("PcCameraVideo", frame)
            if cv2.waitKey(1) and (0xFF == ord("x")) and self.show:
                break
        return


def main():
    service_name = SensorNameSpace.camera.name
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
        s = CameraService(service, param)
        service_server = HarmoniServiceServer(name=service, service_manager=s)
        if test:
            rospy.loginfo("Testing the %s" % (service))
            s.start()
        else:
            service_server.update_feedback()
            rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
