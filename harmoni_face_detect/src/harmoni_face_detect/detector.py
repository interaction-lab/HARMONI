#! /usr/bin/env python3

import rospy
import roslib
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
from cv_bridge import CvBridge, CvBridgeError
import dlib
import numpy as np

from harmoni_common_lib.constants import State, RouterDetector, HelperFunctions, RouterSensor
from harmoni_common_lib.child import HarwareReadingServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
from harmoni_common_msgs.msg import Object2D, Object2DArray
from sensor_msgs.msg import Image

class DlibFaceDetector(HarmoniServiceManager):
    """Face detector based off of Dlib

    Args:
        detector_threshold(int): Confidence threshold for faces. Positive values
            will return fewer detections, and negative values more detections.
            This value can be changed at any time with no major side-effects.
    """
    # The input image can be upsampled for the detector to see more faces. 
    # This is usually not necessary.
    #UPSAMPLING = 0 
    #DEFAULT_RATE = 10 # Hz

    def __init__(self, name, param, detector_threshold=0):
        self.name = name
        self._upsampling = param["up_sampling"]
        self._rate = param["rate_frame"]
        self.subscriber_id = param["subscriber_id"]
        self.update(State.INIT)
        self.detector_threshold = detector_threshold
        self.service_id = HelperFunctions.get_child_id(self.name)
        self._image_source = RouterSensor.camera.value + self.subscriber_id + "watching"#/harmoni/sensing/watching/pc_camera"
        self._image_sub = None #assign this when start() called. #TODO test subscription during init
        self._face_pub = rospy.Publisher(RouterDetector.face_detect.value + self.service_id, Object2D, queue_size=1)
        
        self._hogFaceDetector = dlib.get_frontal_face_detector()
        self._cv_bridge = CvBridge()   

    def test(self):
        success = True
        return success
    
    def start(self,rate):
        """
        Args:
            rate(int): How often the detector should run per second (Hz).
                Note that this rate should be limited by subscribed camera framerate.
                TODO: actually use this rate. Rate currently matches camera publish rate regardless of this setting
        """
        super().start(rate)
        self._rate = rate
        self._image_sub = rospy.Subscriber(self._image_source, Image, self.detect_callback)
        self.update(State.START)

    def stop(self):
        rospy.loginfo("Face detector stopped.")
        self.update(State.SUCCESS)
        try:
            self._image_sub.unregister()
        except rospy.ROSInternalException:
            pass
    
    def pause(self):
        self.stop()

    def detect_callback(self,image):
        """Uses image to detect and publish face info.

        Args:
            image(Image): the image we want to run face detection on.
        """
        frame = self.cv_bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
        
        if frame is not None:
            h, w, _ = frame.shape
            
            # preprocess img acquired
            #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # convert bgr to rgb
            faces = []
            dets, probs, idx = self._hogFaceDetector.run(frame, self._upsampling, self.detector_threshold)
            for i, d in enumerate(dets):
                center = d.center()
                x1 = d.left()
                y1 = d.top()
                x2 = d.right()
                y2 = d.bottom()
                
                faces.append(Object2D(width=w, height=h,id=idx[i],center_x=center[0],center_y=center[1],topleft_x=x1,
                         topleft_y=y1,botright_x=x2,botright_y=y2,confidence=probs[i]))
            self._face_pub.publish(Object2DArray(faces))
                

def main():
    test = rospy.get_param("/test/")
    input_test = rospy.get_param("/input_test/")
    id_test = rospy.get_param("/id_test/")
    try:
        service_name = RouterDetector.face_detect.name
        rospy.init_node(service_name + "_node")
        list_service_names = HelperFunctions.get_child_list(service_name)
        service_server_list = []
        last_event = ""  # TODO
        for service in list_service_names:
            print("The service is: "+service)
            service_id = HelperFunctions.get_child_id(service)
            param = rospy.get_param("/harmoni_face_detect/"+service_id+"_param/") #TODO: FIX IT WITH ~
            s = DlibFaceDetector(service, param)
            service_server_list.append(HarwareReadingServer(name=service, service_manager=s))
            if test and (service_id == id_test):
                rospy.loginfo("Testing the %s" %(service))
                s.start(input_test)
        if not test:
            for server in service_server_list:
                server.update_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()