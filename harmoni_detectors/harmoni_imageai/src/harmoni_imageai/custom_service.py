#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

# Specific Imports
from harmoni_common_lib.constants import State, DetectorNameSpace, SensorNameSpace
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from imageai.Detection.Custom import CustomObjectDetection
from std_msgs.msg import String
import numpy as np
import os
import io
import cv2
from six.moves import queue

class ImageAICustomService(HarmoniServiceManager):
    """
    ImageAICustom service
    """

    def __init__(self, name, param):
        super().__init__(name)
        """ Initialization of variables and imageai parameters """
        self.subscriber_id = param["subscriber_id"]
        self.model_name = param["model_name"]
        self.json_name = param["json_name"]
        self.frame_per_second = param["frame_per_second"]
        self.frame_detection_interval = param["frame_detection_interval"]
        self.output_file_name = param["output_file_name"]
        self.minimum_percentage_probability = param["minimum_percentage_probability"]
        self.return_detected_frame = param["return_detected_frame"]
    
        self.model_path = "/root/harmoni_catkin_ws/src/HARMONI/harmoni_detectors/harmoni_imageai/src/"
        self.service_id = hf.get_child_id(self.name)
        self.result_msg = ""
        self.state = None

        self.detector = CustomObjectDetection()
        self.detector.setModelTypeAsYOLOv3()
        self.detector.setJsonPath("/root/harmoni_catkin_ws/src/HARMONI/harmoni_detectors/harmoni_imageai/src/detection_config_card.json") 
        self.detector.setModelPath(os.path.join(self.model_path, self.model_name))
        self.detector.loadModel()
        self.capture_frame = False
        self.custom_objects_probability = {
                                "carta" : 0,
                                "no" : 0,
                                "nocapito" : 0,
                                "ombrello" : 0,
                                "plastica" : 0,
                                "si" : 0,
                                "stop" : 0,
                                "vetro" : 0
                            }

        self.cv_bridge = CvBridge()
        self._buff = queue.Queue()
        self.closed = False

        self.data = b""

        """Setup publishers and subscribers"""
        rospy.Subscriber(
            SensorNameSpace.camera.value + self.subscriber_id,
            Image,
            self.callback,
        )

        self.channel_qt = rospy.Subscriber(
            "/camera/color/image_raw",
            Image,
            self.callback,
        )

        self.text_pub = rospy.Publisher(
            DetectorNameSpace.imageai_custom_yolo.value + self.service_id, String, queue_size=10
        )

        rospy.Subscriber(
            DetectorNameSpace.imageai_custom_yolo.value + self.service_id,
            String,
            self.imageai_callback,
        )

        """Setup the imageai service as server """
        self.state = State.INIT
        return

    def pause_back(self, data):
        rospy.loginfo(f"pausing for data: {len(data.data)}")
        self.pause()
        rospy.sleep(int(len(data.data) / 30000))  # TODO calibrate this guess
        self.state = State.START
        return

    def callback(self, data):
        """ Callback function subscribing to the camera topic"""
        if self.state == State.REQUEST:
            if self.capture_frame:
                self._buff.put(data)
                self.capture_frame = False
    
    
    def imageai_callback(self, data):
        """ Callback function subscribing to the camera topic"""
        self.response_received = True


    def request(self, data):
        rospy.loginfo("Start the %s request" % self.name)
        self.state = State.REQUEST
        self.capture_frame = True
        try:
            for element in self.custom_objects_probability:
                self.custom_objects_probability[element] = 0
            
            for i in range(1, 6):
                data_tmp = self.cv_bridge.imgmsg_to_cv2(self._buff.get(), desired_encoding='passthrough')
                self.detections = self.detector.detectObjectsFromImage(input_type="array", 
                                                                    output_type="array",
                                                                    input_image=data_tmp,
                                                                    minimum_percentage_probability=self.minimum_percentage_probability,
                                                                    extract_detected_objects=True)
                self.capture_frame = True
                print("RISULTATO ANAISI FOTO NUMERO: ", i)
                #self.result_msg = str(self.detections[1])
                if len(self.detections[1]) != 0:
                    for eachObject in self.detections[1]:
                        self.custom_objects_probability[eachObject["name"]] = eachObject["percentage_probability"]
                        print(eachObject["name"] , " : " , eachObject["percentage_probability"], " : ", eachObject["box_points"] )
                        print("--------------------------------")

                    self.result_msg = max(self.custom_objects_probability, key=self.custom_objects_probability.get)
                else:
                    print("YoloCustom detection --> null")
                    self.result_msg = "null"
            
                if self.result_msg != "null":
                    break
            
            print("The winner is: ")
            if self.result_msg == "null":
                print("No one, the child didn't show anything =(")
            else:
                print(self.result_msg)
            self.response_received = True
            self.state = State.SUCCESS

        except rospy.ServiceException:
            self.state = State.FAILED
            self.response_received = True
            rospy.loginfo("Service call failed")
            self.result_msg = ""
        finally: 
            self._buff.queue.clear()
        return {"response": self.state, "message": self.result_msg}

    def start(self, rate=""):
        
        #try:
        rospy.loginfo("Start the %s service" % self.name)
        if self.state == State.INIT:
            self.state = State.START

            
        else:
            self.state = State.START

        #except Exception:
        #    rospy.loginfo("Killed the %s service" % self.name)
        return

    #TODO
    def stop(self):
        rospy.loginfo("Stop the %s service" % self.name)
        try:            
            # Signal the STT input data generator to terminate so that the client's
            # streaming_recognize method will not block the process termination.
            self.closed = True
            self._buff.put(None)

            self.state = State.SUCCESS
        except Exception:
            self.state = State.FAILED
        return

    def pause(self):
        rospy.loginfo("Pause the %s service" % self.name)
        self.state = State.SUCCESS
        return

def main():
    """Set names, collect params, and give service to server"""

    service_name = DetectorNameSpace.imageai_custom_yolo.name  # "imageai"
    instance_id = rospy.get_param("instance_id")  # "default"
    service_id = f"{service_name}_{instance_id}"
    try:
        rospy.init_node(service_name, log_level=rospy.DEBUG)

        # stt/default_param/[all your params]
        params = rospy.get_param(service_name + "/" + instance_id + "_param/")

        s = ImageAICustomService(service_id, params)

        service_server = HarmoniServiceServer(name=service_id, service_manager=s)

        print(service_name)
        print("**********************************************************************************************")
        print(service_id)

        #s.start()

        # Streaming audio from mic
        service_server.start_sending_feedback()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()