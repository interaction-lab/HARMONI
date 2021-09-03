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
from imageai.Detection import VideoObjectDetection
from std_msgs.msg import String
import numpy as np
import os
import io
import cv2
from six.moves import queue

class ImageAIYoloService(HarmoniServiceManager):
    """
    ImageAIYolo service
    """

    def __init__(self, name, param):
        super().__init__(name)
        """ Initialization of variables and imageai parameters """
        self.subscriber_id = param["subscriber_id"]
        self.frame_per_second = param["frame_per_second"]
        self.output_file_name = param["output_file_name"]
        self.minimum_percentage_probability = param["minimum_percentage_probability"]
        self.return_detected_frame = param["return_detected_frame"]
        self.log_progress = param["log_progress"]
    
        self.model_path = os.getcwd()
        self.temp_path = os.chdir("/root/harmoni_catkin_ws/src/HARMONI/harmoni_detectors/harmoni_imageai/temp_data")
        self.service_id = hf.get_child_id(self.name)
        self.result_msg = ""

        self._buff = queue.Queue()
        self.closed = False

        """Setup publishers and subscribers"""
        rospy.Subscriber(
            SensorNameSpace.camera.value + self.subscriber_id,
            Image,
            self.callback,
        )

        self.text_pub = rospy.Publisher(
            DetectorNameSpace.imageai.value + self.service_id, String, queue_size=10
        )

        rospy.Subscriber(
            DetectorNameSpace.imageai.value + self.service_id,
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

        if self.state == State.START:
            # rospy.loginfo("Add data to buffer")
            self._buff.put(data.data)
            # rospy.loginfo("Items in buffer: "+ str(self._buff.qsize()))

    
    def imageai_callback(self, data):
        """ Callback function subscribing to the camera topic"""
        self.response_received = True


    #TODO
    def request(self, data):

        rospy.loginfo("Start the %s request" % self.name)
        #self.state = State.REQUEST
        #self.state = State.START
        try:
            #detect objects coming from camera stream
            self.video_path = detector.detectObjectsFromVideo(
                custom_objects=self.custom_objects,
                camera_input=self.camera,
                output_file_path=os.path.join(self.temp_path, self.output_file_name),
                frames_per_second=self.frame_per_second, 
                log_progress=True, 
                per_second_function=self.forSeconds,
                per_frame_function=self.forFrame,
                per_minute_function=self.forMinute,
                minimum_percentage_probability=self.minimum_percentage_probability)

            #r = rospy.Rate(1)
            #while not self.response_received:
            #    r.sleep()

        except rospy.ServiceException:
            self.start = State.FAILED
            self.response_received = True
            rospy.loginfo("Service call failed")
            self.result_msg = ""
        print("Le risposte sono: ")
        print(self.state)
        print(self.result_msg)
        return {"response": self.state, "message": self.result_msg}

    def start(self, rate=""):
        try:
            rospy.loginfo("Start the %s service" % self.name)
            if self.state == State.INIT:
                self.state = State.START
                self.camera = cv2.VideoCapture(self._buff) #TODO link with published camera data
                #In the following 4 lines, we created a new instance of 
                #the VideoObjectDetection class in the first line, 
                #set the model type to YOLOv3 in the second line,
                #set the model path to the YOLOv3 model file in the third line
                #and load the model in the fourth line.
                self.detector = VideoObjectDetection()
                self.detector.setModelTypeAsYOLOv3()
                self.detector.setModelPath(os.path.join(self.model_path, "yolo.h5"))
                self.detector.loadModel()
                #custom_objects refers to the objects we want to detect
                self.custom_objects = detector.CustomObjects(person=True) 
            else:
                self.state = State.START

        except Exception:
            rospy.loginfo("Killed the %s service" % self.name)
        return

    #TODO
    def stop(self):
        rospy.loginfo("Stop the %s service" % self.name)
        try:            
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

    def forFrame(self, frame_number, output_array, output_count):
        print("FOR FRAME " , frame_number)
        print("Output for each object : ", output_array)
        print("Output count for unique objects : ", output_count)
        print("------------END OF A FRAME --------------")

    def forSeconds(self, second_number, output_arrays, count_arrays, average_output_count):
        print("SECOND : ", second_number)
        print("Array for the outputs of each frame ", output_arrays)
        print("Array for output count for unique objects in each frame : ", count_arrays)
        print("Output average count for unique objects in the last second: ", average_output_count)
        print("------------END OF A SECOND --------------")

    def forMinute(self, minute_number, output_arrays, count_arrays, average_output_count):
        print("MINUTE : ", minute_number)
        print("Array for the outputs of each frame ", output_arrays)
        print("Array for output count for unique objects in each frame : ", count_arrays)
        print("Output average count for unique objects in the last minute: ", average_output_count)
        print("------------END OF A MINUTE --------------")

def main():
    """Set names, collect params, and give service to server"""

    service_name = DetectorNameSpace.imageai.name  # "imageai"
    instance_id = rospy.get_param("instance_id")  # "default"
    service_id = f"{service_name}_{instance_id}"
    try:
        rospy.init_node(service_name, log_level=rospy.DEBUG)

        # imageai/default_param/[all your params]
        params = rospy.get_param(service_name + "/" + instance_id + "_param/")

        s = ImageAIYoloService(service_id, params)

        service_server = HarmoniServiceServer(name=service_id, service_manager=s)

        s.start()

        # Streaming audio from mic
        service_server.start_sending_feedback()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()